#include "excitation_trajectory.hpp"

#include <Eigen/SVD>
// 力矩可行性约束要用 RNEA（traj_constraint_violation 里）。
#include <pinocchio/algorithm/rnea.hpp>

#include <Eigen/src/Core/Matrix.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>

#include <libcmaes/cmaes.h>
#include <pinocchio/algorithm/frames.hpp>

ExcitationTrajectory::ExcitationTrajectory(const std::string& urdf_path) {
    pinocchio::urdf::buildModel(urdf_path, model_);
    data_ = pinocchio::Data(model_);
    if (!model_.existFrame("link6")) {
        throw std::runtime_error("The URDF model does not contain the end-effector frame: link6");
    }
    end_effector_frame_id_ = model_.getFrameId("link6");

    q.resize(model_.nq);
    dq.resize(model_.nv);
    ddq.resize(model_.nv);
    calc_thread_ = std::thread([this]() { calc_func(); });
}

ExcitationTrajectory::~ExcitationTrajectory() {
    {
        std::unique_lock<std::mutex> lock(start_calc_mtx_);
        exit_thread     = true;
        start_calc_flag = true;
    }
    start_calc_cv_.notify_one();
    if (calc_thread_.joinable()) {
        calc_thread_.join();
    }
}

void ExcitationTrajectory::generate(double period, int repeat_cnt) {
    traj_repeat_cnt             = repeat_cnt;
    period_s                    = period;
    trajectory_generatefinished = false;
    trajectory_generatefailed   = false;
    std::unique_lock<std::mutex> lock(start_calc_mtx_);
    start_calc_flag = true;
    start_calc_cv_.notify_one();
}
bool ExcitationTrajectory::generate_is_finished() {
    return trajectory_generatefinished;
}

bool ExcitationTrajectory::generate_failed() const {
    return trajectory_generatefailed;
}

bool ExcitationTrajectory::get_target_position(const rclcpp::Duration& time, Eigen::VectorXd& pos) {
    if (best_traj == nullptr) {
        return false;
    }

    return best_traj->position(std::fmod(time.seconds(), best_traj->period()), pos);
}

void ExcitationTrajectory::calc_func() {

    // O10：原来 k_max_fevals = 600，而优化变量是 nq x 2 x harmonics = 36 维，
    // 600 次评估只有 17 倍维度——CMA-ES 远未收敛，返回的基本是"随机可行解"。
    // 同时 k_initial_sigma = 0.7 让初始种群几乎全部违反软限位（joint5 的软限位
    // 只有 ±0.428 rad），目标函数被惩罚项淹没、优化信息全丢。
    // 现在：1500 次评估（约 42 倍维度）+ 更小的初始 sigma，让初始种群落在可行域内。
    constexpr int k_harmonics           = 3;
    constexpr double k_eval_dt          = 0.01;
    constexpr int k_max_fevals          = 1500;
    constexpr int k_lambda              = 24;
    constexpr double k_coeff_bound      = 2.0;
    constexpr double k_initial_sigma    = 0.25;
    constexpr double k_infeasible_cost  = 1000.0;
    constexpr double k_violation_weight = 1000.0;
    // 单次评估的采样步长：约束检查要够密，但长周期不能把评估次数放大到不可接受。
    // T = 10 s 时仍是 0.01（1000 点/周期，与原实现一致）；T = 120 s 时放宽到 0.06
    // （2000 点/周期，仍远密于最快谐波 3/T 的周期 40 s）。
    const double eval_dt = std::max(k_eval_dt, period_s / 2000.0);


    while (1) {
        std::unique_lock<std::mutex> lock(start_calc_mtx_);
        start_calc_cv_.wait(lock, [this] { return start_calc_flag || exit_thread; });
        start_calc_flag = false;
        if (exit_thread) {
            return;
        }
        lock.unlock();

        // SVD 分解，评估最小参数集
        std::random_device rd;
        std::seed_seq seq{rd(), rd(), rd(), rd(), rd(), rd(), rd(), rd()};
        std::mt19937 random_genenter(seq);

        std::uniform_real_distribution<double> pos_range(svd_pos_lower, svd_pos_upper);
        std::uniform_real_distribution<double> vel_range(-svd_vel_upper, svd_vel_upper);
        std::uniform_real_distribution<double> acc_range(-svd_acc_upper, svd_acc_upper);

        // 填写随机参数
        for (int i = 0; i < svd_Y_stack_num; i++) {
            for (int j = 0; j < model_.nv; j++) {
                q[j]   = pos_range(random_genenter);
                dq[j]  = vel_range(random_genenter);
                ddq[j] = acc_range(random_genenter);
            }

            const Eigen::MatrixXd& Yi = pinocchio::computeJointTorqueRegressor(model_, data_, q, dq, ddq);
            if (i == 0) {
                Y.resize(Yi.rows() * svd_Y_stack_num, Yi.cols());
            }
            Y.block(i * Yi.rows(), 0, Yi.rows(), Yi.cols()) = Yi;
        }

        Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeFullV);
        const Eigen::VectorXd singular_values = svd.singularValues();
        identifiable_parameter_num_           = 0;
        for (int i = 0; i < singular_values.size(); i++) {
            if (singular_values[i] > SVD_ZERO * singular_values[0]) {
                identifiable_parameter_num_++;
            }
        }


        if (identifiable_parameter_num_ <= 0) {
            std::cerr << "No identifiable dynamic parameters found, skip excitation trajectory optimization." << std::endl;
            trajectory_generatefailed   = true;
            trajectory_generatefinished = true;
            continue;
        }

        // 3次谐波，关节个数维度，一个周期
        FourierTrajectory fourier_traj(model_.nq, k_harmonics, period_s);
        const int coefficient_num = model_.nq * 2 * fourier_traj.harmonics();
        std::vector<double> x0(coefficient_num, 0.0);
        std::vector<double> lbounds(coefficient_num, -k_coeff_bound);
        std::vector<double> ubounds(coefficient_num, k_coeff_bound);

        libcmaes::GenoPheno<libcmaes::pwqBoundStrategy> gp(lbounds.data(), ubounds.data(), coefficient_num);
        // O10：seed 为 0 时 libcmaes 会拿"当前时间"当种子（parameters.h:80），
        // 于是每次启动的激励轨迹都不同、无法重放。这里透传 measure_trajectory_seed。
        const std::uint64_t seed = seed_.load(std::memory_order_relaxed);
        libcmaes::CMAParameters<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy>> cma_params(x0, k_initial_sigma, k_lambda, seed, gp);
        cma_params.set_algo(aCMAES);
        cma_params.set_max_fevals(k_max_fevals);
        cma_params.set_quiet(true);
        cma_params.set_mt_feval(false);

        double best_score = -std::numeric_limits<double>::infinity();
        Eigen::MatrixXd best_mat;

        libcmaes::FitFunc objective = [this, &fourier_traj, &best_score, &best_mat, eval_dt, k_infeasible_cost,
                                       k_violation_weight](const double* x, const int n) {
            if (n != fourier_traj.dof() * 2 * fourier_traj.harmonics()) {
                return k_infeasible_cost;
            }

            fourier_traj.set_coefficients(x, n);

            const double violation = traj_constraint_violation(fourier_traj, eval_dt);
            if (violation > 0.0) {
                return k_infeasible_cost + k_violation_weight * violation;
            }

            const double score = traj_score(fourier_traj, eval_dt, identifiable_parameter_num_);
            if (!std::isfinite(score)) {
                return k_infeasible_cost;
            }
            return -score;
        };

        const libcmaes::CMASolutions cma_solutions =
            libcmaes::cmaes<libcmaes::GenoPheno<libcmaes::pwqBoundStrategy>>(objective, cma_params);
        Eigen::VectorXd best_x = cma_solutions.get_best_seen_candidate().get_x_pheno_dvec(cma_params);
        fourier_traj.set_coefficients(best_x);

        // O10 的关键修正：CMA-ES 交回来的"最优"候选**经常仍然违反软限位**——实测 21 次
        // 测量无一例外，违例 0.12 ~ 6.68 rad。指令远超行程时 PID 会在 20 N·m 上限饱和，
        // 实际关节角被甩出硬限位（实测 joint6 行程 15.4 rad、速度峰值 1064 rad/s），
        // 记录下来的数据直接报废。
        // 可行域是以 0 为内点的凸集，所以按比例整体缩小一定能回到可行域：二分出最大的
        // 可行缩放系数，**保证输出轨迹一定可行**，而不是把"最不违反"的样本发下去。
        const double raw_violation = traj_constraint_violation(fourier_traj, eval_dt);
        double scale               = 1.0;
        if (!(raw_violation <= 0.0)) {
            scale = find_feasible_scale(fourier_traj, best_x, eval_dt);
            best_x *= scale;
            fourier_traj.set_coefficients(best_x);
        }

        const double final_violation = traj_constraint_violation(fourier_traj, eval_dt);
        double final_score           = std::numeric_limits<double>::quiet_NaN();
        if (final_violation <= 0.0) {
            final_score = traj_score(fourier_traj, eval_dt, identifiable_parameter_num_);
        }

        last_raw_violation_.store(raw_violation, std::memory_order_relaxed);
        last_applied_scale_.store(scale, std::memory_order_relaxed);
        last_final_violation_.store(final_violation, std::memory_order_relaxed);
        last_final_score_.store(final_score, std::memory_order_relaxed);

        best_traj = std::make_shared<FourierTrajectory>(fourier_traj);
        if (std::isfinite(best_score)) {
            best_coefficients_ = best_mat;
        }
        std::cout << "excitation: seed=" << seed << " raw_violation=" << raw_violation << " scale=" << scale
                  << " violation=" << final_violation << " score=" << final_score << std::endl;
        std::cout << "best_x:" << best_x << std::endl;
        // 违例仍不为 0 只可能是连 q≡0 都不可行（例如末端箱式约束把原点排除），
        // 这时必须**显式失败**，而不是把一条不可行的轨迹发下去。
        trajectory_generatefailed   = best_traj == nullptr || !(final_violation <= 0.0);
        trajectory_generatefinished = true;
    }
}

int ExcitationTrajectory::get_available_param_num()const {
    return identifiable_parameter_num_;
}

void ExcitationTrajectory::set_seed(std::uint64_t seed) {
    seed_.store(seed, std::memory_order_relaxed);
}

std::uint64_t ExcitationTrajectory::get_seed() const {
    return seed_.load(std::memory_order_relaxed);
}

double ExcitationTrajectory::get_raw_violation() const {
    return last_raw_violation_.load(std::memory_order_relaxed);
}

double ExcitationTrajectory::get_applied_scale() const {
    return last_applied_scale_.load(std::memory_order_relaxed);
}

double ExcitationTrajectory::get_final_violation() const {
    return last_final_violation_.load(std::memory_order_relaxed);
}

double ExcitationTrajectory::get_final_score() const {
    return last_final_score_.load(std::memory_order_relaxed);
}

double ExcitationTrajectory::find_feasible_scale(FourierTrajectory& traj, const Eigen::VectorXd& coefficients, double dt) {
    // λ = 0 时 q(t) ≡ 0，位置与速度约束必然满足；λ 增大时违例单调不减，
    // 所以直接二分可行区间的右端。若连 λ = 0 都不可行（末端箱式约束把原点排除），
    // 返回 0，由调用方判定生成失败。
    double feasible   = 0.0;
    double infeasible = 1.0;
    for (int i = 0; i < 40; ++i) {
        const double mid            = 0.5 * (feasible + infeasible);
        const Eigen::VectorXd trial = mid * coefficients;
        traj.set_coefficients(trial);
        if (traj_constraint_violation(traj, dt) <= 0.0) {
            feasible = mid;
        } else {
            infeasible = mid;
        }
    }
    traj.set_coefficients(coefficients);
    return feasible;
}

double ExcitationTrajectory::traj_constraint_violation(const FourierTrajectory& traj, double dt) {
    // 轨迹关节数与模型不一致时视为不可用
    if (traj.dof() != model_.nq) {
        return std::numeric_limits<double>::infinity();
    }

    // 傅里叶轨迹为周期轨迹，采样一个周期即可覆盖全部位置/速度取值
    const int sample_cnt = static_cast<int>(traj.period() / dt);
    double violation     = 0.0;

    for (int i = 0; i <= sample_cnt; ++i) {
        const double t = i * dt;
        q              = traj.position(t);
        dq             = traj.velocity(t);

        // 指令力矩可行性：用 RNEA 算这条**指令**轨迹需要的关节力矩，超过
        // effortLimit * traj_torque_limit_rate 就惩罚。放在位置/速度检查之前，
        // 这样 CMA-ES 能同时看到"关节走得到"和"电机给得出"。
        if (traj_torque_limit_rate > 0.0) {
            ddq = traj.acceleration(t);
            pinocchio::rnea(model_, data_, q, dq, ddq);
            for (int j = 0; j < traj.dof(); ++j) {
                const double effort_limit = model_.effortLimit[static_cast<Eigen::Index>(j)];
                if (!std::isfinite(effort_limit) || effort_limit <= 0.0) {
                    continue;
                }
                const double allowed = traj_torque_limit_rate * effort_limit;
                const double needed  = std::fabs(data_.tau[static_cast<Eigen::Index>(j)]);
                if (needed > allowed) {
                    const double normalized_violation = (needed - allowed) / allowed;
                    violation += normalized_violation * normalized_violation;
                }
            }
        }

        for (int j = 0; j < traj.dof(); ++j) {
            // 位置软限位：将关节行程按软限位比例缩放到行程中点附近
            const double lower = model_.lowerPositionLimit[j];
            const double upper = model_.upperPositionLimit[j];
            if (std::isfinite(lower) && std::isfinite(upper) && upper > lower) {
                const double mid        = 0.5 * (lower + upper);
                const double half_range = 0.5 * (upper - lower);
                const double soft_lower = mid - traj_pos_soft_limit_rate * half_range;
                const double soft_upper = mid + traj_pos_soft_limit_rate * half_range;
                const double pos_scale  = std::max(traj_pos_soft_limit_rate * half_range, 1e-6);

                if (q[j] < soft_lower) {
                    const double normalized_violation = (soft_lower - q[j]) / pos_scale;
                    violation += normalized_violation * normalized_violation;
                } else if (q[j] > soft_upper) {
                    const double normalized_violation = (q[j] - soft_upper) / pos_scale;
                    violation += normalized_violation * normalized_violation;
                }
            }

            // 关节速度超限
            const double velocity_limit = model_.velocityLimit[j];
            if (std::isfinite(velocity_limit) && velocity_limit > 0.0 && std::fabs(dq[j]) > velocity_limit) {
                const double normalized_violation = (std::fabs(dq[j]) - velocity_limit) / velocity_limit;
                violation += normalized_violation * normalized_violation;
            }
        }

        pinocchio::framesForwardKinematics(model_, data_, q);
        Eigen::Vector3d end_effector_position = data_.oMf[end_effector_frame_id_].translation();
        // 地面净空要卡在**工具最低点**上：工具比末端原点低 end_effector_tool_offset。
        // 只卡原点会让工具网格钻进地面（实测 -4.6 mm），触发 MuJoCo 接触冲量。
        end_effector_position[2] -= end_effector_tool_offset;
        if (!end_effector_position.allFinite()) {
            return std::numeric_limits<double>::infinity();
        }
        const double lower_limits[] = {
            end_effector_x_lower_limit,
            end_effector_y_lower_limit,
            end_effector_z_lower_limit,
        };
        const double upper_limits[] = {
            end_effector_x_upper_limit,
            end_effector_y_upper_limit,
            end_effector_z_upper_limit,
        };
        for (std::size_t axis = 0; axis < 3; ++axis) {
            const double lower = lower_limits[axis];
            const double upper = upper_limits[axis];
            if (std::isnan(lower) || std::isnan(upper) || lower > upper
                || (std::isinf(lower) && lower > 0.0) || (std::isinf(upper) && upper < 0.0)) {
                return std::numeric_limits<double>::infinity();
            }
            const double range = std::isfinite(lower) && std::isfinite(upper) ? std::max(upper - lower, 1e-6) : 1.0;
            if (std::isfinite(lower) && end_effector_position[static_cast<Eigen::Index>(axis)] < lower) {
                const double normalized_violation =
                    (lower - end_effector_position[static_cast<Eigen::Index>(axis)]) / range;
                violation += normalized_violation * normalized_violation;
            } else if (std::isfinite(upper) && end_effector_position[static_cast<Eigen::Index>(axis)] > upper) {
                const double normalized_violation =
                    (end_effector_position[static_cast<Eigen::Index>(axis)] - upper) / range;
                violation += normalized_violation * normalized_violation;
            }
        }
    }

    return violation;
}

double ExcitationTrajectory::traj_score(const FourierTrajectory& traj, double dt, int identifiable_param_num) // 求激励轨迹的得分
{
    const int sample_cnt = static_cast<int>(traj.period() / dt);
    for (int i = 0; i < sample_cnt; i++) {
        q   = traj.position(i * dt);
        dq  = traj.velocity(i * dt);
        ddq = traj.acceleration(i * dt);

        const Eigen::MatrixXd& Yi = pinocchio::computeJointTorqueRegressor(model_, data_, q, dq, ddq);
        if (i == 0) {
            Y.resize(sample_cnt * Yi.rows(), Yi.cols());
        }
        Y.block(i * Yi.rows(), 0, Yi.rows(), Yi.cols()) = Yi;
    }
    // 进行SVD，对轨迹质量进行评分
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Y, Eigen::ComputeThinV);
    S = svd.singularValues();

    double ret                = 0.0f;
    const int score_param_num = std::min<int>(identifiable_param_num, S.size());
    if (score_param_num <= 0) {
        return -std::numeric_limits<double>::infinity();
    }
    for (int i = 0; i < score_param_num; i++)
        ret = ret + std::log10(std::max(S[i], SVD_ZERO));
    return ret / score_param_num;
}
