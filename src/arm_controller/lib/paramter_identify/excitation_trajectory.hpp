#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <string>
#include <thread>

#include <Eigen/Dense>
#include <limits>
#include <pinocchio/algorithm/regressor.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "fourier_trajectory.hpp"

class ExcitationTrajectory{
public:
    explicit ExcitationTrajectory(const std::string& urdf_path);
    ~ExcitationTrajectory();
    void generate(double period, int repeat_cnt);
    bool generate_is_finished();
    bool generate_failed() const;

    /**
     * @brief 固定 CMA-ES 的随机种子（docs/bugs.md O10）。
     *
     * libcmaes 的语义是「seed 为 0 就用当前时间当种子」，所以默认 0 意味着
     * **每次启动生成的激励轨迹都不同、无法重放**。传入非 0 值即可复现同一条轨迹，
     * 这是做任何"换一版模型再测一次"对比实验的前提。
     *
     * @param seed 0 = 随机（保持旧行为），非 0 = 固定。
     */
    void set_seed(std::uint64_t seed);

    bool get_target_position(const rclcpp::Duration &time,Eigen::VectorXd &pos);

    int get_available_param_num() const;

    /// 最近一次生成实际使用的种子（0 表示由 libcmaes 按时间随机）。
    std::uint64_t get_seed() const;
    /// CMA-ES 原始最优解的软限位违例（缩放前），单位 rad；未生成时为 NaN。
    double get_raw_violation() const;
    /// 为满足约束而施加的整体缩放系数（1.0 = 无需缩放）。
    double get_applied_scale() const;
    /// 最终轨迹的软限位违例，应恒为 0；非 0 表示生成失败。
    double get_final_violation() const;
    /// 最终轨迹的激励得分（越大越好）；失败时为 NaN。
    double get_final_score() const;

    double end_effector_x_lower_limit{-std::numeric_limits<double>::infinity()};
    double end_effector_x_upper_limit{std::numeric_limits<double>::infinity()};
    double end_effector_y_lower_limit{-std::numeric_limits<double>::infinity()};
    double end_effector_y_upper_limit{std::numeric_limits<double>::infinity()};
    double end_effector_z_lower_limit{-std::numeric_limits<double>::infinity()};
    double end_effector_z_upper_limit{std::numeric_limits<double>::infinity()};

    /// 指令力矩可行性：RNEA 算出的 |关节目力矩| 不得超过 model_.effortLimit 的这个比例
    /// （<= 0 表示关闭）。实机上力矩超限会直接导致跟踪失败；仿真里会把关节甩到高速，
    /// 记录到的 qacc 出现尖峰（实测最大 7375 rad/s²），辨识数据报废。
    double traj_torque_limit_rate{0.0};

    /// 末端**工具最低点**相对末端 frame 原点的偏移（沿 -z，米）。
    ///
    /// 地面净空约束必须作用在工具最低点上，而不是末端原点：实测 link6 的工具网格比它的
    /// frame 原点低 0.0569 m，只卡原点（z ≥ 0.1）时网格仍会钻进地面 4.6 mm，
    /// MuJoCo 的接触冲量会带来 81.9 N·m 的模型残差——而 URDF 里没有地面，无法解释。
    double end_effector_tool_offset{0.0};

    /// 关节位置软限位比例（相对行程中点，0.7 = 只用中间 70%）。日志里会打出实际生效值。
    double traj_pos_soft_limit_rate{0.7};

private:
    std::atomic_bool trajectory_generatefinished{false};
    std::atomic_bool trajectory_generatefailed{false};

    //评估轨迹可行性和质量
    double traj_score(const FourierTrajectory& traj,double dt,int identifiable_param_num);
    double traj_constraint_violation(const FourierTrajectory& traj,double dt);

    /**
     * @brief 求把系数整体缩放后仍满足全部约束的最大系数（二分）。
     *
     * 可行域是以 0 为内点的**凸集**：q(t) 对傅里叶系数线性，约束是逐关节箱式限位，
     * 因此 λ·c 的违例随 λ∈[0,1] 单调不增。λ=0 必然可行，二分即可保证输出可行。
     * @return 可行区间右端（≤1）；若连 λ=0 都不可行（例如末端箱式约束把原点排除）
     *         则返回 0，调用方应据此判定生成失败。
     */
    double find_feasible_scale(FourierTrajectory& traj, const Eigen::VectorXd& coefficients, double dt);

    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::FrameIndex end_effector_frame_id_{0};


    //参数
    const double SVD_ZERO=1e-6;
    double svd_pos_lower{-2.0};
    double svd_pos_upper{2.0};
    double svd_vel_upper{4.0};
    double svd_acc_upper{6.0};
    int svd_Y_stack_num{30};    //堆叠30个Y矩阵评估矩阵特性

    // O10：随机种子与最近一次生成的诊断量（生成线程写、实时线程读，故用原子量）
    std::atomic<std::uint64_t> seed_{0};
    std::atomic<double> last_raw_violation_{std::numeric_limits<double>::quiet_NaN()};
    std::atomic<double> last_applied_scale_{1.0};
    std::atomic<double> last_final_violation_{std::numeric_limits<double>::quiet_NaN()};
    std::atomic<double> last_final_score_{std::numeric_limits<double>::quiet_NaN()};

    //缓存
    Eigen::VectorXd q, dq, ddq;
    Eigen::MatrixXd Y;
    Eigen::VectorXd S;
    Eigen::MatrixXd best_coefficients_;
    Eigen::MatrixXd param_mat;

    int identifiable_parameter_num_{0};

    std::mutex start_calc_mtx_;
    std::condition_variable start_calc_cv_;
    bool start_calc_flag{false};
    std::atomic_bool exit_thread{false};
    void calc_func();

    std::shared_ptr<FourierTrajectory> best_traj;
    int traj_repeat_cnt{1};
    double period_s{10.0};

    std::thread calc_thread_;
};
