#include "kinamic.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace {

/// 连续多少次步被拒绝就判定停滞并提前退出，避免在实时回路里空转满迭代上限。
constexpr int kMaxConsecutiveRejections = 8;

}  // namespace

IKSolver::Tuning& IKSolver::tuning() {
    static Tuning instance;
    return instance;
}

IKSolver::IKSolver(std::shared_ptr<ModelBase> robot, std::shared_ptr<TaskMapping> task_mapping)
    : robot_(std::move(robot)), task_mapping_(std::move(task_mapping)) {
    if (robot_ == nullptr) {
        throw std::invalid_argument("IKSolver requires a model");
    }
    if (task_mapping_ == nullptr) {
        throw std::invalid_argument("IKSolver requires a TaskMapping");
    }

    // 任务维由映射决定（4 轴模型 4 维，6 轴模型 6 维），这里不再写死 6。
    task_dim_ = task_mapping_->task_dim();
    if (task_dim_ <= 0) {
        throw std::invalid_argument("TaskMapping::task_dim() must be positive");
    }

    const Eigen::Index joint_count = robot_->dof();
    const Eigen::Index task_dim    = static_cast<Eigen::Index>(task_dim_);
    workspace_.solution.resize(joint_count);
    workspace_.current.resize(task_dim);
    workspace_.error.resize(task_dim);
    workspace_.zero_error.setZero(task_dim);
    workspace_.trial.resize(joint_count);
    workspace_.trial_error.resize(task_dim);
    workspace_.rhs.resize(joint_count);
    workspace_.joint_step.resize(joint_count);
    workspace_.geometric_jacobian.resize(6, joint_count);
    workspace_.task_jacobian.resize(task_dim, joint_count);
    workspace_.hessian.resize(joint_count, joint_count);
    workspace_.hessian.setZero();
    workspace_.ldlt.compute(workspace_.hessian);
}

bool IKSolver::task_position(const Eigen::VectorXd& joint_pos, Eigen::VectorXd* task_position) {
    if (task_position == nullptr || joint_pos.size() != robot_->dof()
        || task_position->size() != static_cast<Eigen::Index>(task_dim_)) {
        return false;
    }

    if (!task_mapping_->position_map(joint_pos, robot_->forward_kinematics(joint_pos), task_position)
        || task_position->size() == 0 || !task_position->allFinite()) {
        return false;
    }
    return true;
}

bool IKSolver::jacobian(
    const Eigen::VectorXd& joint_pos, const Eigen::VectorXd& error, Eigen::MatrixXd* task_jacobian) {
    if (task_jacobian == nullptr || joint_pos.size() != robot_->dof()
        || task_jacobian->rows() != static_cast<Eigen::Index>(task_dim_) || task_jacobian->cols() != robot_->dof()
        || error.size() != static_cast<Eigen::Index>(task_dim_)) {
        return false;
    }

    if (!robot_->geometric_jacobian(joint_pos, &workspace_.geometric_jacobian)
        || workspace_.geometric_jacobian.rows() != 6
        || workspace_.geometric_jacobian.cols() != robot_->dof()
        || !workspace_.geometric_jacobian.allFinite()) {
        return false;
    }

    if (!task_mapping_->jacobian_map(joint_pos, error, workspace_.geometric_jacobian, task_jacobian)
        || task_jacobian->rows() == 0 || task_jacobian->cols() != workspace_.geometric_jacobian.cols()
        || !task_jacobian->allFinite()) {
        return false;
    }

    return true;
}

bool IKSolver::jacobian(const Eigen::VectorXd& joint_pos, Eigen::MatrixXd* task_jacobian) {
    return jacobian(joint_pos, workspace_.zero_error, task_jacobian);
}

bool IKSolver::evaluate_error(
    const Eigen::VectorXd& target, const Eigen::VectorXd& joint_pos, Eigen::VectorXd* error) {
    if (!task_position(joint_pos, &workspace_.current) || workspace_.current.size() != target.size()) {
        return false;
    }
    // 误差交给 TaskMapping 计算：姿态用旋转向量表示时逐分量相减在 |rv| ≈ π 处失效。
    return task_mapping_->error_map(target, workspace_.current, error);
}

bool IKSolver::within_tolerance(const Eigen::VectorXd& error) const {
    if (error.size() != static_cast<Eigen::Index>(task_dim_)) {
        return false;
    }

    const Tuning& config = tuning();
    const Eigen::Index position_dim =
        std::min<Eigen::Index>(task_mapping_->position_dim(), static_cast<Eigen::Index>(task_dim_));
    if (position_dim < 0) {
        return false;
    }

    const double position_norm = error.head(position_dim).norm();
    const double rotation_norm = error.tail(static_cast<Eigen::Index>(task_dim_) - position_dim).norm();
    return position_norm <= config.position_tolerance && rotation_norm <= config.rotation_tolerance;
}

void IKSolver::record_error(const Eigen::VectorXd& error) {
    const Eigen::Index position_dim =
        std::min<Eigen::Index>(task_mapping_->position_dim(), error.size());
    last_error_norm_          = error.norm();
    last_position_error_norm_ = position_dim > 0 ? error.head(position_dim).norm() : 0.0;
    last_rotation_error_norm_ = error.size() > position_dim ? error.tail(error.size() - position_dim).norm() : 0.0;
}

IKSolver::SolveResult IKSolver::solve(const Eigen::VectorXd& target, Eigen::VectorXd& joint_pos) {
    const Tuning& config = tuning();

    // 默认按「数值失败」处理：只有确实停滞在局部极小时才升级为「不可达」，
    // 避免在输入非法等情况下给出关于可达性的错误结论。
    last_result_ = SolveResult::Failed;

    const int joint_count = robot_->dof();
    if (target.size() != static_cast<Eigen::Index>(task_dim_) || joint_pos.size() != joint_count) {
        return SolveResult::Failed;
    }

    last_error_norm_          = 0.0;
    last_position_error_norm_ = 0.0;
    last_rotation_error_norm_ = 0.0;
    last_iterations_          = 0;

    workspace_.solution = joint_pos;
    if (!workspace_.solution.allFinite()) {
        return SolveResult::Failed;
    }

    const Eigen::VectorXd& lower = robot_->lower_joint_limit();
    const Eigen::VectorXd& upper = robot_->upper_joint_limit();
    const bool use_joint_limits = lower.size() == joint_count && upper.size() == joint_count
        && lower.allFinite() && upper.allFinite();

    // 限位截断留出 joint_limit_slack 的余量。实测关节角可能略微超出 URDF 限位
    // （柔性、噪声、仿真软约束），严格截断会把「当前位形」本身变成不可达目标，
    // 逆解必然失败——这正是 cart_traj 曾经一进入就报求解失败的原因之一。
    const double slack = config.joint_limit_slack;
    Eigen::VectorXd effective_lower = lower;
    Eigen::VectorXd effective_upper = upper;
    if (use_joint_limits) {
        effective_lower = lower.array() - slack;
        effective_upper = upper.array() + slack;
        workspace_.solution = workspace_.solution.cwiseMax(effective_lower).cwiseMin(effective_upper);
    }

    if (!evaluate_error(target, workspace_.solution, &workspace_.error)) {
        return SolveResult::Failed;
    }
    record_error(workspace_.error);
    double current_norm = workspace_.error.norm();
    if (within_tolerance(workspace_.error)) {
        joint_pos    = workspace_.solution;
        last_result_ = SolveResult::Converged;
        return SolveResult::Converged;
    }

    double lambda       = config.lambda_initial;
    double trust_radius = config.trust_initial;
    int consecutive_rejections = 0;

    for (int iteration = 0; iteration < config.max_iterations; ++iteration) {
        last_iterations_ = iteration + 1;

        if (!jacobian(workspace_.solution, workspace_.error, &workspace_.task_jacobian)) {
            record_error(workspace_.error);
            return SolveResult::Failed;
        }

        // Levenberg-Marquardt 步：(JᵀJ + λI)⁻¹ Jᵀ e
        workspace_.hessian.noalias() = workspace_.task_jacobian.transpose() * workspace_.task_jacobian;
        workspace_.hessian.diagonal().array() += lambda;
        workspace_.rhs.noalias() = workspace_.task_jacobian.transpose() * workspace_.error;
        workspace_.ldlt.compute(workspace_.hessian);
        workspace_.joint_step = workspace_.ldlt.solve(workspace_.rhs);
        if (!workspace_.joint_step.allFinite()) {
            record_error(workspace_.error);
            return SolveResult::Failed;
        }

        // 信赖域：限制步长范数。接近奇异时 LM 步会非常大，硬截断到某个固定上限会让
        // 迭代来回震荡；按半径等比缩回则保持方向、只缩短步长。
        const double step_norm = workspace_.joint_step.norm();
        if (step_norm > trust_radius) {
            workspace_.joint_step *= trust_radius / step_norm;
        }

        if (use_joint_limits) {
            workspace_.joint_step = workspace_.joint_step
                                        .cwiseMax(effective_lower - workspace_.solution)
                                        .cwiseMin(effective_upper - workspace_.solution);
        }

        workspace_.trial = workspace_.solution + workspace_.joint_step;
        if (!evaluate_error(target, workspace_.trial, &workspace_.trial_error)) {
            record_error(workspace_.error);
            return SolveResult::Failed;
        }
        const double trial_norm = workspace_.trial_error.norm();

        if (trial_norm < current_norm) {
            // 实际下降：接受该步，放宽信赖域、减小阻尼
            workspace_.solution = workspace_.trial;
            workspace_.error    = workspace_.trial_error;
            current_norm        = trial_norm;
            record_error(workspace_.error);
            if (within_tolerance(workspace_.error)) {
                joint_pos    = workspace_.solution;
                last_result_ = SolveResult::Converged;
                return SolveResult::Converged;
            }
            lambda       = std::max(lambda * config.lambda_down, config.lambda_min);
            trust_radius = std::min(trust_radius * config.trust_grow, config.trust_max);
            consecutive_rejections = 0;
        } else {
            // 未下降：拒绝该步，收紧信赖域、加大阻尼后重试
            lambda       = std::min(lambda * config.lambda_up, config.lambda_max);
            trust_radius = std::max(trust_radius * config.trust_shrink, config.trust_min);
            // 停滞提前退出：连续多次无法下降说明已到数值极限（目标与可达集存在微小
            // 不一致时误差降不到零）。此时必须尽快返回，否则会在实时回路里空转满
            // max_iterations，拖慢控制周期、让机械臂跟不上轨迹。
            if (++consecutive_rejections >= kMaxConsecutiveRejections) {
                record_error(workspace_.error);
                // 连续多次找不到下降方向 = 已经停在局部极小。误差降不到判据以内，
                // 说明目标不在可达集内（可达集边界到目标的距离就是 current_norm）。
                last_result_ = within_tolerance(workspace_.error) ? SolveResult::Converged : SolveResult::Unreachable;
                return last_result_;
            }
        }
    }

    record_error(workspace_.error);
    if (within_tolerance(workspace_.error)) {
        joint_pos    = workspace_.solution;
        last_result_ = SolveResult::Converged;
        return SolveResult::Converged;
    }
    // 迭代次数用尽仍未达标。实测目标远远超出工作空间时（如 +X 平移 2 m，残余 1.2 m）
    // 就是这种情况：每步都能略微下降，因此连续拒绝计数永远攒不满，只能靠次数上限退出。
    // 从调用方的角度看这与「停滞」是同一件事——**目标没有达到**，所以同样报 Unreachable，
    // 而不是含糊的「数值失败」。残余由 last_error_norm() 给出，量级自会说明差多远。
    last_result_ = SolveResult::Unreachable;
    return SolveResult::Unreachable;
}
