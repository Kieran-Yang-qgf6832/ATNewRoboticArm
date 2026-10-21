#include "arm.hpp"

#include <sstream>
#include <stdexcept>

ArmSolve::ArmSolve(std::shared_ptr<ModelBase> model, std::shared_ptr<TaskMapping> task_mapping)
    : model_(model) {
    ik_solver_ = std::make_shared<IKSolver>(model, task_mapping);

    // 任务维由映射决定（4 轴模型 4 维，6 轴模型 6 维）：cart_traj 的力矩前馈
    // 走的就是 inverse_dynamic，所以这里必须跟着任务维走，不能写死 6。
    task_dim_ = ik_solver_->task_dim();
    if (task_dim_ <= 0) {
        throw std::invalid_argument("TaskMapping::task_dim() must be positive");
    }

    const Eigen::Index joint_count = model_->dof();
    const Eigen::Index task_dim    = static_cast<Eigen::Index>(task_dim_);
    workspace_.finite_joint_pos.resize(joint_count);
    workspace_.joint_vel.resize(joint_count);
    workspace_.jacobian_velocity.resize(task_dim);
    workspace_.task_acc_rhs.resize(task_dim);
    workspace_.joint_acc.resize(joint_count);
    workspace_.task_force_torque.resize(joint_count);
    workspace_.jacobian.resize(task_dim, joint_count);
    workspace_.next_jacobian.resize(task_dim, joint_count);
    workspace_.jacobian_delta.resize(task_dim, joint_count);
    workspace_.jacobian_transpose.resize(joint_count, task_dim);
    workspace_.jacobian.setZero();
    workspace_.next_jacobian.setZero();
    workspace_.jacobian_delta.setZero();
    workspace_.jacobian_transpose.setZero();
    workspace_.jacobian_solver.compute(workspace_.jacobian);
    workspace_.static_force_solver.compute(workspace_.jacobian_transpose);
}

bool ArmSolve::inverse_kinamic(const Eigen::VectorXd& task_pos, Eigen::VectorXd* joint_pos) {
    if (joint_pos == nullptr) {
        return false;
    }

    const int joint_count = model_->dof();
    if (joint_pos->size() != joint_count) {
        return false;
    }
    // 以调用方传入的关节角作为迭代初值（不在此处清零）：调用方应传入当前实测关节角，
    // 让迭代从离目标最近的位形出发。若从零位形起步，当目标位姿远离零位形或接近奇异时，
    // 阻尼最小二乘迭代会在迭代上限内不收敛而返回 false，进而让控制器 update() 报错。
    // 初值的限幅（关节限位）与有限性检查由 IKSolver::solve 负责。
    return ik_solver_->solve(task_pos, *joint_pos) == IKSolver::SolveResult::Converged;
}

IKSolver::SolveResult ArmSolve::last_ik_result() const {
    return ik_solver_ != nullptr ? ik_solver_->last_result() : IKSolver::SolveResult::Failed;
}

bool ArmSolve::forward_kinamic(const Eigen::VectorXd& joint_pos, Eigen::VectorXd* task_pos) {
    return ik_solver_->task_position(joint_pos, task_pos);
}

double ArmSolve::last_ik_error_norm() const {
    return ik_solver_ != nullptr ? ik_solver_->last_error_norm() : 0.0;
}

double ArmSolve::last_ik_position_error_norm() const {
    return ik_solver_ != nullptr ? ik_solver_->last_position_error_norm() : 0.0;
}

double ArmSolve::last_ik_rotation_error_norm() const {
    return ik_solver_ != nullptr ? ik_solver_->last_rotation_error_norm() : 0.0;
}

int ArmSolve::last_ik_iterations() const {
    return ik_solver_ != nullptr ? ik_solver_->last_iterations() : 0;
}

bool ArmSolve::inverse_dynamic(
    const Eigen::VectorXd& joint_pos,
    const Eigen::VectorXd& task_vel,
    const Eigen::VectorXd& task_acc,
    const Eigen::VectorXd& task_force,
    Eigen::VectorXd* joint_torque) {
    if (joint_torque == nullptr) {
        return false;
    }

    const int joint_count = model_->dof();
    if (joint_pos.size() != joint_count) {
        std::ostringstream message;
        message << "Invalid joint position size: expected " << joint_count << ", got " << joint_pos.size();
        throw std::invalid_argument(message.str());
    }
    if (!joint_pos.allFinite()) {
        throw std::invalid_argument("Joint position contains non-finite values");
    }

    constexpr double kJacobianDifferenceStep = 1e-6;

    if (!ik_solver_->jacobian(joint_pos, &workspace_.jacobian)) {
        return false;
    }
    if (task_vel.size() != workspace_.jacobian.rows()) {
        std::ostringstream message;
        message << "Invalid task velocity size: expected " << workspace_.jacobian.rows() << ", got " << task_vel.size();
        throw std::invalid_argument(message.str());
    }
    if (task_acc.size() != workspace_.jacobian.rows()) {
        std::ostringstream message;
        message << "Invalid task acceleration size: expected " << workspace_.jacobian.rows() << ", got " << task_acc.size();
        throw std::invalid_argument(message.str());
    }
    if (task_force.size() != workspace_.jacobian.rows()) {
        std::ostringstream message;
        message << "Invalid task force size: expected " << workspace_.jacobian.rows() << ", got " << task_force.size();
        throw std::invalid_argument(message.str());
    }
    if (joint_torque->size() != joint_count) {
        throw std::invalid_argument("Invalid joint torque output size");
    }
    workspace_.jacobian_solver.compute(workspace_.jacobian);

    workspace_.joint_vel = workspace_.jacobian_solver.solve(task_vel);
    if (!workspace_.joint_vel.allFinite()) {
        return false;
    }

    // x_ddot = J(q) q_ddot + J_dot(q, q_dot) q_dot.
    workspace_.finite_joint_pos.noalias() = joint_pos + kJacobianDifferenceStep * workspace_.joint_vel;
    if (!ik_solver_->jacobian(workspace_.finite_joint_pos, &workspace_.next_jacobian)) {
        return false;
    }

    workspace_.jacobian_delta.noalias() = workspace_.next_jacobian - workspace_.jacobian;
    workspace_.jacobian_delta /= kJacobianDifferenceStep;

    workspace_.jacobian_velocity.noalias() = workspace_.jacobian_delta * workspace_.joint_vel;

    workspace_.task_acc_rhs.noalias() = task_acc - workspace_.jacobian_velocity;
    workspace_.joint_acc = workspace_.jacobian_solver.solve(workspace_.task_acc_rhs);
    if (!workspace_.joint_acc.allFinite()) {
        return false;
    }

    if (!model_->inverse_dynamic(joint_pos, workspace_.joint_vel, workspace_.joint_acc, joint_torque)) {
        return false;
    }

    workspace_.task_force_torque.noalias() = workspace_.jacobian.transpose() * task_force;
    *joint_torque += workspace_.task_force_torque;
    return joint_torque->allFinite();
}

bool ArmSolve::inverse_velocity(
    const Eigen::VectorXd& joint_pos, const Eigen::VectorXd& task_vel, Eigen::VectorXd* joint_vel) {
    if (joint_vel == nullptr) {
        return false;
    }

    const int joint_count = model_->dof();
    if (joint_pos.size() != joint_count) {
        throw std::invalid_argument("Invalid joint position size");
    }
    if (!joint_pos.allFinite()) {
        throw std::invalid_argument("Joint position contains non-finite values");
    }
    if (!ik_solver_->jacobian(joint_pos, &workspace_.jacobian)) {
        return false;
    }
    if (task_vel.size() != workspace_.jacobian.rows()) {
        throw std::invalid_argument("Invalid task velocity size");
    }

    workspace_.jacobian_solver.compute(workspace_.jacobian);
    workspace_.joint_vel.noalias() = workspace_.jacobian_solver.solve(task_vel);
    if (!workspace_.joint_vel.allFinite()) {
        return false;
    }
    if (joint_vel->size() != joint_count) {
        return false;
    }
    *joint_vel = workspace_.joint_vel;
    return true;
}

bool ArmSolve::static_force(const Eigen::VectorXd& joint_pos,
                            const Eigen::VectorXd& joint_torque_residual,
                            Eigen::VectorXd* task_force) {
    if (task_force == nullptr) {
        return false;
    }

    const int joint_count = model_->dof();
    if (joint_pos.size() != joint_count) {
        throw std::invalid_argument("Invalid joint position size");
    }
    if (joint_torque_residual.size() != joint_count) {
        std::ostringstream message;
        message << "Invalid joint torque residual size: expected " << joint_count << ", got " << joint_torque_residual.size();
        throw std::invalid_argument(message.str());
    }
    if (task_force->size() != static_cast<Eigen::Index>(task_dim_)) {
        throw std::invalid_argument("Invalid task force output size");
    }

    if (!ik_solver_->jacobian(joint_pos, &workspace_.jacobian)) {
        return false;
    }

    workspace_.jacobian_transpose.noalias() = workspace_.jacobian.transpose();
    workspace_.static_force_solver.compute(workspace_.jacobian_transpose);
    *task_force = workspace_.static_force_solver.solve(joint_torque_residual);
    return task_force->allFinite();
}
