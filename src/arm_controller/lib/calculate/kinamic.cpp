#include "kinamic.hpp"

#include <stdexcept>
#include <utility>

namespace {

constexpr int kMaxIterations     = 200;
constexpr double kTolerance      = 1e-6;
constexpr double kDamping        = 1e-4;
constexpr double kMaxJointStep   = 0.1;

}  // namespace

IKSolver::IKSolver(std::shared_ptr<ModelBase> robot, std::shared_ptr<TaskMapping> task_mapping)
    : robot_(std::move(robot)), task_mapping_(std::move(task_mapping)) {
    if (robot_ == nullptr) {
        throw std::invalid_argument("IKSolver requires a model");
    }
    if (task_mapping_ == nullptr) {
        throw std::invalid_argument("IKSolver requires a TaskMapping");
    }

    const Eigen::Index joint_count = robot_->dof();
    workspace_.solution.resize(joint_count);
    workspace_.current.resize(6);
    workspace_.error.resize(6);
    workspace_.rhs.resize(joint_count);
    workspace_.joint_step.resize(joint_count);
    workspace_.geometric_jacobian.resize(6, joint_count);
    workspace_.task_jacobian.resize(6, joint_count);
    workspace_.hessian.resize(joint_count, joint_count);
    workspace_.hessian.setZero();
    workspace_.ldlt.compute(workspace_.hessian);
}

bool IKSolver::task_position(const Eigen::VectorXd& joint_pos, Eigen::VectorXd* task_position) {
    if (task_position == nullptr || joint_pos.size() != robot_->dof() || task_position->size() != 6) {
        return false;
    }

    if (!task_mapping_->position_map(joint_pos, robot_->forward_kinematics(joint_pos), task_position)
        || task_position->size() == 0 || !task_position->allFinite()) {
        return false;
    }
    return true;
}

bool IKSolver::jacobian(const Eigen::VectorXd& joint_pos, Eigen::MatrixXd* task_jacobian) {
    if (task_jacobian == nullptr || joint_pos.size() != robot_->dof() || task_jacobian->rows() != 6
        || task_jacobian->cols() != robot_->dof()) {
        return false;
    }

    if (!robot_->geometric_jacobian(joint_pos, &workspace_.geometric_jacobian)
        || workspace_.geometric_jacobian.rows() != 6
        || workspace_.geometric_jacobian.cols() != robot_->dof()
        || !workspace_.geometric_jacobian.allFinite()) {
        return false;
    }

    if (!task_mapping_->jacobian_map(joint_pos, workspace_.geometric_jacobian, task_jacobian)
        || task_jacobian->rows() == 0 || task_jacobian->cols() != workspace_.geometric_jacobian.cols()
        || !task_jacobian->allFinite()) {
        return false;
    }

    return true;
}

bool IKSolver::solve(const Eigen::VectorXd& target, Eigen::VectorXd& joint_pos) {

    const int joint_count = robot_->dof();
    if (target.size() != 6 || joint_pos.size() != joint_count) {
        return false;
    }
    workspace_.solution = joint_pos;
    const Eigen::VectorXd& lower = robot_->lower_joint_limit();
    const Eigen::VectorXd& upper = robot_->upper_joint_limit();
    const bool use_joint_limits = lower.size() == joint_count && upper.size() == joint_count
        && lower.allFinite() && upper.allFinite();

    if (!workspace_.solution.allFinite()) {
        return false;
    }
    if (use_joint_limits) {
        workspace_.solution = workspace_.solution.cwiseMax(lower).cwiseMin(upper);
    }

    for (int iteration = 0; iteration < kMaxIterations; ++iteration) {
        if (!task_position(workspace_.solution, &workspace_.current) || workspace_.current.size() != target.size()) {
            return false;
        }

        // 任务空间误差必须交给 TaskMapping 计算：姿态用旋转向量表示时，逐分量相减
        // 在 |rv| ≈ π 处会得到 ≈2π 的假误差，迭代必然发散（见 TaskMapping::error_map）。
        if (!task_mapping_->error_map(target, workspace_.current, &workspace_.error)) {
            return false;
        }
        if (!workspace_.error.allFinite()) {
            return false;
        }
        if (workspace_.error.norm() < kTolerance) {
            joint_pos = workspace_.solution;
            return true;
        }

        if (!jacobian(workspace_.solution, &workspace_.task_jacobian)
            || workspace_.task_jacobian.rows() != target.size()
            || workspace_.task_jacobian.cols() != joint_count) {
            return false;
        }

        workspace_.hessian.noalias() = workspace_.task_jacobian.transpose() * workspace_.task_jacobian;
        workspace_.hessian.diagonal().array() += kDamping * kDamping;

        workspace_.rhs.noalias() = workspace_.task_jacobian.transpose() * workspace_.error;
        workspace_.ldlt.compute(workspace_.hessian);
        workspace_.joint_step = workspace_.ldlt.solve(workspace_.rhs);
        if (!workspace_.joint_step.allFinite()) {
            return false;
        }

        workspace_.joint_step = workspace_.joint_step.cwiseMax(-kMaxJointStep).cwiseMin(kMaxJointStep);
        if (use_joint_limits) {
            workspace_.joint_step =
                workspace_.joint_step.cwiseMax(lower - workspace_.solution).cwiseMin(upper - workspace_.solution);
        }
        workspace_.solution += workspace_.joint_step;
    }

    if (task_position(workspace_.solution, &workspace_.current) && workspace_.current.size() == target.size()
        && task_mapping_->error_map(target, workspace_.current, &workspace_.error)
        && workspace_.error.norm() < kTolerance) {
        joint_pos = workspace_.solution;
        return true;
    }
    return false;
}
