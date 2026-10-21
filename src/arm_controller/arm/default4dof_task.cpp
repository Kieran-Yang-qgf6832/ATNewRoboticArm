#include "default4dof_task.hpp"

#include <cmath>

namespace {

/// 任务空间维度：[x, y, z, q4]
constexpr Eigen::Index kTaskDim = 4;

/// 位置分量个数（前三个）
constexpr Eigen::Index kPositionDim = 3;

/// 2*pi，用于把角度差归一化到 [-pi, pi]
constexpr double kTwoPi = 2.0 * 3.14159265358979323846;

}  // namespace

Default4DofTaskSpaceMapping::Default4DofTaskSpaceMapping() = default;

bool Default4DofTaskSpaceMapping::position_map(const Eigen::VectorXd& joint_pos,
                                               const Eigen::Isometry3d& pose,
                                               Eigen::VectorXd* task_position) {
    if (task_position == nullptr || task_position->size() != kTaskDim || joint_pos.size() < kTaskDim) {
        return false;
    }

    task_position->head<kPositionDim>() = pose.translation();
    (*task_position)(3)                = joint_pos(3);
    return task_position->allFinite();
}

bool Default4DofTaskSpaceMapping::jacobian_map(const Eigen::VectorXd& joint_pos,
                                               const Eigen::VectorXd& error,
                                               const Eigen::MatrixXd& jacobian,
                                               Eigen::MatrixXd* task_jacobian) {
    (void)joint_pos;
    if (task_jacobian == nullptr || jacobian.rows() != 6 || jacobian.cols() < kTaskDim || error.size() != kTaskDim) {
        return false;
    }
    if (task_jacobian->rows() != kTaskDim || task_jacobian->cols() != jacobian.cols()) {
        return false;
    }

    // 位置部分：e_p = p_t - p_c，∂e_p/∂q = -J_v，故 GN 雅可比 = +J_v。
    task_jacobian->topRows(kPositionDim) = jacobian.topRows(kPositionDim);

    // 自转部分：e_4 = wrap(q4_t - q4_c)，∂e_4/∂q4 = -1，故 GN 雅可比第 4 行是
    // 「选中 joint4」的单位行。注意它不是几何雅可比第 4 列（那一列是工具轴的角速度），
    // 两者只在「任务坐标 = 关节角」这个语义下等价。
    task_jacobian->row(3).setZero();
    (*task_jacobian)(3, 3) = 1.0;

    return task_jacobian->allFinite();
}

bool Default4DofTaskSpaceMapping::error_map(const Eigen::VectorXd& target,
                                            const Eigen::VectorXd& current,
                                            Eigen::VectorXd* error) {
    if (error == nullptr || target.size() != kTaskDim || current.size() != kTaskDim || error->size() != kTaskDim) {
        return false;
    }

    error->head<kPositionDim>() = target.head<kPositionDim>() - current.head<kPositionDim>();

    // 自转角取最短路径：目标与当前跨过 ±pi 时直接相减会得到 ≈2pi 的假误差。
    const double delta = target(3) - current(3);
    (*error)(3)        = std::remainder(delta, kTwoPi);

    return error->allFinite();
}

bool Default4DofTaskSpaceMapping::target_from_pose(const Eigen::VectorXd& joint_pos,
                                                   const Eigen::VectorXd& pose,
                                                   Eigen::VectorXd* target) const {
    if (target == nullptr || target->size() != kTaskDim || pose.size() != 6 || joint_pos.size() < kTaskDim) {
        return false;
    }

    target->head<kPositionDim>() = pose.head<kPositionDim>();
    (*target)(3)                 = joint_pos(3);
    return target->allFinite();
}
