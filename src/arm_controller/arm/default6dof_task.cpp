#include "default6dof_task.hpp"

#include <Eigen/Geometry>

#include <cmath>

namespace {

/// 旋转角过小视为零旋转，避免除零。
constexpr double kMinRotationAngle = 1e-12;

/**
 * @brief 旋转向量（angle * axis）转 Eigen 轴角表示
 * @param rotation_vector 长度 3 的旋转向量
 * @return 轴角表示；模长过小时返回零旋转
 */
Eigen::AngleAxisd rotation_vector_to_angle_axis(const Eigen::Vector3d& rotation_vector) {
    const double angle = rotation_vector.norm();
    if (angle < kMinRotationAngle) {
        return Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX());
    }
    return Eigen::AngleAxisd(angle, rotation_vector / angle);
}

}  // namespace

Default6DofTaskSpaceMapping::Default6DofTaskSpaceMapping() = default;

bool Default6DofTaskSpaceMapping::position_map(const Eigen::VectorXd& joint_pos,
                                               const Eigen::Isometry3d& pose,
                                               Eigen::VectorXd* task_position) {
    (void)joint_pos;
    if (task_position == nullptr) {
        return false;
    }

    if (task_position->size() != 6) {
        return false;
    }
    task_position->head<3>() = pose.translation();

    Eigen::AngleAxisd rotation(pose.linear());
    if (std::abs(rotation.angle()) < 1e-12) {
        task_position->tail<3>().setZero();
    } else {
        task_position->tail<3>() = rotation.angle() * rotation.axis();
    }

    return task_position->allFinite();
}

bool Default6DofTaskSpaceMapping::jacobian_map(const Eigen::VectorXd& joint_pos,
                                               const Eigen::MatrixXd& jacobian,
                                               Eigen::MatrixXd* task_jacobian) {
    (void)joint_pos;
    if (task_jacobian == nullptr || jacobian.rows() != 6) {
        return false;
    }

    if (task_jacobian->rows() != jacobian.rows() || task_jacobian->cols() != jacobian.cols()) {
        return false;
    }
    task_jacobian->noalias() = jacobian;
    return task_jacobian->allFinite();
}

bool Default6DofTaskSpaceMapping::error_map(const Eigen::VectorXd& target,
                                            const Eigen::VectorXd& current,
                                            Eigen::VectorXd* error) {
    if (error == nullptr || target.size() != 6 || current.size() != 6 || error->size() != 6) {
        return false;
    }

    // 位置部分：直接相减。
    error->head<3>() = target.head<3>() - current.head<3>();

    // 姿态部分：旋转向量逐分量相减在 |rv| ≈ π 处失效——rv 与 -rv 表示同一姿态，
    // 目标与当前一旦落在奇异点两侧，误差会被算成 ≈2π 而不是 ≈0，逆解迭代随即发散。
    // 改用相对旋转 R_err = exp(target_rv) · exp(current_rv)^T，其对数映射的角度恒在 [0, π]。
    const Eigen::AngleAxisd target_rotation  = rotation_vector_to_angle_axis(target.tail<3>());
    const Eigen::AngleAxisd current_rotation = rotation_vector_to_angle_axis(current.tail<3>());
    const Eigen::Matrix3d relative =
        target_rotation.toRotationMatrix() * current_rotation.toRotationMatrix().transpose();
    const Eigen::AngleAxisd relative_rotation(relative);

    if (std::abs(relative_rotation.angle()) < kMinRotationAngle) {
        error->tail<3>().setZero();
    } else {
        error->tail<3>() = relative_rotation.angle() * relative_rotation.axis();
    }

    return error->allFinite();
}
