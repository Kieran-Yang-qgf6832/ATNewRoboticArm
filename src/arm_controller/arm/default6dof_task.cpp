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


/// 小于该角度时改用泰勒展开，避免 (1-cos θ)/θ² 与 (θ-sin θ)/θ³ 的数值相消。
constexpr double kSmallAngleForSeries = 1e-4;

/// 三维向量的反对称矩阵 [v]×
Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d& v) {
    Eigen::Matrix3d matrix;
    matrix << 0.0, -v.z(), v.y(), v.z(), 0.0, -v.x(), -v.y(), v.x(), 0.0;
    return matrix;
}

/**
 * @brief 右雅可比 J_r(ρ)：把「体坐标系角速度」换算成旋转向量变化率
 *
 * @code
 *   J_r(ρ) = I - (1-cos θ)/θ² [ρ]× + (θ-sin θ)/θ³ [ρ]×²
 * @endcode
 * θ = π 处两项系数分别为 2/π² 与 1/π²，**并不奇异**；真正数值不稳的是 θ → 0
 * （两项相消），故小角度改用泰勒极限：@c I - ½[ρ]× + (1/6)[ρ]×²。
 *
 * @param rho 旋转向量（模长即旋转角，由 AngleAxisd 得到时恒在 [0, π]）
 * @return J_r(ρ)
 */
Eigen::Matrix3d right_jacobian(const Eigen::Vector3d& rho) {
    const double theta         = rho.norm();
    const Eigen::Matrix3d skew = skew_symmetric(rho);

    double first;
    double second;
    if (theta < kSmallAngleForSeries) {
        first  = 0.5;
        second = 1.0 / 6.0;
    } else {
        first  = (1.0 - std::cos(theta)) / (theta * theta);
        second = (theta - std::sin(theta)) / (theta * theta * theta);
    }

    return Eigen::Matrix3d::Identity() - first * skew + second * (skew * skew);
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
                                               const Eigen::VectorXd& error,
                                               const Eigen::MatrixXd& jacobian,
                                               Eigen::MatrixXd* task_jacobian) {
    (void)joint_pos;
    if (task_jacobian == nullptr || jacobian.rows() != 6 || error.size() != 6) {
        return false;
    }

    if (task_jacobian->rows() != jacobian.rows() || task_jacobian->cols() != jacobian.cols()) {
        return false;
    }

    // 位置部分：e_p = p_t - p_c，∂e_p/∂q = -J_v，故 GN 雅可比 = +J_v，直接照搬。
    task_jacobian->topRows<3>() = jacobian.topRows<3>();

    // 姿态部分：e_r = log(R_t R_cᵀ)，de_r/dt = -J_r(ρ_e)·ω_c，故 GN 雅可比
    // = +J_r(ρ_e)·J_geom_rot。少了这一步等于默认 J_r = I，只在小误差下成立；
    // 误差接近 π 时线性化失准，逆解会停滞在判据之上（见 docs/bugs.md O5）。
    const Eigen::Matrix3d rotation_error_jacobian = right_jacobian(error.tail<3>());
    task_jacobian->bottomRows<3>()                = rotation_error_jacobian * jacobian.bottomRows<3>();

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
