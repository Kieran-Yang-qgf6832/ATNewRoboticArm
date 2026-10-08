#pragma once

#include <Eigen/Dense>

class TaskMapping {
public:
    virtual ~TaskMapping() = default;

    // Map the model state to the user-defined task coordinates.
    virtual bool position_map(const Eigen::VectorXd& joint_pos,
                              const Eigen::Isometry3d& pose,
                              Eigen::VectorXd* task_position) = 0;

    // Map the model geometric Jacobian to the Jacobian of those coordinates.
    virtual bool jacobian_map(const Eigen::VectorXd& joint_pos,
                              const Eigen::MatrixXd& jacobian,
                              Eigen::MatrixXd* task_jacobian) = 0;

    /**
     * @brief 计算任务空间误差（目标 - 当前）
     *
     * **不能用 @c target - @c current 直接相减**：姿态若用旋转向量表示，它在
     * |rv| ≈ π 处是奇异的——rv 与 -rv 表示同一姿态，一旦目标与当前落在奇异点两侧，
     * 逐分量相减会得到 ≈2π 的巨大误差，逆解迭代随即发散（项目 bugs.md 记录的 A2）。
     *
     * 实现应把姿态误差算成**相对旋转** log(R_target · R_currentᵀ)，其结果恒在
     * [0, π] 内、连续、无奇异。
     *
     * @param target  目标任务空间坐标
     * @param current 当前任务空间坐标（由 @ref position_map 得到）
     * @param error   输出：与 @p target 同维度的任务空间误差
     * @return true 计算成功
     */
    virtual bool error_map(const Eigen::VectorXd& target,
                           const Eigen::VectorXd& current,
                           Eigen::VectorXd* error) = 0;
};
