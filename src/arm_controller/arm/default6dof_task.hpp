#pragma once

#include "task.hpp"
class Default6DofTaskSpaceMapping : public TaskMapping{
public:
    Default6DofTaskSpaceMapping();

    bool position_map(const Eigen::VectorXd& joint_pos,
                      const Eigen::Isometry3d& pose,
                      Eigen::VectorXd* task_position) override;

    bool jacobian_map(const Eigen::VectorXd& joint_pos,
                      const Eigen::MatrixXd& jacobian,
                      Eigen::MatrixXd* task_jacobian) override;

    /**
     * @brief 计算 6 维任务空间误差
     *
     * 位置部分直接相减；姿态部分用相对旋转 log(R_target · R_currentᵀ)，
     * 避免旋转向量在 |rv| ≈ π 处的奇异（详见 @ref TaskMapping::error_map）。
     */
    bool error_map(const Eigen::VectorXd& target,
                   const Eigen::VectorXd& current,
                   Eigen::VectorXd* error) override;
};
