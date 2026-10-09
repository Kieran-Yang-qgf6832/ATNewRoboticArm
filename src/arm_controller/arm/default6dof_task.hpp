#pragma once

#include "task.hpp"
class Default6DofTaskSpaceMapping : public TaskMapping{
public:
    Default6DofTaskSpaceMapping();

    bool position_map(const Eigen::VectorXd& joint_pos,
                      const Eigen::Isometry3d& pose,
                      Eigen::VectorXd* task_position) override;

    /**
     * @brief 把几何雅可比映射为任务误差的 Gauss-Newton 雅可比
     *
     * 位置部分直接照搬；姿态部分乘右雅可比 @c J_r(ρ_e)（@c ρ_e 为误差旋转向量），
     * 详见 @ref TaskMapping::jacobian_map。
     */
    bool jacobian_map(const Eigen::VectorXd& joint_pos,
                      const Eigen::VectorXd& error,
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
