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

    /**
     * @brief 切换 5 维（只控位置 + 工具指向）模式
     *
     * 打开后：
     * - @ref error_map 把旋转误差投影到「垂直于工具轴」的平面（去掉滚转分量）；
     * - @ref jacobian_map 把姿态雅可比的旋转行左乘投影矩阵 @c I - zzᵀ。
     * 效果是 joint6（绕工具轴）那一列精确为 0，LM 步长在那一维恒为 0，
     * 因此 joint6 停在初值不动——正是实机上 joint6 没有电机的行为。
     *
     * @param roll_free true = 5 维任务
     * @note 工具轴取自最近一次 @ref position_map 传入的位姿，求解器在每次
     *       误差/雅可比求值前都会先调用它，因此缓存始终与当前迭代点一致。
     */
    void set_roll_free(bool roll_free) override;

    bool roll_free() const override { return roll_free_; }

private:
    bool roll_free_{false};
    /// 当前迭代点处、表达在基座系里的工具 z 轴（单位向量）
    Eigen::Vector3d tool_axis_{Eigen::Vector3d::UnitZ()};
};
