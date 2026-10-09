#pragma once

#include <Eigen/Dense>

class TaskMapping {
public:
    virtual ~TaskMapping() = default;

    // Map the model state to the user-defined task coordinates.
    virtual bool position_map(const Eigen::VectorXd& joint_pos,
                              const Eigen::Isometry3d& pose,
                              Eigen::VectorXd* task_position) = 0;

    /**
     * @brief 把模型几何雅可比映射为「任务误差对关节角导数」的相反数
     *
     * Gauss-Newton 的步长公式是 @c dq = (JᵀJ)⁻¹ Jᵀ e，其中 @c J = -∂e/∂q
     * （@c e 为任务误差，见 @ref error_map）。位置部分 @c e_p = p_t - p_c 给出
     * @c ∂e_p/∂q = -J_v，故 @c J = +J_v。
     *
     * **姿态部分不能直接照搬几何雅可比**：@c e_r = log(R_t R_cᵀ) 的时间导数为
     * @c de_r/dt = -J_r(ρ_e)·ω_c（右雅可比、带负号，@c ρ_e 是**误差**旋转向量），
     * 因此 @c J_rot = +J_r(ρ_e)·J_geom_rot。少了这一步等于默认 @c J_r = I，
     * 只在小误差下成立；误差接近 π 时线性化失准，逆解会停滞在判据之上再也降不下去
     * （项目 bugs.md 记录的 O5）。
     *
     * @param joint_pos     当前关节角
     * @param error         当前任务空间误差（由 @ref error_map 得到）
     * @param jacobian      模型几何雅可比
     * @param task_jacobian 输出：任务误差的 Gauss-Newton 雅可比
     * @return true 映射成功
     */
    virtual bool jacobian_map(const Eigen::VectorXd& joint_pos,
                              const Eigen::VectorXd& error,
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
