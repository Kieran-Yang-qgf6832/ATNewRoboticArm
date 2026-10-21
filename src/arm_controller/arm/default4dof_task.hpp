#pragma once

#include "task.hpp"

/**
 * @brief 4 自由度机械臂的任务空间映射：[x, y, z, q4]
 *
 * 任务空间维度必须等于模型真正能独立控制的自由度个数。4 轴臂的 6 维几何雅可比
 * 秩恒为 4（joint1/joint2 是绕 y 的平面 2R、joint3 是腕部偏转、joint4 绕工具轴
 * 纯自转且对末端原点零位移），所以 6 维任务空间下笛卡尔目标必然不可达。实测：
 * 把 6 维目标直接做最小二乘「尽力而为」，pick_demo 的位置误差达 28.6 mm（姿态
 * 反而只差 0.0095 rad）；只控位置时位置误差 0.12 mm。降维必须显式做。
 *
 * 四个坐标的分工：
 * - x, y, z 由 joint1/joint2/joint3 决定（y 方向工作空间只有约 4 cm 厚，是结构限制）；
 * - q4 是 joint4 的角度，即工具绕自身轴的自转角——**它不改变末端原点位置**，
 *   所以是唯一一个与位置解耦的姿态自由度。
 *
 * 工具指向（工具 z 轴方向）**不受控**：它由位置经关节 1/2/3 唯一决定（至多差 IK 分支）。
 * 实测 5 cm 平移会让工具指向漂 4–8°，吸盘取放可以接受；需要严格贴面时改用关节运动。
 *
 * @see Default6DofTaskSpaceMapping
 * @see TaskMapping
 */
class Default4DofTaskSpaceMapping : public TaskMapping {
public:
    Default4DofTaskSpaceMapping();

    /// 4 维任务空间：[x, y, z, q4]（q4 为 joint4 自转角，弧度）
    int task_dim() const override { return 4; }

    /**
     * @brief 正映射：位姿 + joint4 角度 -> [x, y, z, q4]
     * @note 4 轴模型的 joint4 轴通过末端原点，所以 q4 只改姿态、不改位置。
     */
    bool position_map(const Eigen::VectorXd& joint_pos,
                      const Eigen::Isometry3d& pose,
                      Eigen::VectorXd* task_position) override;

    /**
     * @brief 把几何雅可比映射为任务误差的 Gauss-Newton 雅可比
     *
     * 前三行照搬几何雅可比的位置行；第四行是 q4 自己的单位行（d(q4)/d(q4) = 1）。
     * 几何雅可比第 4 列在 4 轴模型下是纯自转（平移分量为 0），与这一行一致。
     */
    bool jacobian_map(const Eigen::VectorXd& joint_pos,
                      const Eigen::VectorXd& error,
                      const Eigen::MatrixXd& jacobian,
                      Eigen::MatrixXd* task_jacobian) override;

    /**
     * @brief 4 维任务空间误差
     *
     * 位置直接相减；自转角按最短路径归一化到 [-pi, pi]，避免 2pi 的跳变。
     */
    bool error_map(const Eigen::VectorXd& target,
                   const Eigen::VectorXd& current,
                   Eigen::VectorXd* error) override;

    /**
     * @brief 6 维位姿指令 -> 4 维任务目标
     *
     * 取 [x, y, z]；自转目标取**当前** joint4 角度（即「保持自转」）。消息里的
     * 姿态分量（rx, ry, rz）被丢弃——4 轴臂在给定位置下没有能力改变工具指向。
     */
    bool target_from_pose(const Eigen::VectorXd& joint_pos,
                          const Eigen::VectorXd& pose,
                          Eigen::VectorXd* target) const override;
};
