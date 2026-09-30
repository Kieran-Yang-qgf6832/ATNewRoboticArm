/**
 * @file joint_traj_task.hpp
 * @brief 关节空间轨迹任务声明
 *
 * 请求控制器切换到 @c joint_traj 控制模式，然后向 @c /arm_joint_traj 发布一条
 * 两点关节轨迹：起点取当前关节角（/joint_states），终点为参数给定的目标关节角。
 * 控制器在两点之间做五次插值生成位置/速度/加速度，再用逆动力学求出关节力矩。
 *
 * 任务分三个阶段：
 * -# @c kSettle：等待控制器真正切入 joint_traj；
 * -# @c kMoving：发布轨迹并等待运动时长；
 * -# @c kDone：检查关节到位误差，切换到后继任务。
 *
 * @note 关节顺序与 @c joints 参数一致，目标关节角长度必须等于关节数。
 *
 * @see TaskFSM
 * @see TaskContext
 *
 * @author lyz
 * @date 2026-09-30
 * @version 0.0.0
 */

#pragma once

#include <any>
#include <string>
#include <vector>

#include "task_context.hpp"
#include "task_fsm.hpp"

/**
 * @brief 关节空间轨迹任务
 *
 * 参数：
 * - @c joint_traj_target_positions：目标关节角（弧度），长度必须等于关节数，必填；
 * - @c joint_traj_settle_duration：切入 joint_traj 后的等待时间（秒，默认 0.3）；
 * - @c joint_traj_motion_duration：轨迹运动时长（秒，默认 3.0）；
 * - @c joint_traj_position_tolerance：到位判定容差（弧度，默认 0.01）；
 * - @c joint_traj_timeout：任务层超时（秒，默认 @c joint_traj_motion_duration + 10）。
 */
class JointTrajTask : public TaskFSM {
public:
    /**
     * @brief 任务阶段
     */
    enum class Phase {
        kSettle, ///< 等待控制器切入 joint_traj
        kMoving, ///< 轨迹执行中
        kDone,   ///< 完成或失败
    };

    /**
     * @brief 构造函数
     * @param task_name 任务名，固定为 "joint_traj"
     * @param ctx       上下文，实际类型为 @c TaskContext*
     * @throws std::invalid_argument 上下文为空指针时抛出
     */
    JointTrajTask(const std::string& task_name, std::any ctx);

    /// 校验目标关节角并请求控制器切换到 joint_traj，进入 kSettle 阶段。
    bool enter(const std::string& last_task, const rclcpp::Time& time) override;

    /// 按阶段推进：等切换稳定 -> 发布关节轨迹 -> 等待到位。
    bool run(const rclcpp::Time& time) override;

    /// @return 轨迹执行完成后返回后继任务名，否则留在本任务
    const std::string& check_switch() const override;

private:
    TaskContext* ctx_{nullptr};          ///< 共享上下文
    Phase phase_{Phase::kSettle};        ///< 当前阶段
    rclcpp::Time phase_start_time_;      ///< 当前阶段开始时刻
    std::vector<double> target_joints_;  ///< 目标关节角（弧度）
    double settle_duration_{0.3};        ///< 切入等待时间（秒）
    double motion_duration_{3.0};        ///< 轨迹运动时长（秒）
    double position_tolerance_{0.01};    ///< 到位容差（弧度）
    double timeout_{13.0};               ///< 任务层超时（秒）
};
