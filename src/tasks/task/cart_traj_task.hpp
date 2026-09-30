/**
 * @file cart_traj_task.hpp
 * @brief 笛卡尔轨迹任务声明
 *
 * 请求控制器切换到 @c cart_traj 控制模式，然后向 @c /arm_cart_traj 发布一条
 * 两点笛卡尔轨迹：起点取当前末端位姿（TF 查询），终点为起点叠加给定偏移
 * （或直接指定的绝对目标位姿）。控制器在两点之间做五次插值、逆运动学求解，
 * 再用逆动力学算出关节力矩。
 *
 * 任务分三个阶段：
 * -# @c kSettle：等待控制器真正切入 cart_traj；
 * -# @c kMoving：发布轨迹并等待运动时长；
 * -# @c kDone：检查到位误差，切换到后继任务。
 *
 * @note 任务空间向量为 [x, y, z, rx, ry, rz]，旋转部分为旋转向量（angle * axis），
 *       因此姿态偏移做向量相加只在偏转角较小时近似成立。
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
 * @brief 笛卡尔轨迹任务
 *
 * 参数：
 * - @c cart_traj_target_offset：相对起点的位姿偏移，6 个元素，默认 {0, 0, 0.05, 0, 0, 0}；
 * - @c cart_traj_target_pose：绝对目标位姿（6 个元素）；给定时忽略上面的偏移；
 * - @c cart_traj_settle_duration：切入 cart_traj 后的等待时间（秒，默认 0.3）；
 * - @c cart_traj_motion_duration：轨迹运动时长（秒，默认 3.0）；
 * - @c cart_traj_position_tolerance：到位判定容差（米，默认 0.005）；
 * - @c cart_traj_timeout：任务层超时（秒，默认 @c cart_traj_motion_duration + 10）。
 */
class CartTrajTask : public TaskFSM {
public:
    /**
     * @brief 任务阶段
     */
    enum class Phase {
        kSettle, ///< 等待控制器切入 cart_traj
        kMoving, ///< 轨迹执行中
        kDone,   ///< 完成或失败
    };

    /**
     * @brief 构造函数
     * @param task_name 任务名，固定为 "cart_traj"
     * @param ctx       上下文，实际类型为 @c TaskContext*
     * @throws std::invalid_argument 上下文为空指针时抛出
     */
    CartTrajTask(const std::string& task_name, std::any ctx);

    /// 请求控制器切换到 cart_traj，进入 kSettle 阶段。
    bool enter(const std::string& last_task, const rclcpp::Time& time) override;

    /// 按阶段推进：等切换稳定 -> 发布轨迹 -> 等待到位。
    bool run(const rclcpp::Time& time) override;

    /// @return 轨迹执行完成后返回后继任务名，否则留在本任务
    const std::string& check_switch() const override;

private:
    /**
     * @brief 由起点位姿与参数计算目标位姿
     * @param start_pose 起点位姿（6 维任务空间向量）
     */
    void compute_target_pose(const std::vector<double>& start_pose);

    TaskContext* ctx_{nullptr};             ///< 共享上下文
    Phase phase_{Phase::kSettle};           ///< 当前阶段
    rclcpp::Time phase_start_time_;         ///< 当前阶段开始时刻
    std::vector<double> start_pose_;        ///< 轨迹起点位姿
    std::vector<double> target_pose_;       ///< 轨迹终点位姿
    std::vector<double> target_offset_;     ///< 相对起点的偏移
    std::vector<double> absolute_pose_;     ///< 绝对目标位姿参数（为空表示使用偏移）
    double settle_duration_{0.3};           ///< 切入等待时间（秒）
    double motion_duration_{3.0};           ///< 轨迹运动时长（秒）
    double position_tolerance_{0.005};      ///< 到位容差（米）
    double timeout_{13.0};                  ///< 任务层超时（秒）
};
