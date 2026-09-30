/**
 * @file reset_task.hpp
 * @brief 复位任务声明
 *
 * 请求控制器切换到 @c reset 控制模式；控制器会按 @c reset_joint_pos 线性插值
 * 到复位位置，完成后自行切回 @c idel。任务层通过 /joint_states 判断复位是否完成，
 * 再切换到后继任务。
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
 * @brief 复位任务
 *
 * 参数（需与控制器 YAML 中的同名参数保持一致）：
 * - @c reset_joint_pos：复位目标关节角（弧度），默认全 0；
 * - @c reset_duration：控制器复位时长（秒，默认 3.0），复位至少需要这么久；
 * - @c reset_tolerance：到位容差（弧度，默认 0.01）；
 * - @c reset_timeout：任务层超时（秒，默认 @c reset_duration + 12）。
 */
class ResetTask : public TaskFSM {
public:
    /**
     * @brief 构造函数
     * @param task_name 任务名，固定为 "reset"
     * @param ctx       上下文，实际类型为 @c TaskContext*
     * @throws std::invalid_argument 上下文为空指针时抛出
     */
    ResetTask(const std::string& task_name, std::any ctx);

    /// 请求控制器切换到 reset。
    bool enter(const std::string& last_task, const rclcpp::Time& time) override;

    /// 轮询关节角，判断是否已到达复位位置。
    bool run(const rclcpp::Time& time) override;

    /// @return 复位完成后返回后继任务名，否则留在本任务
    const std::string& check_switch() const override;

private:
    TaskContext* ctx_{nullptr};             ///< 共享上下文
    std::vector<double> target_joint_pos_;  ///< 复位目标关节角（弧度）
    rclcpp::Time enter_time_;               ///< 进入任务的时刻
    double reset_duration_{3.0};            ///< 控制器复位时长（秒）
    double reset_tolerance_{0.01};          ///< 到位容差（弧度）
    double reset_timeout_{15.0};            ///< 任务层超时（秒）
    bool done_{false};                      ///< 复位是否结束（成功或超时）
};
