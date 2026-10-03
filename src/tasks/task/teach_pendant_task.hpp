/**
 * @file teach_pendant_task.hpp
 * @brief 示教任务声明
 *
 * 请求控制器切换到 @c teach_pendant 控制模式：位置增益置零、只保留速度增益，
 * 力矩输出为重力补偿项，因此机械臂近似零力平衡，可由外力自由拖动（示教）。
 * 保持指定时长后自动请求控制器切回 @c idel。
 *
 * @warning 控制器对 @c teach_pendant 有硬约束——只能从 @c idel 进入、只能回到 @c idel
 *          （见 arm_controller 的 TeachPendantState::enter/exit）。runner 会自动在本
 *          任务前后补一个 idel 任务，写到 task_sequence 里时无需手动加。
 *
 * @see TaskFSM
 * @see TaskContext
 *
 * @author lyz
 * @date 2026-10-03
 * @version 0.0.0
 */

#pragma once

#include <any>
#include <string>
#include <vector>

#include "task_context.hpp"
#include "task_fsm.hpp"

/**
 * @brief 示教任务
 *
 * 参数：
 * - @c teach_pendant_duration：示教时长（秒，默认 10.0）；小于等于 0 表示不设时限，
 *   一直保持到外部中断（此时注意 runner 的 @c task_timeout 会先触发）。
 */
class TeachPendantTask : public TaskFSM {
public:
    /**
     * @brief 构造函数
     * @param task_name 任务名，固定为 "teach_pendant"
     * @param ctx       上下文，实际类型为 @c TaskContext*
     * @throws std::invalid_argument 上下文为空指针时抛出
     */
    TeachPendantTask(const std::string& task_name, std::any ctx);

    /// 记录起始关节角并请求控制器切换到 teach_pendant。
    bool enter(const std::string& last_task, const rclcpp::Time& time) override;

    /// 计时到点后请求控制器切回 idel。
    bool run(const rclcpp::Time& time) override;

    /// @return 示教结束后返回后继任务名（runner 自动成 idel），否则留在本任务
    const std::string& check_switch() const override;

private:
    /// 结束示教：请求切回 idel 并打印关节角变化。
    void finish_teaching();

    TaskContext* ctx_{nullptr};           ///< 共享上下文
    rclcpp::Time enter_time_;             ///< 进入任务的时刻
    std::vector<double> start_joints_;    ///< 示教开始时的关节角（用于日志）
    double duration_{10.0};               ///< 示教时长（秒），<= 0 表示无时限
    bool done_{false};                    ///< 示教是否结束
};
