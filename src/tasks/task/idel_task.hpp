/**
 * @file idel_task.hpp
 * @brief 空闲任务声明
 *
 * 把控制器切回 @c idel（锁定当前位置），并可选地在停留一段时间后切换到
 * 后继任务。通常作为任务链末端：runner 检测到它没有后继时结束整个任务链。
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

#include "task_context.hpp"
#include "task_fsm.hpp"

/**
 * @brief 空闲任务
 *
 * 参数：
 * - @c idel_hold_duration：进入本任务后停留多久才允许切到后继任务（秒，默认 0.3）。
 */
class IdelTask : public TaskFSM {
public:
    /**
     * @brief 构造函数
     * @param task_name 任务名，固定为 "idel"
     * @param ctx       上下文，实际类型为 @c TaskContext*
     * @throws std::invalid_argument 上下文为空指针时抛出
     */
    IdelTask(const std::string& task_name, std::any ctx);

    /// 请求控制器切换到 idel，并记录进入时刻。
    bool enter(const std::string& last_task, const rclcpp::Time& time) override;

    /// 检查停留时间是否满足。
    bool run(const rclcpp::Time& time) override;

    /// @return 停留结束且存在后继任务时返回后继任务名，否则留在本任务
    const std::string& check_switch() const override;

private:
    TaskContext* ctx_{nullptr}; ///< 共享上下文
    rclcpp::Time enter_time_;   ///< 进入任务的时刻
    double hold_duration_{0.3}; ///< 停留时长（秒）
    bool settled_{false};       ///< 停留是否结束
};
