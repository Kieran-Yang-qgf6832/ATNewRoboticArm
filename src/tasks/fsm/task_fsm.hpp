/**
 * @file task_fsm.hpp
 * @brief 任务层有限状态机基类声明
 *
 * 与控制器侧 FSM（src/arm_controller/lib/fsm/fsm.hpp）保持相同的接口形态，
 * 但语义不同：这里的每个状态是一个「任务」，任务内部通过 @ref TaskContext
 * 驱动 arm_controller 切换控制模式（exp_state 参数）并下发任务指令。
 *
 * 任务之间通过 @ref TaskFSM::set_next_task 串成任务链：任务完成后
 * check_switch() 返回下一个任务名，由 @ref TaskFSMFactory 完成切换。
 *
 * @note 任务层运行在普通 ROS 节点线程中，允许阻塞式服务调用与内存分配，
 *       不受 arm_controller 实时控制路径的约束。
 *
 * @see TaskFSMFactory
 * @see TaskContext
 *
 * @author lyz
 * @date 2026-09-30
 * @version 0.0.0
 */

#pragma once

#include <any>
#include <string>
#include <utility>

#include <rclcpp/time.hpp>

/**
 * @brief 任务层状态机基类
 *
 * 子类需要实现的接口与控制器侧 FSM 一致：
 * - enter()：进入任务，通常在这里请求控制器切换到对应控制模式；
 * - run()：周期性推进任务（等待运动完成、发布指令等）；
 * - check_switch()：返回下一个任务名，返回自身任务名表示继续留在本任务；
 * - exit()：离开任务，一般只做标志位复位。
 *
 * @warning 任务内部不应直接持有 ros2_control 句柄，只能通过 @ref TaskContext
 *          与 arm_controller 交互，保证任务层与控制器层解耦。
 */
class TaskFSM {
public:
    /**
     * @brief 构造函数
     * @param task_name 任务名，同时作为状态机中的状态标识
     * @param ctx       上下文，实际类型为 @c TaskContext*
     */
    TaskFSM(std::string task_name, std::any ctx)
        : task_name_(std::move(task_name))
        , ctx_(std::move(ctx)) {}

    virtual ~TaskFSM() = default;

    /// @return 任务名
    const std::string& get_name() const { return task_name_; }

    /// 设置本任务完成后自动切换到的下一个任务（空串表示任务链末端）。
    void set_next_task(const std::string& next_task) { next_task_ = next_task; }

    /// @return 后继任务名，空串表示任务链末端
    const std::string& next_task() const { return next_task_; }

    /**
     * @brief 进入任务时调用
     * @param last_task 上一个任务名（首次进入为空串）
     * @param time       进入时刻
     * @return true 进入成功；false 表示前置条件不满足
     */
    virtual bool enter(const std::string& last_task, const rclcpp::Time& time) {
        (void)last_task;
        (void)time;
        return true;
    }

    /**
     * @brief 离开任务时调用
     * @param next_task 下一个任务名
     * @return true 允许离开；false 拒绝切换
     */
    virtual bool exit(const std::string& next_task) {
        (void)next_task;
        return true;
    }

    /**
     * @brief 检查是否需要切换任务
     * @return 需要切换到的任务名；返回自身任务名表示留在本任务
     */
    virtual const std::string& check_switch() const { return task_name_; }

    /**
     * @brief 周期性推进任务
     * @param time 当前时刻
     * @return true 运行正常；false 表示任务内部不可恢复错误
     */
    virtual bool run(const rclcpp::Time& time) {
        (void)time;
        return true;
    }

protected:
    std::string task_name_; ///< 任务名
    std::string next_task_; ///< 后继任务名（空串表示任务链末端）
    std::any ctx_;          ///< 上下文，实际类型为 TaskContext*
};
