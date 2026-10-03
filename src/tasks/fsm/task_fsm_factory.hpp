/**
 * @file task_fsm_factory.hpp
 * @brief 任务层状态机工厂声明
 *
 * 负责注册任务、串联任务链并周期性调度，调度语义与控制器侧 FSMFactory
 * （src/arm_controller/lib/fsm/fsm_factory.hpp）保持一致：
 * -# run() 先调用当前任务的 run()，再用 check_switch() 判断是否需要切换；
 * -# 切换动作延迟到下一个周期，在同一周期内完成 exit()/enter()。
 *
 * @see TaskFSM
 * @see TaskContext
 *
 * @author lyz
 * @date 2026-09-30
 * @version 0.0.0
 */

#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/time.hpp>

#include "task_fsm.hpp"

/**
 * @brief 任务层状态机工厂
 *
 * 持有全部任务实例，按任务链顺序调度。任务名在工厂内唯一，重名注册会失败。
 */
class TaskFSMFactory {
public:
    TaskFSMFactory() {
        next_task_name_.reserve(32);
        current_task_name_.reserve(32);
    }

    /**
     * @brief 注册任务
     * @param task 任务实例，任务名取自 @ref TaskFSM::get_name()
     * @return true 注册成功；false 空指针或任务名重复
     */
    bool register_task(std::unique_ptr<TaskFSM> task) {
        if (task == nullptr) {
            return false;
        }
        return task_map_.emplace(task->get_name(), std::move(task)).second;
    }

    /**
     * @brief 查询任务是否已注册
     * @param name 任务名
     * @return true 已注册
     */
    bool has_task(const std::string& name) const { return task_map_.find(name) != task_map_.end(); }

    /**
     * @brief 列出全部已注册任务名
     * @return 任务名列表，用于日志与错误提示
     */
    std::vector<std::string> task_names() const {
        std::vector<std::string> names;
        names.reserve(task_map_.size());
        for (const auto& entry : task_map_) {
            names.push_back(entry.first);
        }
        return names;
    }

    /**
     * @brief 设置起始任务
     * @param name 任务名
     * @return true 设置成功；false 任务未注册
     * @note 必须在第一次调用 run() 之前设置
     */
    bool set_init_task(const std::string& name) {
        if (!has_task(name)) {
            return false;
        }
        current_task_name_ = name;
        return true;
    }

    /**
     * @brief 串联任务链：@p name 完成后自动切换到 @p next
     * @param name 当前任务名
     * @param next 后继任务名
     * @return true 串联成功；false 任务未注册
     * @note 同一个任务实例只保存一个后继，因此同一任务类型在链中重复出现时，
     *       其出口以后一次 link() 为准。
     */
    bool link(const std::string& name, const std::string& next) {
        auto task = task_map_.find(name);
        if (task == task_map_.end() || task->second == nullptr || !has_task(next)) {
            return false;
        }
        task->second->set_next_task(next);
        return true;
    }

    /**
     * @brief 推进一个调度周期
     * @param time 当前时刻
     * @return true 调度正常；false 当前任务不存在或任务回调返回失败
     */
    bool run(const rclcpp::Time& time) {
        auto current = task_map_.find(current_task_name_);
        if (current == task_map_.end() || current->second == nullptr) {
            return false;
        }
        auto& current_task = *current->second;

        if (first_run_) {
            first_run_ = false;
            return current_task.enter("", time);
        }

        if (switching_) {
            auto next = task_map_.find(next_task_name_);
            if (next == task_map_.end() || next->second == nullptr) {
                return false;
            }
            auto& next_task = *next->second;
            if (!current_task.exit(next_task.get_name()) || !next_task.enter(current_task.get_name(), time)) {
                return false;
            }
            current_task_name_ = next_task_name_;
            switching_         = false;
            ++switch_count_;
            return true;
        }

        if (!current_task.run(time)) {
            return false;
        }
        const std::string& next_name = current_task.check_switch();
        if (next_name != current_task.get_name()) {
            if (!has_task(next_name)) {
                return false;
            }
            next_task_name_ = next_name;
            switching_      = true;
        }
        return true;
    }

    /// @return 当前任务名
    const std::string& current_task() const { return current_task_name_; }

    /**
     * @brief 已完成的切换次数
     * @return 调度器启动后成功完成的 exit()/enter() 次数
     * @note 同一个任务实例在任务链中可能出现在多个位置（例如链中间的 idel），
     *       因此调用方用「切换次数」而不是 TaskFSM::next_task() 判断是否走到链尾：
     *       任务链长度为 N 时，切换次数达到 N-1 即表示已到达链尾任务。
     */
    std::size_t switch_count() const { return switch_count_; }

private:
    bool first_run_{true};            ///< 是否尚未调用首次 enter()
    bool switching_{false};           ///< 是否已确定下一周期要切换
    std::size_t switch_count_{0};     ///< 已完成的切换次数
    std::string next_task_name_;      ///< 待切换到的任务名
    std::string current_task_name_;   ///< 当前任务名
    std::unordered_map<std::string, std::unique_ptr<TaskFSM>> task_map_; ///< 任务表
};
