#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include <rclcpp/time.hpp>

#include "fsm.hpp"

/**
 * @brief 状态机推进结果
 *
 * 把「什么情况下允许停用控制器链」从隐含约定变成显式契约：
 * - @c Ok          本周期正常推进；
 * - @c Recoverable 状态报告失败（求解失败、拒绝进入/离开等），调用方应锁定当前
 *                  位置并切回 idel，**不得**停用控制器链；
 * - @c Fatal       状态机自身结构损坏（状态名不存在、指针为空），属于代码缺陷，
 *                  调用方可以停用控制器链。
 */
enum class RunResult {
    Ok,
    Recoverable,
    Fatal,
};

class FSMFactory {
public:
    FSMFactory() {
        current_state_name_.reserve(32);
        next_state_name_.reserve(32);
    }

    bool register_fsm(std::unique_ptr<FSM> fsm) {
        if (fsm == nullptr) {
            return false;
        }
        return fsm_map.emplace(fsm->get_name(), std::move(fsm)).second;
    }
    bool set_init_state(const std::string& name) {
        const auto state = fsm_map.find(name);
        if (state == fsm_map.end()) {
            return false;
        }
        current_state_name_ = name;
        return true;
    }

    /**
     * @brief 外部期望切换到的目标状态名
     *
     * 由外部指令设置（控制器侧通过 @c exp_state 参数），状态的 @c check_switch() 读取它
     * 决定是否切换。放在基类是因为失败路径需要它：状态报告失败时，工厂直接请求切到
     * 这里，而不依赖状态自身「清理 + 返回 true」来触发切换。
     */
    std::string exp_state_name{"idel"};

    RunResult run(const rclcpp::Time& time) {
        auto current_state = fsm_map.find(current_state_name_);
        if (current_state == fsm_map.end() || current_state->second == nullptr) {
            // 当前状态名不存在或指针为空：状态机结构损坏，非状态自身可控。
            return RunResult::Fatal;
        }
        auto& current_fsm = *current_state->second;

        if (first_run) {
            first_run = false;
            if (!current_fsm.enter("", time)) {
                // 初始状态拒绝进入：保持位置即可，不停用链条。
                return RunResult::Recoverable;
            }
            return RunResult::Ok;
        }

        if (state_switch) {
            auto next_state = fsm_map.find(next_state_name_);
            if (next_state == fsm_map.end() || next_state->second == nullptr) {
                // 目标状态名不存在或指针为空：结构损坏。
                return RunResult::Fatal;
            }
            auto& next_fsm = *next_state->second;
            if (!current_fsm.exit(next_fsm.get_name())) {
                // 当前状态拒绝离开：保持位置，等待外部干预。
                return RunResult::Recoverable;
            }
            if (!next_fsm.enter(current_fsm.get_name(), time)) {
                // 目标状态拒绝进入：保持位置，等待外部干预。
                return RunResult::Recoverable;
            }
            current_state_name_ = next_state_name_;
            state_switch        = false;
        } else {
            if (!current_fsm.run(time)) {
                // 状态自身报告失败（求解失败、数据异常等）：可恢复。
                //
                // 关键：这里**不能直接返回**。原实现依赖各状态在失败路径上「清理内部
                // 阶段标志 + 写 command_ 保持值 + 置 exp_state + 返回 true」来让
                // check_switch() 得以执行；一旦状态只 return false，state_switch 就
                // 永远不会置位，状态再也切不出去（控制器只能一直保持，任务层等到超时）。
                // 因此由工厂直接请求切到外部期望状态（控制器在故障时已置为 idel），
                // 保持值的写入由 ArmController::apply_safe_hold_and_idel() 负责。
                if (!exp_state_name.empty() && exp_state_name != current_state_name_) {
                    const auto target = fsm_map.find(exp_state_name);
                    if (target == fsm_map.end() || target->second == nullptr) {
                        // 期望状态名不存在：结构损坏。
                        return RunResult::Fatal;
                    }
                    next_state_name_ = exp_state_name;
                    state_switch     = true;
                }
                return RunResult::Recoverable;
            }
            const std::string& next_state = current_fsm.check_switch();
            if (next_state != current_fsm.get_name()) {
                const auto state = fsm_map.find(next_state);
                if (state == fsm_map.end() || state->second == nullptr) {
                    // 状态请求切换到一个不存在的目标：结构损坏。
                    return RunResult::Fatal;
                }
                next_state_name_ = next_state;
                state_switch     = true;
            }
        }

        return RunResult::Ok;
    }

    // 当前活动状态名（只读）：供外部观测控制器实际处于哪个状态。
    const std::string& current_state_name() const { return current_state_name_; }

private:
    bool first_run{true};
    bool state_switch{false};
    std::string next_state_name_;
    std::string current_state_name_;
    std::string last_state_name;
    std::unordered_map<std::string, std::unique_ptr<FSM>> fsm_map;
};
