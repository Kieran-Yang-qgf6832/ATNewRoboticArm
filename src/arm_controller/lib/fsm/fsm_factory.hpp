#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include <rclcpp/time.hpp>

#include "fsm.hpp"

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
    bool run(const rclcpp::Time& time) {
        auto current_state = fsm_map.find(current_state_name_);
        if (current_state == fsm_map.end() || current_state->second == nullptr) {
            return false;
        }
        auto& current_fsm = *current_state->second;

        if(first_run)
        {
            first_run=false;
            current_fsm.enter("", time);
            return true;
        }

        if (state_switch) {
            auto next_state = fsm_map.find(next_state_name_);
            if (next_state == fsm_map.end() || next_state->second == nullptr) {
                return false;
            }
            auto& next_fsm = *next_state->second;
            bool switch_success=current_fsm.exit(next_fsm.get_name());
            if(!switch_success)
                return false;
            switch_success=next_fsm.enter(current_fsm.get_name(), time);
            if(!switch_success)
                return false;
            current_state_name_=next_state_name_;
            state_switch=false;
        } else {
            bool success = current_fsm.run(time);
            if (!success)
                return false;
            const std::string& next_state = current_fsm.check_switch();
            if(next_state != current_fsm.get_name())
            {
                const auto state = fsm_map.find(next_state);
                if (state == fsm_map.end() || state->second == nullptr) {
                    return false;
                }
                next_state_name_ = next_state;
                state_switch=true;
            } 
        }

        return true;
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
