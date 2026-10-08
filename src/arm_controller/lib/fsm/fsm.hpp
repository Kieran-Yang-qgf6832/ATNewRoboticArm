#pragma once

#include <string>
#include <any>

#include <rclcpp/time.hpp>

class FSMFactory;

class FSM {
public:
    FSM(const std::string& fsm_name, std::any ctx)
        : fsm_name_(fsm_name)
        , ctx_(ctx) {}

    virtual ~FSM() = default;

    const std::string& get_name() const { return fsm_name_; }

    virtual bool enter(const std::string& last_state, const rclcpp::Time& time) {
        (void)last_state;
        (void)time;
        return true;
    }

    virtual bool exit(const std::string& next_state) {
        (void)next_state;
        return true;
    }

    virtual const std::string& check_switch() const {
        return fsm_name_;
    }

    virtual bool run(const rclcpp::Time& time) {
        (void)time;
        return true;
    }

    /**
     * @brief 本次失败发生在哪个阶段
     *
     * 状态在 @c run() 返回 false 之前设置，供上层日志定位。故障**处置策略**统一由
     * ArmController::update() 负责，这里只提供诊断信息，不参与决策。
     *
     * @return 阶段名；未失败或未设置时为 nullptr
     */
    const char* fault_stage() const { return fault_stage_; }

    /// 清除上一次失败留下的阶段名（由工厂在每次调用 run() 前调用）
    void clear_fault_stage() { fault_stage_ = nullptr; }

protected:
    /// 设置本次失败的阶段名；传入的是字符串字面量，不涉及动态内存
    void set_fault_stage(const char* stage) { fault_stage_ = stage; }

    std::string fsm_name_;
    std::any ctx_;

private:
    const char* fault_stage_{nullptr};
};
