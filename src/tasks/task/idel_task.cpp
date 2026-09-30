#include "idel_task.hpp"

#include <stdexcept>

namespace {

/// 任务链末端任务名，任务失败时回退到该任务。
const std::string kIdelTaskName = "idel";

}  // namespace

IdelTask::IdelTask(const std::string& task_name, std::any ctx)
    : TaskFSM(task_name, ctx)
    , ctx_(std::any_cast<TaskContext*>(ctx)) {
    if (ctx_ == nullptr) {
        throw std::invalid_argument("IdelTask requires a TaskContext");
    }
    hold_duration_ = ctx_->node()->declare_parameter<double>("idel_hold_duration", 0.3);
}

bool IdelTask::enter(const std::string& last_task, const rclcpp::Time& time) {
    (void)last_task;

    enter_time_ = time;
    settled_    = false;
    if (!ctx_->set_exp_state(kIdelTaskName)) {
        ctx_->fail("failed to switch arm_controller to idel");
        settled_ = true;
    }
    return true;
}

bool IdelTask::run(const rclcpp::Time& time) {
    if (settled_) {
        return true;
    }
    if ((time - enter_time_).seconds() >= hold_duration_) {
        settled_ = true;
    }
    return true;
}

const std::string& IdelTask::check_switch() const {
    if (!settled_ || next_task_.empty()) {
        return task_name_;
    }
    return next_task_;
}
