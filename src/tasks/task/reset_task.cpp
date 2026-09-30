#include "reset_task.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace {

/// 任务链末端任务名，任务失败时回退到该任务。
const std::string kIdelTaskName = "idel";

}  // namespace

ResetTask::ResetTask(const std::string& task_name, std::any ctx)
    : TaskFSM(task_name, ctx)
    , ctx_(std::any_cast<TaskContext*>(ctx)) {
    if (ctx_ == nullptr) {
        throw std::invalid_argument("ResetTask requires a TaskContext");
    }

    auto* node = ctx_->node().get();
    target_joint_pos_ = node->declare_parameter<std::vector<double>>("reset_joint_pos", std::vector<double>{});
    reset_duration_   = node->declare_parameter<double>("reset_duration", 3.0);
    reset_tolerance_  = node->declare_parameter<double>("reset_tolerance", 0.01);

    const double timeout_param = node->declare_parameter<double>("reset_timeout", 0.0);
    reset_timeout_             = timeout_param > 0.0 ? timeout_param : reset_duration_ + 12.0;

    const std::size_t joint_count = ctx_->options().joints.size();
    if (target_joint_pos_.empty()) {
        // 未配置时按控制器默认复位位置（全 0）处理。
        target_joint_pos_.assign(joint_count, 0.0);
    } else if (target_joint_pos_.size() != joint_count) {
        RCLCPP_WARN(
            node->get_logger(), "reset_joint_pos has %zu values but %zu joints; padding with 0.0",
            target_joint_pos_.size(), joint_count);
        target_joint_pos_.resize(joint_count, 0.0);
    }
}

bool ResetTask::enter(const std::string& last_task, const rclcpp::Time& time) {
    (void)last_task;

    enter_time_ = time;
    done_       = false;
    if (!ctx_->set_exp_state("reset")) {
        ctx_->fail("failed to switch arm_controller to reset");
        done_ = true;
    }
    return true;
}

bool ResetTask::run(const rclcpp::Time& time) {
    if (done_) {
        return true;
    }

    const double elapsed = (time - enter_time_).seconds();

    if (ctx_->have_joint_states() && ctx_->joint_positions().size() == target_joint_pos_.size()) {
        const auto& joints = ctx_->joint_positions();
        bool reached       = true;
        for (std::size_t i = 0; i < joints.size(); ++i) {
            if (std::abs(joints[i] - target_joint_pos_[i]) > reset_tolerance_) {
                reached = false;
                break;
            }
        }
        if (reached && elapsed >= reset_duration_) {
            done_ = true;
            RCLCPP_INFO(ctx_->node()->get_logger(), "reset task finished in %.2f s", elapsed);
            return true;
        }
    }

    if (elapsed > reset_timeout_) {
        ctx_->fail("reset task did not reach the target position within the timeout");
        done_ = true;
    }
    return true;
}

const std::string& ResetTask::check_switch() const {
    if (!done_) {
        return task_name_;
    }
    if (ctx_->failed()) {
        return kIdelTaskName;
    }
    return next_task_.empty() ? kIdelTaskName : next_task_;
}
