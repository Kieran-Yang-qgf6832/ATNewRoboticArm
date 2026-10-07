#include "reset_task.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>
#include <string>

namespace {

/// 任务链末端任务名，任务失败时回退到该任务。
const std::string kIdelTaskName = "idel";

/**
 * @brief 把关节角格式化为便于日志打印的字符串
 * @param values 关节角向量（弧度）
 * @return 形如 "[0.000, 0.000, ...]" 的字符串
 */
std::string format_joints(const std::vector<double>& values) {
    std::string text = "[";
    for (std::size_t i = 0; i < values.size(); ++i) {
        text += (i == 0 ? "" : ", ");
        text += std::to_string(values[i]);
    }
    text += "]";
    return text;
}

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

    const double grace_param = node->declare_parameter<double>("reset_grace", 3.0);
    reset_grace_             = grace_param >= 0.0 ? grace_param : 3.0;

    const double timeout_param = node->declare_parameter<double>("reset_timeout", 0.0);
    reset_timeout_             = timeout_param > 0.0 ? timeout_param : reset_duration_ + reset_grace_ + 9.0;

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
    if (!ctx_->require_controller_idel("reset")) {
        ctx_->fail("reset task requires the controller to be in idel before starting");
        done_ = true;
        return true;
    }
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

    const double elapsed  = (time - enter_time_).seconds();
    const bool has_states = ctx_->have_joint_states() && ctx_->joint_positions().size() == target_joint_pos_.size();

    if (has_states) {
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

    // 保险：复位时长加宽限时间后仍未到位（例如控制器侧重力静差超过容差而卡在 reset），
    // 只告警并结束本任务，让任务链继续往下走，不阻塞后继任务。
    if (elapsed >= reset_duration_ + reset_grace_) {
        log_not_reached(elapsed, "reset did not settle within reset_duration + reset_grace");
        RCLCPP_WARN(ctx_->node()->get_logger(), "reset task skipped after %.2f s; continuing with the next task", elapsed);
        done_ = true;
        return true;
    }

    // 兜底：宽限时间被配得很大、或始终收不到 /joint_states 时的最终上限。
    if (elapsed > reset_timeout_) {
        log_not_reached(elapsed, "reset did not finish before reset_timeout");
        RCLCPP_WARN(ctx_->node()->get_logger(), "reset task timed out after %.2f s; continuing with the next task", elapsed);
        done_ = true;
    }
    return true;
}

void ResetTask::log_not_reached(double elapsed, const std::string& reason) const {
    const rclcpp::Logger logger = ctx_->node()->get_logger();

    if (!ctx_->have_joint_states() || ctx_->joint_positions().size() != target_joint_pos_.size()) {
        RCLCPP_WARN(
            logger, "reset task: %s after %.2f s: no valid /joint_states (target %s)", reason.c_str(), elapsed,
            format_joints(target_joint_pos_).c_str());
        return;
    }

    const auto& joints = ctx_->joint_positions();
    double max_error   = 0.0;
    std::size_t worst  = 0;
    for (std::size_t i = 0; i < joints.size(); ++i) {
        const double error = std::abs(joints[i] - target_joint_pos_[i]);
        if (error > max_error) {
            max_error = error;
            worst     = i;
        }
    }

    RCLCPP_WARN(
        logger, "reset task: %s after %.2f s: max joint error %.4f rad on joint%d (tolerance %.4f rad)",
        reason.c_str(), elapsed, max_error, static_cast<int>(worst) + 1, reset_tolerance_);
    RCLCPP_WARN(logger, "  target (rad): %s", format_joints(target_joint_pos_).c_str());
    RCLCPP_WARN(logger, "  actual (rad): %s", format_joints(joints).c_str());
    RCLCPP_WARN(
        logger,
        "  the arm is still holding the reset target, so the following task starts from it; check reset_joint_pos / "
        "reset_tolerance on both nodes and the sim controller tracking error");
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
