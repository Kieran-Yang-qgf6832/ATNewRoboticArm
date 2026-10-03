#include "teach_pendant_task.hpp"

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

TeachPendantTask::TeachPendantTask(const std::string& task_name, std::any ctx)
    : TaskFSM(task_name, ctx)
    , ctx_(std::any_cast<TaskContext*>(ctx)) {
    if (ctx_ == nullptr) {
        throw std::invalid_argument("TeachPendantTask requires a TaskContext");
    }
    duration_ = ctx_->node()->declare_parameter<double>("teach_pendant_duration", duration_);
}

bool TeachPendantTask::enter(const std::string& last_task, const rclcpp::Time& time) {
    (void)last_task;

    enter_time_ = time;
    done_       = false;
    if (ctx_->have_joint_states()) {
        start_joints_ = ctx_->joint_positions();
    } else {
        start_joints_.clear();
    }

    if (!ctx_->set_exp_state("teach_pendant")) {
        ctx_->fail("failed to switch arm_controller to teach_pendant");
        done_ = true;
        return true;
    }

    if (duration_ > 0.0) {
        RCLCPP_INFO(
            ctx_->node()->get_logger(), "teach pendant enabled for %.1f s: drag the arm by hand", duration_);
    } else {
        RCLCPP_INFO(
            ctx_->node()->get_logger(),
            "teach pendant enabled without a time limit; interrupt the runner to leave this task");
    }
    return true;
}

bool TeachPendantTask::run(const rclcpp::Time& time) {
    if (done_ || duration_ <= 0.0) {
        return true;
    }
    if ((time - enter_time_).seconds() >= duration_) {
        finish_teaching();
    }
    return true;
}

const std::string& TeachPendantTask::check_switch() const {
    if (!done_) {
        return task_name_;
    }
    if (ctx_->failed()) {
        return kIdelTaskName;
    }
    return next_task_.empty() ? kIdelTaskName : next_task_;
}

void TeachPendantTask::finish_teaching() {
    done_ = true;

    if (!ctx_->set_exp_state(kIdelTaskName)) {
        ctx_->fail("failed to switch arm_controller back to idel after teaching");
        return;
    }

    if (start_joints_.empty() || !ctx_->have_joint_states() || ctx_->joint_positions().size() != start_joints_.size()) {
        RCLCPP_INFO(ctx_->node()->get_logger(), "teach pendant finished");
        return;
    }

    std::vector<double> delta(start_joints_.size(), 0.0);
    for (std::size_t i = 0; i < delta.size(); ++i) {
        delta[i] = ctx_->joint_positions()[i] - start_joints_[i];
    }
    RCLCPP_INFO(
        ctx_->node()->get_logger(), "teach pendant finished, joint delta (rad): %s", format_joints(delta).c_str());
}
