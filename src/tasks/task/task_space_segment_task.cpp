#include "task_space_segment_task.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace {

/// 任务链末端任务名，任务失败时回退到该任务。
const std::string kIdelTaskName = "idel";

/// 任务空间自由度：3 位置 + 3 旋转向量。
constexpr std::size_t kTaskDof = 6;

/// 把任务空间位姿转成轨迹消息需要的 float 向量。
std::vector<float> to_float_vector(const std::vector<double>& values) {
    std::vector<float> result;
    result.reserve(values.size());
    for (const double value : values) {
        result.push_back(static_cast<float>(value));
    }
    return result;
}

}  // namespace

TaskSpaceSegmentTask::TaskSpaceSegmentTask(
    const std::string& task_name, std::any ctx, const std::string& parameter_prefix, double default_position_tolerance)
    : TaskFSM(task_name, ctx)
    , context_(std::any_cast<TaskContext*>(ctx))
    , position_tolerance_(default_position_tolerance)
    , parameter_prefix_(parameter_prefix) {
    if (context_ == nullptr) {
        throw std::invalid_argument("TaskSpaceSegmentTask requires a TaskContext");
    }

    auto* node = context_->node().get();
    target_offset_ = node->declare_parameter<std::vector<double>>(
        parameter_prefix_ + "_target_offset", std::vector<double>{0.0, 0.0, 0.05, 0.0, 0.0, 0.0});
    absolute_pose_ =
        node->declare_parameter<std::vector<double>>(parameter_prefix_ + "_target_pose", std::vector<double>{});
    settle_duration_ =
        node->declare_parameter<double>(parameter_prefix_ + "_settle_duration", settle_duration_);
    motion_duration_ =
        node->declare_parameter<double>(parameter_prefix_ + "_motion_duration", motion_duration_);
    position_tolerance_ =
        node->declare_parameter<double>(parameter_prefix_ + "_position_tolerance", position_tolerance_);

    const double timeout_param = node->declare_parameter<double>(parameter_prefix_ + "_timeout", 0.0);
    timeout_                   = timeout_param > 0.0 ? timeout_param : motion_duration_ + 10.0;

    if (!absolute_pose_.empty() && absolute_pose_.size() != kTaskDof) {
        RCLCPP_WARN(
            node->get_logger(), "%s_target_pose must have %zu values, got %zu; falling back to the offset",
            parameter_prefix_.c_str(), kTaskDof, absolute_pose_.size());
        absolute_pose_.clear();
    }
    if (target_offset_.size() != kTaskDof) {
        RCLCPP_WARN(
            node->get_logger(), "%s_target_offset must have %zu values, got %zu; using zeros", parameter_prefix_.c_str(),
            kTaskDof, target_offset_.size());
        target_offset_.assign(kTaskDof, 0.0);
    }
}

bool TaskSpaceSegmentTask::enter(const std::string& last_task, const rclcpp::Time& time) {
    (void)last_task;

    phase_            = Phase::kSettle;
    phase_start_time_ = time;
    if (!context_->set_exp_state(parameter_prefix_)) {
        context_->fail("failed to switch arm_controller to " + parameter_prefix_);
        phase_ = Phase::kDone;
    }
    return true;
}

bool TaskSpaceSegmentTask::run(const rclcpp::Time& time) {
    if (phase_ == Phase::kDone) {
        return true;
    }

    const double elapsed = (time - phase_start_time_).seconds();

    if (phase_ == Phase::kSettle) {
        if (elapsed < settle_duration_) {
            return true;
        }
        if (!context_->end_effector_pose(&start_pose_)) {
            if (elapsed > timeout_) {
                context_->fail("unable to read the end effector pose before publishing the task space trajectory");
                phase_ = Phase::kDone;
            }
            return true;
        }

        compute_target_pose(start_pose_);
        const std::vector<std::vector<float>> points{to_float_vector(start_pose_), to_float_vector(target_pose_)};
        const std::vector<float> seconds{0.0f, static_cast<float>(motion_duration_)};
        if (!publish_task_space(points, seconds)) {
            context_->fail("failed to publish the task space trajectory");
            phase_ = Phase::kDone;
            return true;
        }

        RCLCPP_INFO(
            context_->node()->get_logger(),
            "%s task: start [%.3f %.3f %.3f], target [%.3f %.3f %.3f], duration %.2f s", task_kind().c_str(),
            start_pose_[0], start_pose_[1], start_pose_[2], target_pose_[0], target_pose_[1], target_pose_[2],
            motion_duration_);

        phase_            = Phase::kMoving;
        phase_start_time_ = time;
        return true;
    }

    // kMoving：等运动执行完，再检查末端到位误差。
    if (elapsed < motion_duration_) {
        return true;
    }

    std::vector<double> current_pose;
    if (context_->end_effector_pose(&current_pose)) {
        double squared_error = 0.0;
        for (std::size_t i = 0; i < 3; ++i) {
            const double error = current_pose[i] - target_pose_[i];
            squared_error += error * error;
        }
        const double position_error = std::sqrt(squared_error);

        if (position_error <= position_tolerance_) {
            RCLCPP_INFO(
                context_->node()->get_logger(), "%s task finished, position error %.4f m", task_kind().c_str(),
                position_error);
            phase_ = Phase::kDone;
            return true;
        }
        if (elapsed >= timeout_) {
            RCLCPP_WARN(
                context_->node()->get_logger(),
                "%s task finished with a residual position error of %.4f m (tolerance %.4f m)", task_kind().c_str(),
                position_error, position_tolerance_);
            phase_ = Phase::kDone;
        }
        return true;
    }

    if (elapsed >= timeout_) {
        context_->fail("unable to verify the end effector pose after the task space trajectory");
        phase_ = Phase::kDone;
    }
    return true;
}

const std::string& TaskSpaceSegmentTask::check_switch() const {
    if (phase_ != Phase::kDone) {
        return task_name_;
    }
    if (context_->failed()) {
        return kIdelTaskName;
    }
    return next_task_.empty() ? kIdelTaskName : next_task_;
}

void TaskSpaceSegmentTask::compute_target_pose(const std::vector<double>& start_pose) {
    if (!absolute_pose_.empty()) {
        target_pose_ = absolute_pose_;
        return;
    }

    target_pose_.resize(start_pose.size(), 0.0);
    for (std::size_t i = 0; i < start_pose.size(); ++i) {
        target_pose_[i] = start_pose[i] + (i < target_offset_.size() ? target_offset_[i] : 0.0);
    }
}
