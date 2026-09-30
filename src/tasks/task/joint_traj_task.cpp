#include "joint_traj_task.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace {

/// 任务链末端任务名，任务失败时回退到该任务。
const std::string kIdelTaskName = "idel";

/// 把关节角转成轨迹消息需要的 float 向量。
std::vector<float> to_float_vector(const std::vector<double>& values) {
    std::vector<float> result;
    result.reserve(values.size());
    for (const double value : values) {
        result.push_back(static_cast<float>(value));
    }
    return result;
}

}  // namespace

JointTrajTask::JointTrajTask(const std::string& task_name, std::any ctx)
    : TaskFSM(task_name, ctx)
    , ctx_(std::any_cast<TaskContext*>(ctx)) {
    if (ctx_ == nullptr) {
        throw std::invalid_argument("JointTrajTask requires a TaskContext");
    }

    auto* node = ctx_->node().get();
    target_joints_ =
        node->declare_parameter<std::vector<double>>("joint_traj_target_positions", std::vector<double>{});
    settle_duration_ =
        node->declare_parameter<double>("joint_traj_settle_duration", settle_duration_);
    motion_duration_ =
        node->declare_parameter<double>("joint_traj_motion_duration", motion_duration_);
    position_tolerance_ =
        node->declare_parameter<double>("joint_traj_position_tolerance", position_tolerance_);

    const double timeout_param = node->declare_parameter<double>("joint_traj_timeout", 0.0);
    timeout_                   = timeout_param > 0.0 ? timeout_param : motion_duration_ + 10.0;
}

bool JointTrajTask::enter(const std::string& last_task, const rclcpp::Time& time) {
    (void)last_task;

    phase_            = Phase::kSettle;
    phase_start_time_ = time;

    const std::size_t joint_count = ctx_->options().joints.size();
    if (target_joints_.size() != joint_count) {
        RCLCPP_ERROR(
            ctx_->node()->get_logger(),
            "joint_traj_target_positions must have %zu values (got %zu); set it with "
            "-p joint_traj_target_positions:=\"[...]\"",
            joint_count, target_joints_.size());
        ctx_->fail("joint_traj_target_positions is missing or has the wrong size");
        phase_ = Phase::kDone;
        return true;
    }

    if (!ctx_->set_exp_state("joint_traj")) {
        ctx_->fail("failed to switch arm_controller to joint_traj");
        phase_ = Phase::kDone;
    }
    return true;
}

bool JointTrajTask::run(const rclcpp::Time& time) {
    if (phase_ == Phase::kDone) {
        return true;
    }

    const double elapsed = (time - phase_start_time_).seconds();

    if (phase_ == Phase::kSettle) {
        if (elapsed < settle_duration_) {
            return true;
        }
        if (!ctx_->have_joint_states()) {
            if (elapsed > timeout_) {
                ctx_->fail("unable to read the joint states before publishing the joint trajectory");
                phase_ = Phase::kDone;
            }
            return true;
        }

        const std::vector<double>& start_joints = ctx_->joint_positions();
        const std::vector<std::vector<float>> points{to_float_vector(start_joints), to_float_vector(target_joints_)};
        const std::vector<float> seconds{0.0f, static_cast<float>(motion_duration_)};
        if (!ctx_->publish_joint_traj(points, seconds)) {
            ctx_->fail("failed to publish the joint trajectory");
            phase_ = Phase::kDone;
            return true;
        }

        RCLCPP_INFO(
            ctx_->node()->get_logger(), "joint trajectory published, moving to the target in %.2f s", motion_duration_);

        phase_            = Phase::kMoving;
        phase_start_time_ = time;
        return true;
    }

    // kMoving：等运动执行完，再检查关节到位误差。
    if (elapsed < motion_duration_) {
        return true;
    }

    if (ctx_->have_joint_states() && ctx_->joint_positions().size() == target_joints_.size()) {
        const auto& joints = ctx_->joint_positions();
        double max_error   = 0.0;
        for (std::size_t i = 0; i < joints.size(); ++i) {
            max_error = std::max(max_error, std::abs(joints[i] - target_joints_[i]));
        }

        if (max_error <= position_tolerance_) {
            RCLCPP_INFO(ctx_->node()->get_logger(), "joint trajectory finished, max joint error %.4f rad", max_error);
            phase_ = Phase::kDone;
            return true;
        }
        if (elapsed >= timeout_) {
            RCLCPP_WARN(
                ctx_->node()->get_logger(),
                "joint trajectory finished with a residual joint error of %.4f rad (tolerance %.4f rad)", max_error,
                position_tolerance_);
            phase_ = Phase::kDone;
        }
        return true;
    }

    if (elapsed >= timeout_) {
        ctx_->fail("unable to verify the joint positions after the joint trajectory");
        phase_ = Phase::kDone;
    }
    return true;
}

const std::string& JointTrajTask::check_switch() const {
    if (phase_ != Phase::kDone) {
        return task_name_;
    }
    if (ctx_->failed()) {
        return kIdelTaskName;
    }
    return next_task_.empty() ? kIdelTaskName : next_task_;
}
