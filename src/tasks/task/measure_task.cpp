#include "measure_task.hpp"

#include <stdexcept>

namespace {

/// 任务链末端任务名，任务失败时回退到该任务。
const std::string kIdelTaskName = "idel";

}  // namespace

MeasureTask::MeasureTask(const std::string& task_name, std::any ctx)
    : TaskFSM(task_name, ctx)
    , ctx_(std::any_cast<TaskContext*>(ctx)) {
    if (ctx_ == nullptr) {
        throw std::invalid_argument("MeasureTask requires a TaskContext");
    }

    auto* node   = ctx_->node().get();
    duration_    = node->declare_parameter<double>("measure_duration", duration_);
    return_settle_ = node->declare_parameter<double>("measure_return_settle", return_settle_);
}

bool MeasureTask::enter(const std::string& last_task, const rclcpp::Time& time) {
    (void)last_task;

    phase_            = Phase::kMeasuring;
    phase_start_time_ = time;

    if (!ctx_->require_controller_idel("parameter measurement")) {
        ctx_->fail("measure task requires the controller to be in idel before starting");
        phase_ = Phase::kDone;
        return true;
    }

    if (!ctx_->set_exp_state("measure")) {
        ctx_->fail("failed to switch arm_controller to measure");
        phase_ = Phase::kDone;
        return true;
    }

    if (duration_ > 0.0) {
        RCLCPP_INFO(
            ctx_->node()->get_logger(),
            "parameter identification started, the arm will move along the excitation trajectory; "
            "waiting %.1f s before returning to idel",
            duration_);
    } else {
        RCLCPP_INFO(
            ctx_->node()->get_logger(),
            "parameter identification started without a time limit; interrupt the runner to leave this task");
    }
    return true;
}

bool MeasureTask::run(const rclcpp::Time& time) {
    if (phase_ == Phase::kDone) {
        return true;
    }

    const double elapsed = (time - phase_start_time_).seconds();

    if (phase_ == Phase::kMeasuring) {
        if (duration_ <= 0.0 || elapsed < duration_) {
            return true;
        }
        if (!ctx_->set_exp_state(kIdelTaskName)) {
            ctx_->fail("failed to switch arm_controller back to idel after parameter identification");
            phase_ = Phase::kDone;
            return true;
        }

        RCLCPP_INFO(
            ctx_->node()->get_logger(),
            "parameter identification finished; the data was written by the controller to the "
            "measure_csv_file_path parameter (default /tmp/measured_for_identification.csv)");
        phase_            = Phase::kReturning;
        phase_start_time_ = time;
        return true;
    }

    // kReturning：留一点时间让控制器完成回到 idel 的切换。
    if (elapsed >= return_settle_) {
        phase_ = Phase::kDone;
    }
    return true;
}

const std::string& MeasureTask::check_switch() const {
    if (phase_ != Phase::kDone) {
        return task_name_;
    }
    if (ctx_->failed()) {
        return kIdelTaskName;
    }
    return next_task_.empty() ? kIdelTaskName : next_task_;
}
