#include "admittance_task.hpp"

namespace {

/// 任务类型名，仅用于日志。
const std::string kTaskKind = "admittance";

/// 到位容差默认值（米）：导纳控制下末端会柔顺偏离期望轨迹，容差放宽。
constexpr double kDefaultPositionTolerance = 0.02;

}  // namespace

AdmittanceTask::AdmittanceTask(const std::string& task_name, std::any ctx)
    : TaskSpaceSegmentTask(task_name, ctx, "admittance", kDefaultPositionTolerance) {}

bool AdmittanceTask::publish_task_space(
    const std::vector<std::vector<float>>& points, const std::vector<float>& seconds) {
    return context_->publish_admittance(points, seconds);
}

const std::string& AdmittanceTask::task_kind() const {
    return kTaskKind;
}
