#include "cart_traj_task.hpp"

namespace {

/// 任务类型名，仅用于日志。
const std::string kTaskKind = "cartesian trajectory";

/// 到位容差默认值（米）。
constexpr double kDefaultPositionTolerance = 0.005;

}  // namespace

CartTrajTask::CartTrajTask(const std::string& task_name, std::any ctx)
    : TaskSpaceSegmentTask(task_name, ctx, "cart_traj", kDefaultPositionTolerance) {}

bool CartTrajTask::publish_task_space(
    const std::vector<std::vector<float>>& points, const std::vector<float>& seconds) {
    return context_->publish_cart_traj(points, seconds);
}

const std::string& CartTrajTask::task_kind() const {
    return kTaskKind;
}
