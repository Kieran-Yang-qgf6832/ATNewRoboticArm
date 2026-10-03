/**
 * @file cart_traj_task.hpp
 * @brief 笛卡尔轨迹任务声明
 *
 * 请求控制器切换到 @c cart_traj 控制模式，然后向 @c /arm_cart_traj 发布一条
 * 两点笛卡尔轨迹：起点取当前末端位姿（TF 查询），终点为起点叠加给定偏移
 * （或直接指定的绝对目标位姿）。控制器在两点之间做五次插值、逆运动学求解，
 * 再用逆动力学算出关节力矩。
 *
 * 流程与参数见基类 @ref TaskSpaceSegmentTask（参数前缀为 @c cart_traj_）。
 *
 * @see TaskSpaceSegmentTask
 * @see AdmittanceTask
 *
 * @author lyz
 * @date 2026-10-03
 * @version 0.0.0
 */

#pragma once

#include <any>
#include <string>
#include <vector>

#include "task_space_segment_task.hpp"

/**
 * @brief 笛卡尔轨迹任务
 *
 * 参数：@c cart_traj_target_offset / @c cart_traj_target_pose /
 * @c cart_traj_settle_duration / @c cart_traj_motion_duration /
 * @c cart_traj_position_tolerance（默认 0.005 m）/ @c cart_traj_timeout。
 */
class CartTrajTask : public TaskSpaceSegmentTask {
public:
    /**
     * @brief 构造函数
     * @param task_name 任务名，固定为 "cart_traj"
     * @param ctx       上下文，实际类型为 @c TaskContext*
     * @throws std::invalid_argument 上下文为空指针时抛出
     */
    CartTrajTask(const std::string& task_name, std::any ctx);

protected:
    /// 通过 /arm_cart_traj 发布笛卡尔轨迹。
    bool publish_task_space(const std::vector<std::vector<float>>& points, const std::vector<float>& seconds) override;

    /// @return 日志用任务类型名
    const std::string& task_kind() const override;
};
