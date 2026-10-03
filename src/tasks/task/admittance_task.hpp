/**
 * @file admittance_task.hpp
 * @brief 导纳柔顺任务声明
 *
 * 请求控制器切换到 @c admittance 控制模式，然后向 @c /arm_admittance 发布一条
 * 两点任务空间轨迹，作为导纳模型的「期望轨迹」：
 * - 起点取当前末端位姿（TF 查询），终点为起点叠加给定偏移（或指定的绝对目标位姿）；
 * - 控制器估计外部力（实测力矩 − 模型力矩），按质量-阻尼-刚度模型生成柔顺修正，
 *   因此末端实际轨迹会偏离期望轨迹，这是预期行为。
 *
 * 与 @ref CartTrajTask 的差别只有「发到哪个话题」：
 * - 期望轨迹不再是硬跟踪，而是柔顺跟随，所以默认到位容差更宽（0.02 m）；
 * - 轨迹时间走完后控制器自动保持当前位置并允许切换状态，任务层据此结束。
 *
 * @note @c AdmittanceCmd.force 字段当前控制器未使用（力由控制器自行估计），
 *       因此本任务只填充 @c seconds 与 @c position。
 * @note 导纳参数（@c admittance_mass / @c admittance_damping / @c admittance_stiffness）
 *       属于控制器侧参数，需在 ros2_controller.yaml 中配置。
 *
 * @see TaskSpaceSegmentTask
 * @see CartTrajTask
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
 * @brief 导纳柔顺任务
 *
 * 参数：@c admittance_target_offset / @c admittance_target_pose /
 * @c admittance_settle_duration / @c admittance_motion_duration /
 * @c admittance_position_tolerance（默认 0.02 m）/ @c admittance_timeout。
 */
class AdmittanceTask : public TaskSpaceSegmentTask {
public:
    /**
     * @brief 构造函数
     * @param task_name 任务名，固定为 "admittance"
     * @param ctx       上下文，实际类型为 @c TaskContext*
     * @throws std::invalid_argument 上下文为空指针时抛出
     */
    AdmittanceTask(const std::string& task_name, std::any ctx);

protected:
    /// 通过 /arm_admittance 发布期望轨迹。
    bool publish_task_space(const std::vector<std::vector<float>>& points, const std::vector<float>& seconds) override;

    /// @return 日志用任务类型名
    const std::string& task_kind() const override;
};
