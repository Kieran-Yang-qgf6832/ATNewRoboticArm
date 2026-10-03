/**
 * @file task_space_segment_task.hpp
 * @brief 任务空间两点轨迹任务的公共实现声明
 *
 * 把「读当前末端位姿 -> 拼两点任务空间轨迹 -> 发布 -> 等运动时长 -> 校验到位」
 * 这套流程抽成基类，子类只需决定轨迹发给哪个控制器状态 / 哪个话题。
 *
 * 任务分三个阶段：
 * -# @c kSettle：等待控制器真正切入目标状态；
 * -# @c kMoving：发布轨迹并等待运动时长；
 * -# @c kDone：检查到位误差，切换到后继任务。
 *
 * @note 起点必须取当前末端位姿：控制器的逆运动学从零种子迭代求解，轨迹首点
 *       若与当前位姿不一致，机械臂会在 t=0 直接跳过去。
 * @note 旋转部分使用旋转向量（angle * axis），因此姿态偏移做向量相加只在
 *       偏转角较小时近似成立。
 *
 * @see CartTrajTask
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

#include "task_context.hpp"
#include "task_fsm.hpp"

/**
 * @brief 任务空间两点轨迹任务基类
 *
 * 参数名统一由 @c parameter_prefix 派生（该前缀同时也是控制器状态名）：
 * - @c <prefix>_target_offset：相对起点的位姿偏移，6 个元素，默认 {0, 0, 0.05, 0, 0, 0}；
 * - @c <prefix>_target_pose：绝对目标位姿（6 个元素）；给定时忽略上面的偏移；
 * - @c <prefix>_settle_duration：切入后的等待时间（秒，默认 0.3）；
 * - @c <prefix>_motion_duration：轨迹运动时长（秒，默认 3.0）；
 * - @c <prefix>_position_tolerance：到位判定容差（米，子类给出默认值）；
 * - @c <prefix>_timeout：任务层超时（秒，默认 @c <prefix>_motion_duration + 10）。
 */
class TaskSpaceSegmentTask : public TaskFSM {
public:
    /**
     * @brief 任务阶段
     */
    enum class Phase {
        kSettle, ///< 等待控制器切入目标状态
        kMoving, ///< 轨迹执行中
        kDone,   ///< 完成或失败
    };

    /**
     * @brief 构造函数，读取并以 @p parameter_prefix 为前缀声明全部参数
     * @param task_name                  任务名
     * @param ctx                        上下文，实际类型为 @c TaskContext*
     * @param parameter_prefix           参数前缀，同时作为控制器状态名
     * @param default_position_tolerance 到位容差默认值（米）
     * @throws std::invalid_argument 上下文为空指针时抛出
     */
    TaskSpaceSegmentTask(
        const std::string& task_name, std::any ctx, const std::string& parameter_prefix,
        double default_position_tolerance);

    /// 请求控制器切入目标状态，进入 kSettle 阶段。
    bool enter(const std::string& last_task, const rclcpp::Time& time) override;

    /// 按阶段推进：等切换稳定 -> 发布轨迹 -> 等待到位。
    bool run(const rclcpp::Time& time) override;

    /// @return 轨迹执行完成后返回后继任务名，否则留在本任务
    const std::string& check_switch() const override;

protected:
    /**
     * @brief 发布一条任务空间两点轨迹
     * @param points  轨迹点，每个点为 6 维任务空间向量
     * @param seconds 与 @p points 一一对应的时刻（秒）
     * @return true 发布成功
     */
    virtual bool publish_task_space(const std::vector<std::vector<float>>& points, const std::vector<float>& seconds) = 0;

    /// @return 任务类型名，仅用于日志
    virtual const std::string& task_kind() const = 0;

    TaskContext* context_{nullptr};      ///< 共享上下文
    Phase phase_{Phase::kSettle};        ///< 当前阶段
    rclcpp::Time phase_start_time_;      ///< 当前阶段开始时刻
    std::vector<double> start_pose_;     ///< 轨迹起点位姿
    std::vector<double> target_pose_;    ///< 轨迹终点位姿
    std::vector<double> target_offset_;  ///< 相对起点的偏移
    std::vector<double> absolute_pose_;  ///< 绝对目标位姿参数（为空表示使用偏移）
    double settle_duration_{0.3};        ///< 切入等待时间（秒）
    double motion_duration_{3.0};        ///< 轨迹运动时长（秒）
    double position_tolerance_{0.005};   ///< 到位容差（米）
    double timeout_{13.0};               ///< 任务层超时（秒）

private:
    /**
     * @brief 由起点位姿与参数计算目标位姿
     * @param start_pose 起点位姿（6 维任务空间向量）
     */
    void compute_target_pose(const std::vector<double>& start_pose);

    std::string parameter_prefix_; ///< 参数前缀 / 控制器状态名
};
