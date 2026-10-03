/**
 * @file measure_task.hpp
 * @brief 参数辨识任务声明
 *
 * 请求控制器切换到 @c measure 控制模式，之后控制器会自行完成整个辨识流程：
 * 生成傅里叶激励轨迹 -> 移动到轨迹起点 -> 执行激励轨迹并同步采集位置/速度/力矩
 * -> 把数据写入 CSV（控制器参数 @c measure_csv_file_path，默认
 * @c /tmp/measured_for_identification.csv）。
 *
 * 任务层无法观测控制器内部阶段，因此按固定时长推进：
 * 等待 @c measure_duration -> 请求切回 idel -> 再等 @c measure_return_settle 结束本任务。
 *
 * @warning 控制器对 @c measure 有硬约束——只能从 @c idel 进入、只能回到 @c idel
 *          （见 arm_controller 的 ParamterMeasureState::enter/exit）。runner 会自动
 *          在本任务前后补一个 idel 任务。
 * @warning 控制器只在测量结束（@c HoldingEnd）后才响应 @c exp_state，所以提前请求
 *          idel 是安全的；但 @c measure_duration 必须不小于控制器侧总时长，否则任务层
 *          会过早进入下一个任务，可能让控制器在离开 @c measure 时 @c enter() 失败
 *          （例如 @c teach_pendant 要求上一个状态必须是 idel）。
 *          控制器侧总时长约为：
 *          @c measure_move_to_start_duration + @c measure_trajectory_period ×
 *          @c measure_trajectory_repeat_cnt + 激励轨迹生成时间（默认约 3 + 10 × 1 = 13 s）。
 * @warning 测量期间机械臂会按激励轨迹自动运动，请确保工作空间安全。
 *
 * @see TaskFSM
 * @see TaskContext
 *
 * @author lyz
 * @date 2026-10-03
 * @version 0.0.0
 */

#pragma once

#include <any>
#include <string>

#include "task_context.hpp"
#include "task_fsm.hpp"

/**
 * @brief 参数辨识任务
 *
 * 参数：
 * - @c measure_duration：任务层在 measure 状态保持的时长（秒，默认 20.0）；
 *   小于等于 0 表示不主动请求回 idel，一直保持到外部中断；
 * - @c measure_return_settle：请求回 idel 后再等待的时间（秒，默认 1.0），
 *   留给控制器完成状态切换。
 */
class MeasureTask : public TaskFSM {
public:
    /**
     * @brief 任务阶段
     */
    enum class Phase {
        kMeasuring, ///< 等待控制器完成测量
        kReturning, ///< 已请求回 idel，等待切换完成
        kDone,      ///< 完成或失败
    };

    /**
     * @brief 构造函数
     * @param task_name 任务名，固定为 "measure"
     * @param ctx       上下文，实际类型为 @c TaskContext*
     * @throws std::invalid_argument 上下文为空指针时抛出
     */
    MeasureTask(const std::string& task_name, std::any ctx);

    /// 请求控制器切换到 measure，进入 kMeasuring 阶段。
    bool enter(const std::string& last_task, const rclcpp::Time& time) override;

    /// 计时结束后请求切回 idel，再留出切换时间后结束本任务。
    bool run(const rclcpp::Time& time) override;

    /// @return 测量结束后返回后继任务名（runner 自动成 idel），否则留在本任务
    const std::string& check_switch() const override;

private:
    TaskContext* ctx_{nullptr};      ///< 共享上下文
    Phase phase_{Phase::kMeasuring}; ///< 当前阶段
    rclcpp::Time phase_start_time_;  ///< 当前阶段开始时刻
    double duration_{20.0};          ///< 任务层等待时长（秒），<= 0 表示不限时
    double return_settle_{1.0};      ///< 请求回 idel 后的等待时长（秒）
};
