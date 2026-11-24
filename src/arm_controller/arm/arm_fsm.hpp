/**
 * @file arm_fsm.hpp
 * @brief 机械臂有限状态机（FSM）状态类声明
 *
 * 定义了机械臂所有控制模式的状态类，包括空闲、复位、轨迹跟踪、伺服、
 * 导纳控制、示教器和参数辨识等状态。每个状态实现 FSM 基类的
 * enter/exit/check_switch/run 接口。
 *
 * @author lyz
 * @date 2026-09-28
 * @version 0.0.0
 *
 * @see FSM
 * @see FSMArmControlFactory
 * @see controller.hpp
 */

#pragma once

#include "fourier_trajectory.hpp"
#include "fsm.hpp"

#include <any>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include "excitation_trajectory.hpp"
#include "paramter_identify.hpp"
#include "task.hpp"
#include "trajectory.hpp"
#include "waypoint_trajectory.hpp"

#include <robot_msgs/msg/admittance_cmd.hpp>
#include <robot_msgs/msg/base_motion_state.hpp>
#include <robot_msgs/msg/cart_traj_cmd.hpp>
#include <robot_msgs/msg/inertial_hold_status.hpp>
#include <robot_msgs/msg/joint_traj_cmd.hpp>
#include <robot_msgs/msg/servo_ctrl_cmd.hpp>
#include <robot_msgs/msg/waypoint_traj_cmd.hpp>
#include <std_msgs/msg/float64.hpp>

class FSMArmControlFactory;

/**
 * @brief 保持路径的重力前馈（docs/bugs.md O6 / O7）
 *
 * \c reset / \c idel 与所有 \c write_hold_command() 保持路径共用同一套补偿：用 RNEA
 * 在 \c dq = ddq = 0 下算出重力力矩，写入 \c command.torque 作为力矩前馈。
 *
 * 没有它时保持只剩 kp 的刚度，静差为 τ/kp：零位姿下 joint4 约 0.124 rad，复位永远
 * 达不到 \c reset_tolerance（O6）；\c cart_traj / \c joint_traj / \c admittance 在
 * 等待轨迹到达的约 0.9 s 里 joint4 会先塌陷约 0.089 rad（5.1°），**笛卡尔运动的
 * 起点因此不是指令要求的位姿且没有任何提示**（O7）。
 *
 * 实时约束：buffer 只在 \c initialize()（构造阶段）分配；\c refresh() 只读写已分配
 * 内存、不做动态分配，可在 \c run() 中每周期调用。求解失败（模型不可用 / 抛异常 /
 * 结果非有限值）时 \c refresh() 返回 false 并打限流 WARN，调用方退回 \c torque = 0
 * ——即退化为无补偿的行为，有静差但不会失控。
 *
 * @see compute_gravity_torque
 */
class HoldGravityFeedforward {
public:
    /**
     * @brief 构造阶段调用一次：读参数、打诊断日志、按关节数分配 buffer
     * @param factory     状态机工厂
     * @param joint_count 关节数
     * @param state_name  状态名，仅用于日志
     */
    void initialize(FSMArmControlFactory* factory, std::size_t joint_count, const char* state_name);

    /**
     * @brief 按当前实测关节角重算重力力矩
     * @return 补偿可用返回 true；否则调用方应退回 torque = 0
     */
    bool refresh(FSMArmControlFactory* factory, std::size_t joint_count, const char* state_name);

    /// 第 index 个关节的重力前馈力矩；补偿不可用时返回 0
    float torque(std::size_t index) const;

    /// 是否启用（控制器参数 gravity_compensation）
    bool enabled() const {
        return enabled_;
    }

private:
    bool enabled_{true};        ///< 是否启用，来自参数 gravity_compensation
    double scale_{1.0};         ///< 前馈缩放，来自参数 gravity_compensation_scale
    bool available_{false};     ///< 最近一次 refresh 是否成功
    Eigen::VectorXd joint_pos_; ///< 当前关节角，RNEA 输入
    Eigen::VectorXd zero_;      ///< 全零向量，充当 RNEA 的 dq / ddq
    Eigen::VectorXd torque_;    ///< 重力补偿力矩，RNEA 输出
};

/**
 * @brief 机械臂空闲状态
 *
 * 机械臂锁定在当前位置，不执行任何动作。所有关节保持位置控制，
 * 速度、力矩和积分项清零。作为系统的初始状态和安全回退状态。
 *
 * @note 此状态为 FSM 的初始状态（set_init_state("idel")）
 * @warning 进入此状态时会立即将当前位置设为目标位置，请确保机械臂已就绪
 *
 * @see FSMArmControlFactory
 */
class IDELState : public FSM {
public:
    /**
     * @brief 构造函数
     * @param name 状态名称，用于 FSM 内部标识
     * @param ctx  上下文指针，实际类型为 FSMArmControlFactory*
     * @throws std::bad_any_cast 当 ctx 无法转换为 FSMArmControlFactory* 时抛出
     */
    IDELState(const std::string& name, std::any ctx);

    /**
     * @brief 进入状态时调用
     * @param last_state 上一个状态的名称
     * @param time       进入时刻
     * @return true 进入成功；false 表示工厂或节点未就绪
     * @note 进入时会记录当前关节位置作为保持位置
     */
    bool enter(const std::string& last_state, const rclcpp::Time& time) override;

    /**
     * @brief 离开状态时调用
     * @param next_state 下一个状态的名称
     * @return 始终返回 true
     */
    bool exit(const std::string& next_state) override;

    /**
     * @brief 检查是否需要切换状态
     * @return 期望切换到的状态名称；若无需切换则返回当前状态名
     * @note 允许从空闲状态切换到任何目标状态
     */
    const std::string& check_switch() const override;

    /**
     * @brief 状态运行时周期性调用
     * @param time 当前时间
     * @return true 运行正常；false 表示工厂或指令 buffer 未就绪
     * @note 每个周期将保持位置写入指令 buffer
     */
    bool run(const rclcpp::Time& time) override;

private:
    FSMArmControlFactory* factory{nullptr}; ///< 状态机工厂指针
    std::size_t joint_count_{0};            ///< 关节数量
    std::vector<float> hold_position_;      ///< 保持位置
    std::vector<float> default_kp_;         ///< 默认位置增益
    std::vector<float> default_kd_;         ///< 默认速度增益
    std::vector<double> default_kp_param_;  ///< 位置增益参数原始值
    std::vector<double> default_kd_param_;  ///< 速度增益参数原始值

    /// 重力补偿（docs/bugs.md O6）。buffer 在构造阶段分配，run() 中不再分配内存。
    HoldGravityFeedforward gravity_;

    /**
     * @brief 消费一次性的力矩脉冲自检请求并在到点时收尾（docs/bugs/O21）
     *
     * 请求由 @c ArmController 的参数回调（参数服务线程）写进
     * @c FSMArmControlFactory::test_torque 的原子量，本函数在实时线程里只做无锁读取，
     * 不查参数（实时约束）。请求要么被消费（joint 原子量清零，随后按 duration 倒计时），
     * 要么过期作废，不会无限期挂起。
     *
     * @param time 当前控制周期时间，用于计时与 30 s 过期判定
     * @note 脉冲只在 @c idel 输出：离开 idel 时本状态的局部状态随 @c exit() 清空，
     *       轨迹态不会带着脉冲跑。
     */
    void update_test_torque_pulse(const rclcpp::Time& time);

    /// 是否正在输出力矩脉冲
    bool test_pulse_active_{false};
    /// 脉冲起点（控制周期时间）
    rclcpp::Time test_pulse_start_;
    /// 本次脉冲的关节下标（0 起）
    std::size_t test_pulse_index_{0};
    /// 本次脉冲的力矩（N·m，URDF 符号）
    float test_pulse_value_{0.0f};
    /// 本次脉冲的持续时长（秒）
    double test_pulse_duration_{0.0};
};

/**
 * @brief 机械臂复位状态
 *
 * 控制机械臂从当前位置平滑移动到预设的复位位置。支持可配置的
 * 复位时长和到位容差。复位完成后自动切换到空闲状态。
 *
 * 兜底：复位时长结束后再等待 @c reset_settle 秒仍未全部到位（例如重力静差大于
 * 容差），会打一条 WARN 并强制切回 @c idel，避免 FSM 永久卡在复位状态、静默
 * 吞掉后续所有 @c exp_state 请求。
 *
 * @warning 只能从空闲状态（idel）进入复位状态
 * @note 复位过程中使用线性插值生成轨迹
 *
 * @see IDELState
 */
class ResetState : public FSM {
public:
    /**
     * @brief 构造函数
     * @param name 状态名称
     * @param ctx  上下文指针
     * @throws std::bad_any_cast 当 ctx 无法转换时抛出
     */
    ResetState(const std::string& name, std::any ctx);

    /**
     * @brief 进入状态时调用
     * @param last_state 上一个状态名称，必须为 "idel"
     * @param time       进入时刻
     * @return true 进入成功；false 表示前置条件不满足
     */
    bool enter(const std::string& last_state, const rclcpp::Time& time) override;

    /**
     * @brief 离开状态时调用
     * @param next_state 下一个状态名称
     * @return 仅当 next_state 为 "idel" 时返回 true
     */
    bool exit(const std::string& next_state) override;

    /**
     * @brief 检查是否需要切换状态
     * @return 复位完成时返回 "idel"，否则返回当前状态名
     */
    const std::string& check_switch() const override;

    /**
     * @brief 状态运行时周期性调用
     * @param time 当前时间
     * @return true 运行正常；false 表示工厂未就绪
     */
    bool run(const rclcpp::Time& time) override;

private:
    FSMArmControlFactory* factory{nullptr};     ///< 状态机工厂指针
    std::size_t joint_count_{0};                ///< 关节数量
    std::vector<float> start_joint_pos_;        ///< 起始关节位置
    std::vector<float> reset_joint_pos_;        ///< 目标复位位置
    std::vector<float> default_kp_;             ///< 默认位置增益
    std::vector<float> default_kd_;             ///< 默认速度增益
    std::vector<double> reset_joint_pos_param_; ///< 复位位置参数原始值
    std::vector<double> default_kp_param_;      ///< 位置增益参数原始值
    std::vector<double> default_kd_param_;      ///< 速度增益参数原始值
    rclcpp::Time reset_start_time_;             ///< 复位开始时间
    float reset_duration_{3.0f};                ///< 复位时长（秒）
    float reset_tolerance_{0.01f};              ///< 到位容差（弧度）
    float reset_settle_{2.0f};                  ///< 复位时长结束后的宽限时长（秒）
    float progress_{0.0f};                      ///< 复位进度 [0, 1]
    bool reset_done_{false};                    ///< 复位完成标志
    bool give_up_warned_{false};                ///< 是否已就「超时仍未到位」告警（只告警一次）

    /// 重力补偿（docs/bugs.md O6）。buffer 在构造阶段分配，run() 中不再分配内存。
    HoldGravityFeedforward gravity_;
};

/**
 * @brief 笛卡尔空间轨迹跟踪状态
 *
 * 接收笛卡尔空间轨迹点，通过逆运动学和逆动力学求解关节指令。
 * 支持位置、速度、加速度前馈和力矩前馈。
 *
 * @note 订阅话题 "arm_cart_traj"，消息类型 CartTrajCmd
 * @warning 轨迹执行期间忽略新的轨迹指令
 *
 * @see JointTrajState
 * @see Trajectory
 */
class CartTrajState : public FSM {
public:
    /**
     * @brief 轨迹执行阶段枚举
     */
    enum class TrajPhase {
        STOP,   ///< 停止状态，等待新轨迹
        MOVING, ///< 轨迹执行中
    };

    /**
     * @brief 构造函数
     * @param name 状态名称
     * @param ctx  上下文指针
     * @throws std::bad_any_cast 当 ctx 无法转换时抛出
     */
    CartTrajState(const std::string& name, std::any ctx);

    /**
     * @brief 进入状态时调用
     * @param last_state 上一个状态名称
     * @param time       进入时刻
     * @return true 进入成功；false 表示工厂或求解器未就绪
     */
    bool enter(const std::string& last_state, const rclcpp::Time& time) override;

    /**
     * @brief 离开状态时调用
     * @param next_state 下一个状态名称
     * @return 始终返回 true
     */
    bool exit(const std::string& next_state) override;

    /**
     * @brief 检查是否需要切换状态
     * @return 轨迹停止且收到新目标时返回目标状态名
     */
    const std::string& check_switch() const override;

    /**
     * @brief 状态运行时周期性调用
     * @param time 当前时间
     * @return true 运行正常；false 表示求解失败或数据异常
     * @throws 内部捕获 Pinocchio 异常并记录警告日志
     */
    bool run(const rclcpp::Time& time) override;

private:
    Point point;                            ///< 当前轨迹点（预分配 buffer）
    TrajPhase state{TrajPhase::STOP};       ///< 当前轨迹阶段
    Trajectory traj;                        ///< 轨迹对象
    rclcpp::Time traj_start_time_;          ///< 轨迹开始时间
    Eigen::VectorXd joint_pos_;             ///< 关节位置
    Eigen::VectorXd torque_;                ///< 关节力矩
    Eigen::VectorXd task_force_;            ///< 任务空间力
    std::vector<float> default_kp_;         ///< 默认位置增益
    std::vector<float> default_kd_;         ///< 默认速度增益
    std::vector<float> hold_position_;      ///< 锁存的保持位置（进入锁实测、轨迹结束锁最后一帧命令，O19）
    std::vector<double> default_kp_param_;  ///< 位置增益参数原始值
    std::vector<double> default_kd_param_;  ///< 速度增益参数原始值
    std::size_t joint_count_{0};            ///< 关节数量
    std::size_t task_dof_{6};               ///< 任务空间自由度
    FSMArmControlFactory* factory{nullptr}; ///< 状态机工厂指针

    /// 保持路径的重力前馈（docs/bugs.md O7）。等待轨迹到达期间用它补掉重力静差。
    HoldGravityFeedforward gravity_;

    /// 下发前可达性预检（docs/bugs.md O2）。全部在构造阶段分配，run() 中不再分配内存。
    /// 6 维轨迹只查终点：单次逆解实测最坏约 386 us（200 次迭代），一个控制周期 2 ms。
    /// position-only 轨迹额外查若干等分中间点（绝对目标可能跨越半个工作空间），
    /// 次数受 kMaxPrecheckSamples 限制，保证 (1 + samples) × 386 us < 2 ms。
    bool precheck_pending_{false};                  ///< 是否有一条新轨迹等待预检
    Eigen::VectorXd precheck_target_;               ///< 待检终点（6 维）
    std::vector<Eigen::VectorXd> precheck_samples_; ///< 等分中间点池（构造期分配）
    int precheck_count_{0};                         ///< 本次实际启用的中间点数
    Eigen::VectorXd precheck_seed_;                 ///< 预检用的关节角种子（当前实测关节角）
    Eigen::VectorXd precheck_solution_;             ///< 预检的解算结果

    /// 当前轨迹的任务空间模式（由 CartTrajCmd.mode 决定，默认 6 维）
    PoseMode pose_mode_{PoseMode::kFullPose};
    /// 位置约束类轨迹的容差：纯位置容差（米）、工具 z 轴角容差（弧度）、零空间偏好权重
    double position_tolerance_{1e-3};
    /// 位置**通过**容差（米）：由 CartTrajCmd.position_pass_tolerance 按轨迹下发。
    /// 迭代收敛判据仍是 position_tolerance_；跑完未收敛时，停在最近可达位姿上，
    /// 位置误差在该值以内仍算成功。0 = 与收敛判据相同（严格）。
    double position_pass_tolerance_{0.0};
    /// 工具 z 轴的**通过**容差（弧度）；仅 position-and-axis 模式使用，与任务层验收同值
    double axis_tolerance_{8.726646259971648e-2}; ///< 5°
    /// 工具 z 轴的**收敛**容差（弧度）：迭代一进入就停，保证能对齐的目标精确对齐
    double axis_converge_tolerance_{1.7453292519943296e-2}; ///< 1°
    double nullspace_weight_{0.0};
    Eigen::VectorXd posture_reference_;            ///< 位置约束轨迹起点的关节角（构造期分配）
    /// 位置约束类轨迹的预检中间采样点数（构造期按实时预算 clamp）
    int precheck_sample_count_{0};
    /// 实机模式：teach_pendant / measure 等物理上不成立的状态在切入前被拒绝
    bool real_mode_{false};
    /// 位置约束类轨迹（position-only / position-and-axis）与笛卡尔多路点的放行开关。
    /// 默认 false：在下发任何运动前拒绝并回 idel。与 real_mode 解耦，实机验证后再打开。
    bool allow_constrained_traj_{false};
    bool reject_constrained_traj_{false};

    /**
     * @brief 对一个 6 维目标做一次逆解，确认可达
     * @param target   待检目标（6 维；position-only / position-and-axis 下姿态分量按模式投影）
     * @param[out] residual 不可达时写入「最近可达位姿到目标还有多远」，否则写 0
     * @return 目标可达返回 true
     */
    bool check_target_reachable(const Eigen::VectorXd& target, double* residual);

    /**
     * @brief 多路点轨迹指令回调（非实时线程）
     *
     * 仅在 @c state == TrajPhase::STOP 时受理：校验消息后逐点逆解得到关节路点，
     * 再交给 @ref WaypointTrajectory 规划一次。规划成功才切到 MOVING；
     * 失败只置 @c waypoint_plan_failed_，由 run() 统一走失效安全路径。
     */
    void on_waypoint_command(const robot_msgs::msg::WaypointTrajCmd& msg);

    /// 采样已规划的多路点轨迹并写指令；失败/到达终点时返回相应状态
    bool run_waypoint_motion(const rclcpp::Time& time);

    rclcpp::Subscription<robot_msgs::msg::CartTrajCmd>::SharedPtr cart_traj_cmd_sub_; ///< 笛卡尔轨迹指令订阅

    /// 多路点 C3 轨迹（WaypointTrajCmd）。规划只在订阅回调里做一次，run() 只采样。
    WaypointTrajectory waypoint_traj_;              ///< 规划器（构造期预分配）
    bool waypoint_active_{false};                   ///< 多路点轨迹正在执行
    bool waypoint_plan_failed_{false};              ///< 规划失败，等待 run() 统一处置
    Eigen::VectorXd waypoint_max_vel_;              ///< 关节速度上限（构造期解析）
    Eigen::VectorXd waypoint_max_acc_;              ///< 关节加速度上限（构造期解析）
    Eigen::VectorXd waypoint_max_jerk_;             ///< 关节 jerk 上限（构造期解析）
    Eigen::VectorXd waypoint_pos_;                  ///< 采样位置（构造期分配）
    Eigen::VectorXd waypoint_vel_;                  ///< 采样速度（构造期分配）
    Eigen::VectorXd waypoint_acc_;                  ///< 采样加速度（构造期分配）
    Eigen::VectorXd waypoint_jerk_;                 ///< 采样 jerk（构造期分配）
    rclcpp::Subscription<robot_msgs::msg::WaypointTrajCmd>::SharedPtr waypoint_cmd_sub_; ///< 多路点指令订阅
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr waypoint_duration_pub_;         ///< 实际规划总时长
};

/**
 * @brief 关节空间轨迹跟踪状态
 *
 * 接收关节空间轨迹点，通过逆动力学求解关节力矩指令。
 * 支持位置、速度、加速度前馈和力矩前馈。
 *
 * @note 订阅话题 "arm_joint_traj"，消息类型 JointTrajCmd
 * @warning 轨迹执行期间忽略新的轨迹指令
 *
 * @see CartTrajState
 */
class JointTrajState : public FSM {
public:
    /**
     * @brief 轨迹执行阶段枚举
     */
    enum class TrajPhase {
        STOP,   ///< 停止状态，等待新轨迹
        MOVING, ///< 轨迹执行中
    };

    /**
     * @brief 构造函数
     * @param name 状态名称
     * @param ctx  上下文指针
     * @throws std::bad_any_cast 当 ctx 无法转换时抛出
     */
    JointTrajState(const std::string& name, std::any ctx);

    /**
     * @brief 进入状态时调用
     * @param last_state 上一个状态名称
     * @param time       进入时刻
     * @return true 进入成功；false 表示工厂或模型未就绪
     */
    bool enter(const std::string& last_state, const rclcpp::Time& time) override;

    /**
     * @brief 离开状态时调用
     * @param next_state 下一个状态名称
     * @return 始终返回 true
     */
    bool exit(const std::string& next_state) override;

    /**
     * @brief 检查是否需要切换状态
     * @return 轨迹停止且收到新目标时返回目标状态名
     */
    const std::string& check_switch() const override;

    /**
     * @brief 状态运行时周期性调用
     * @param time 当前时间
     * @return true 运行正常；false 表示求解失败或数据异常
     * @throws 内部捕获 Pinocchio 异常并记录警告日志
     */
    bool run(const rclcpp::Time& time) override;

private:
    Point point;                            ///< 当前轨迹点（预分配 buffer）
    TrajPhase state{TrajPhase::STOP};       ///< 当前轨迹阶段
    Trajectory traj;                        ///< 轨迹对象
    rclcpp::Time traj_start_time_;          ///< 轨迹开始时间
    Eigen::VectorXd torque;                 ///< 关节力矩
    std::vector<float> default_kp_;         ///< 默认位置增益
    std::vector<float> default_kd_;         ///< 默认速度增益
    std::vector<float> hold_position_;      ///< 锁存的保持位置（进入锁实测、轨迹结束锁最后一帧命令，O19）
    std::vector<double> default_kp_param_;  ///< 位置增益参数原始值
    std::vector<double> default_kd_param_;  ///< 速度增益参数原始值
    std::size_t joint_count_{0};            ///< 关节数量
    FSMArmControlFactory* factory{nullptr}; ///< 状态机工厂指针

    /// 保持路径的重力前馈（docs/bugs.md O7）。等待轨迹到达期间用它补掉重力静差。
    HoldGravityFeedforward gravity_;

    rclcpp::Subscription<robot_msgs::msg::JointTrajCmd>::SharedPtr joint_trajectory_cmd_sub_; ///< 关节轨迹指令订阅

    /**
     * @brief 多路点轨迹指令回调（非实时线程）
     *
     * 仅在 @c state == TrajPhase::STOP 时受理：第一个节点取**上一帧关节命令**
     * （不是实测，见 O19 的参考连续性），其余为用户路点，交给 @ref WaypointTrajectory
     * 规划一次。规划成功才切到 MOVING；
     * 失败只置 @c waypoint_plan_failed_，由 run() 统一走失效安全路径。
     */
    void on_waypoint_command(const robot_msgs::msg::WaypointTrajCmd& msg);

    /// 采样已规划的多路点轨迹并写指令；失败/到达终点时返回相应状态
    bool run_waypoint_motion(const rclcpp::Time& time);

    /// 多路点 C3 轨迹（WaypointTrajCmd）。规划只在订阅回调里做一次，run() 只采样。
    WaypointTrajectory waypoint_traj_;              ///< 规划器（构造期预分配）
    bool waypoint_active_{false};                   ///< 多路点轨迹正在执行
    bool waypoint_plan_failed_{false};              ///< 规划失败，等待 run() 统一处置
    Eigen::VectorXd waypoint_max_vel_;              ///< 关节速度上限（构造期解析）
    Eigen::VectorXd waypoint_max_acc_;              ///< 关节加速度上限（构造期解析）
    Eigen::VectorXd waypoint_max_jerk_;             ///< 关节 jerk 上限（构造期解析）
    Eigen::VectorXd waypoint_pos_;                  ///< 采样位置（构造期分配）
    Eigen::VectorXd waypoint_vel_;                  ///< 采样速度（构造期分配）
    Eigen::VectorXd waypoint_acc_;                  ///< 采样加速度（构造期分配）
    Eigen::VectorXd waypoint_jerk_;                 ///< 采样 jerk（构造期分配）
    rclcpp::Subscription<robot_msgs::msg::WaypointTrajCmd>::SharedPtr waypoint_cmd_sub_; ///< 多路点指令订阅
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr waypoint_duration_pub_;         ///< 实际规划总时长
};

/**
 * @brief 伺服控制状态
 *
 * 接收速度指令，通过积分生成期望位置，再通过逆运动学和逆动力学
 * 求解关节指令。支持笛卡尔空间速度伺服和力前馈。
 *
 * @note 订阅话题 "arm_servo"，消息类型 ServoCtrlCmd
 * @warning 速度积分可能导致位置漂移，需定期校正
 *
 * @see AdmittanceState
 */
class ServoState : public FSM {
public:
    /**
     * @brief 构造函数
     * @param name 状态名称
     * @param ctx  上下文指针
     * @throws std::bad_any_cast 当 ctx 无法转换时抛出
     */
    ServoState(const std::string& name, std::any ctx);

    /**
     * @brief 进入状态时调用
     * @param last_state 上一个状态名称
     * @param time       进入时刻
     * @return true 进入成功；false 表示正运动学求解失败
     */
    bool enter(const std::string& last_state, const rclcpp::Time& time) override;

    /**
     * @brief 离开状态时调用
     * @param next_state 下一个状态名称
     * @return 始终返回 true
     */
    bool exit(const std::string& next_state) override;

    /**
     * @brief 检查是否需要切换状态
     * @return 收到新目标时返回目标状态名
     */
    const std::string& check_switch() const override;

    /**
     * @brief 状态运行时周期性调用
     * @param time 当前时间
     * @return true 运行正常；false 表示求解失败
     * @throws 内部捕获 Pinocchio 异常
     */
    bool run(const rclcpp::Time& time) override;

private:
    FSMArmControlFactory* factory{nullptr};                                             ///< 状态机工厂指针
    std::size_t joint_count_{0};                                                        ///< 关节数量
    std::size_t task_dof_{6};                                                           ///< 任务空间自由度
    rclcpp::Time last_update_time_;                                                     ///< 上次更新时间
    Eigen::VectorXd joint_pos_;                                                         ///< 关节位置
    Eigen::VectorXd joint_velocity_;                                                    ///< 关节速度
    Eigen::VectorXd task_position_;                                                     ///< 任务空间位置
    Eigen::VectorXd desired_task_position_;                                             ///< 期望任务空间位置
    Eigen::VectorXd desired_task_velocity_;                                             ///< 期望任务空间速度
    Eigen::VectorXd desired_task_acceleration_;                                         ///< 期望任务空间加速度
    Eigen::VectorXd task_force_;                                                        ///< 任务空间力
    Eigen::VectorXd torque_;                                                            ///< 关节力矩
    std::vector<float> default_kp_;                                                     ///< 默认位置增益
    std::vector<float> default_kd_;                                                     ///< 默认速度增益
    std::vector<double> default_kp_param_;                                              ///< 位置增益参数原始值
    std::vector<double> default_kd_param_;                                              ///< 速度增益参数原始值

    rclcpp::Subscription<robot_msgs::msg::ServoCtrlCmd>::SharedPtr servo_ctrl_cmd_sub_; ///< 伺服指令订阅
};

/**
 * @brief 导纳控制状态
 *
 * 基于导纳模型实现力控制。通过估计外部力（实测力矩与模型力矩之差），
 * 结合质量-阻尼-刚度模型生成修正轨迹，实现柔顺控制。
 *
 * @note 订阅话题 "arm_admittance"，消息类型 AdmittanceCmd
 * @warning 导纳参数（质量、阻尼、刚度）需根据实际负载调整
 * @see AdmittanceCmd
 *
 * @see ServoState
 */
class AdmittanceState : public FSM {
public:
    /**
     * @brief 构造函数
     * @param name 状态名称
     * @param ctx  上下文指针
     * @throws std::bad_any_cast 当 ctx 无法转换时抛出
     */
    AdmittanceState(const std::string& name, std::any ctx);

    /**
     * @brief 进入状态时调用
     * @param last_state 上一个状态名称
     * @param time       进入时刻
     * @return true 进入成功；false 表示正运动学求解失败
     */
    bool enter(const std::string& last_state, const rclcpp::Time& time) override;

    /**
     * @brief 离开状态时调用
     * @param next_state 下一个状态名称
     * @return 始终返回 true
     */
    bool exit(const std::string& next_state) override;

    /**
     * @brief 检查是否需要切换状态
     * @return 轨迹执行完毕且收到新目标时返回目标状态名
     */
    const std::string& check_switch() const override;

    /**
     * @brief 状态运行时周期性调用
     * @param time 当前时间
     * @return true 运行正常；false 表示求解失败
     * @throws 内部捕获 Pinocchio 异常
     */
    bool run(const rclcpp::Time& time) override;

private:
    /**
     * @brief 加载导纳参数
     * @param name       参数名称
     * @param fallback   默认值
     * @param destination 目标向量
     * @note 若参数不存在或长度不匹配，使用 fallback 值
     */
    void load_admittance_parameters(const char* name, const Eigen::VectorXd& fallback, Eigen::VectorXd& destination);

    FSMArmControlFactory* factory{nullptr};     ///< 状态机工厂指针
    std::size_t joint_count_{0};                ///< 关节数量
    std::size_t task_dof_{6};                   ///< 任务空间自由度
    bool trajectory_active_{false};             ///< 轨迹激活标志
    rclcpp::Time traj_start_time_;              ///< 轨迹开始时间
    rclcpp::Time last_update_time_;             ///< 上次更新时间
    Trajectory traj;                            ///< 轨迹对象
    Point point;                                ///< 当前轨迹点
    Eigen::VectorXd joint_pos_;                 ///< 关节位置
    Eigen::VectorXd joint_velocity_;            ///< 关节速度
    Eigen::VectorXd joint_acceleration_;        ///< 关节加速度
    Eigen::VectorXd model_torque_;              ///< 模型力矩
    Eigen::VectorXd torque_residual_;           ///< 力矩残差（估计外力）
    Eigen::VectorXd task_force_;                ///< 任务空间力
    Eigen::VectorXd task_position_;             ///< 任务空间位置
    Eigen::VectorXd desired_task_position_;     ///< 期望任务空间位置
    Eigen::VectorXd desired_task_velocity_;     ///< 期望任务空间速度
    Eigen::VectorXd desired_task_acceleration_; ///< 期望任务空间加速度
    Eigen::VectorXd position_error_;            ///< 位置误差
    Eigen::VectorXd velocity_error_;            ///< 速度误差
    Eigen::VectorXd torque_;                    ///< 关节力矩
    Eigen::VectorXd admittance_mass_;           ///< 导纳质量参数
    Eigen::VectorXd admittance_damping_;        ///< 导纳阻尼参数
    Eigen::VectorXd admittance_stiffness_;      ///< 导纳刚度参数
    std::vector<float> default_kp_;             ///< 默认位置增益
    std::vector<float> default_kd_;             ///< 默认速度增益
    std::vector<float> hold_position_;          ///< 锁存的保持位置（轨迹未激活时使用）
    std::vector<double> default_kp_param_;      ///< 位置增益参数原始值
    std::vector<double> default_kd_param_;      ///< 速度增益参数原始值

    /// 保持路径的重力前馈（docs/bugs.md O7）。轨迹未激活时用它补掉重力静差。
    HoldGravityFeedforward gravity_;

    rclcpp::Subscription<robot_msgs::msg::AdmittanceCmd>::SharedPtr admittance_cmd_sub_; ///< 导纳指令订阅
};

/**
 * @brief 示教器状态
 *
 * 实现重力补偿的示教功能。用户可外力拖动机械臂，控制器实时计算
 * 重力补偿力矩，使机械臂保持零力平衡状态。
 *
 * @note 只能从空闲状态进入，且只能切换到空闲状态
 * @warning 此状态下位置增益为零，仅速度增益生效，请确保拖动速度适中
 *
 * @see IDELState
 */
class TeachPendantState : public FSM {
public:
    /**
     * @brief 构造函数
     * @param name 状态名称
     * @param ctx  上下文指针
     * @throws std::bad_any_cast 当 ctx 无法转换时抛出
     */
    TeachPendantState(const std::string& name, std::any ctx);

    /**
     * @brief 进入状态时调用
     * @param last_state 上一个状态名称，必须为 "idel"
     * @param time       进入时刻
     * @return true 进入成功；false 表示前置条件不满足
     */
    bool enter(const std::string& last_state, const rclcpp::Time& time) override;

    /**
     * @brief 离开状态时调用
     * @param next_state 下一个状态名称
     * @return 仅当 next_state 为 "idel" 时返回 true
     */
    bool exit(const std::string& next_state) override;

    /**
     * @brief 检查是否需要切换状态
     * @return 收到切换到 "idel" 指令时返回 "idel"
     */
    const std::string& check_switch() const override;

    /**
     * @brief 状态运行时周期性调用
     * @param time 当前时间
     * @return true 运行正常；false 表示求解失败
     * @throws 内部捕获 Pinocchio 异常并记录警告日志
     */
    bool run(const rclcpp::Time& time) override;

private:
    FSMArmControlFactory* factory{nullptr}; ///< 状态机工厂指针
    std::size_t joint_count_{0};            ///< 关节数量
    std::vector<float> default_kd_;         ///< 默认速度增益
    std::vector<double> default_kd_param_;  ///< 速度增益参数原始值
    Eigen::VectorXd joint_pos_;             ///< 关节位置
    Eigen::VectorXd task_zero_;             ///< 零向量（用于逆动力学）
    Eigen::VectorXd gravity_torque_;        ///< 重力补偿力矩
    Eigen::VectorXd end_effector_pose_;     ///< 末端执行器位姿
};

/**
 * @brief 参数辨识状态
 *
 * 通过激励轨迹采集机械臂动力学数据，用于参数辨识。
 * 支持傅里叶激励轨迹生成、数据采集和 CSV 保存。
 *
 * @note 只能从空闲状态进入，且只能切换到空闲状态
 * @warning 辨识过程中机械臂会自动运动，请确保工作空间安全
 * @see ExcitationTrajectory
 * @see ParamterIdentify
 */
class ParamterMeasureState : public FSM {
public:
    /**
     * @brief 测量阶段枚举
     */
    enum class MeasurePhase {
        WaitingForTrajectory, ///< 等待轨迹生成
        MovingToStart,        ///< 移动到起始位置
        ExecutingTrajectory,  ///< 执行激励轨迹
        HoldingEnd,           ///< 保持末端位置
    };

    /**
     * @brief 构造函数
     * @param name 状态名称
     * @param ctx  上下文指针
     * @throws std::bad_any_cast 当 ctx 无法转换时抛出
     */
    ParamterMeasureState(const std::string& name, std::any ctx);

    /**
     * @brief 进入状态时调用
     * @param last_state 上一个状态名称，必须为 "idel"
     * @param time       进入时刻
     * @return true 进入成功；false 表示前置条件不满足
     */
    bool enter(const std::string& last_state, const rclcpp::Time& time) override;

    /**
     * @brief 离开状态时调用
     * @param next_state 下一个状态名称
     * @return 仅当 next_state 为 "idel" 时返回 true
     */
    bool exit(const std::string& next_state) override;

    /**
     * @brief 检查是否需要切换状态
     * @return 测量完成且收到切换到 "idel" 指令时返回 "idel"
     */
    const std::string& check_switch() const override;

    /**
     * @brief 状态运行时周期性调用
     * @param time 当前时间
     * @return true 运行正常；false 表示求解失败或数据异常
     * @note 按阶段执行：等待轨迹生成 -> 移动到起始位置 -> 执行激励轨迹 -> 保持末端位置
     */
    bool run(const rclcpp::Time& time) override;

private:
    FSMArmControlFactory* factory{nullptr};                       ///< 状态机工厂指针
    std::size_t joint_count_{0};                                  ///< 关节数量
    std::shared_ptr<ExcitationTrajectory> excitation_trajectory_; ///< 激励轨迹生成器
    std::unique_ptr<ParamterIdentify> paramter_identify_;         ///<

    std::atomic_bool trajectory_generation_failed_{false};
    Trajectory move_to_start_trajectory_;
    Point move_to_start_point_;
    Point start_point_;                                           // run() 内构造 move_to_start 轨迹起点用的预分配 buffer
    Point end_point_;                                             // run() 内构造 move_to_start 轨迹终点用的预分配 buffer
    std::vector<float> hold_position_;
    std::vector<float> default_kp_;
    std::vector<float> default_kd_;
    std::vector<double> default_kp_param_;
    std::vector<double> default_kd_param_;
    Eigen::VectorXd target_position_;
    Eigen::VectorXd excitation_start_position_;
    Eigen::VectorXd excitation_end_position_;
    Eigen::VectorXd measured_position_;
    Eigen::VectorXd measured_velocity_;
    Eigen::VectorXd measured_acceleration_;                       ///< 真实关节加速度（O8），来自 state interface
    Eigen::VectorXd measured_torque_;
    // O8 时间对齐用：上一拍的位置/速度/时间。MuJoCo 的 qacc 对应**步进前**的状态，
    // 必须延后一拍与它配对，刚体方程才成立（残差 0.25% -> 0.0003%）。
    Eigen::VectorXd prev_position_;
    Eigen::VectorXd prev_velocity_;
    double prev_time_{0.0};
    bool has_prev_sample_{false};
    rclcpp::Time measure_start_time_;
    rclcpp::Time phase_start_time_;
    std::string measure_csv_file_path_{"measured_for_identification.csv"};
    double trajectory_period_{10.0};
    double move_to_start_duration_{3.0};
    double measure_record_sample_rate_{500.0};
    int trajectory_repeat_cnt_{1};
    bool measure_done_{false};
    bool csv_save_requested_{false};
    bool record_full_warned_{false}; ///< 记录缓冲区写满只告警一次
    MeasurePhase measure_phase_{MeasurePhase::HoldingEnd};
};

/**
 * @brief 基座运动补偿 / 末端惯性空间保持状态
 *
 * 语义：机械臂基座在 world 系里按 x/y 平移 + yaw 转向运动，末端（link6）在 world 系
 * 里保持进入状态那一刻的位姿不变。
 *
 * 控制律（运动学补偿）：
 * -# 每拍把 world 系目标左乘 @c T_world_base⁻¹ 变到 base 系，得到 base 系目标；
 * -# 用上一拍解作种子做一次阻尼最小二乘逆解，得到关节位置指令；
 * -# 用基座旋量算期望关节速度 @c J⁻¹·(-v_base - ω_base × p_target) 作为速度前馈；
 * -# 力矩仍走现有的重力前馈（RNEA，@c dq = ddq = 0），刚度取 @c inertial_hold_kp/kd。
 *
 * 坐标与时序：
 * - 基座位姿/速度/加速度来自话题 @c base_motion_topic（默认 @c base_motion_state，
 *   消息 @c robot_msgs/msg/BaseMotionState），订阅在构造期建立、实时路径只做定长拷贝；
 * - 消息按时间戳用 v/a 外推到当前控制周期时刻（外推量受 @c inertial_hold_max_extrapolation
 *   限幅），补偿运输与调度延迟；
 * - 基座只有 x/y/yaw，基座始终水平，所以 base 系下重力方向恒定，现有 FK/Jacobian/RNEA
 *   全部可以直接复用，不需要浮动基座模型。
 *
 * 失效安全（与本仓库其他状态的约定一致：状态只报告，不自己改期望状态）：
 * - 基座话题超过 @c base_state_timeout 未更新：**冻结**关节指令（沿用上一拍位置、
 *   速度前馈置零）并限流 ERROR，不切状态——失去世界系参照时只有冻结不会产生突跳；
 * - 逆解不收敛（基座跑远、目标超出可达集）：同样沿用上一拍指令并限流 WARN；
 * - @c real_mode 下拒绝进入（实机没有基座话题，属于配置错误而不是运行期故障）。
 *
 * @note 进入时快照「当前实测末端位姿」作为 world 目标，全程不变。
 * @warning 实时路径（run）不做参数查询与动态内存分配；话题内容只在锁内做定长拷贝。
 */
class InertialHoldState : public FSM {
public:
    /**
     * @brief 构造函数：读取参数、分配 buffer、建立基座话题订阅与状态发布器
     * @param name 状态名称（"inertial_hold"）
     * @param ctx  上下文指针，实际类型为 FSMArmControlFactory*
     */
    InertialHoldState(const std::string& name, std::any ctx);

    /// 快照当前实测末端位姿作为 world 目标，并把指令初始化成当前位置保持
    bool enter(const std::string& last_state, const rclcpp::Time& time) override;

    /// 恢复被本状态改写的求解器迭代上限
    bool exit(const std::string& next_state) override;

    /// 允许被任意 exp_state 切走（与 idel 一致）
    const std::string& check_switch() const override;

    /// 每拍：取基座状态 → 外推 → 逆解 → 速度前馈 → 写指令 → 记录误差
    bool run(const rclcpp::Time& time) override;

private:
    /// 基座状态的一帧快照（话题内容在回调用锁写、在 run 中用 try_lock 读）
    struct BaseSample {
        int64_t stamp_ns{0};                                         ///< 消息时间戳（ns，仿真/系统时钟）
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};            ///< world 系平移
        Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()}; ///< base → world
        Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};     ///< world 系线速度
        Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};    ///< world 系角速度
        Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()}; ///< world 系线加速度
        Eigen::Vector3d angular_acceleration{Eigen::Vector3d::Zero()};///< world 系角加速度
        bool motion_active{false};                                    ///< 基座轨迹是否已启动
    };

    /// 发布给外部诊断用的快照（实时线程写、定时器读）
    struct StatusSample {
        Eigen::Isometry3d target{Eigen::Isometry3d::Identity()};
        Eigen::Isometry3d actual{Eigen::Isometry3d::Identity()};
        double position_error{0.0};
        double orientation_error{0.0};
        bool base_state_valid{false};
        bool ik_converged{false};
        int64_t stamp_ns{0};
        bool valid{false};
    };

    /// 基座话题回调（非实时线程）：校验后写入 latest_base_
    void on_base_motion_state(const robot_msgs::msg::BaseMotionState& msg);

    /// 取最新一帧基座状态；没有收到过返回 false（实时线程用 try_lock，拿不到锁沿用上次）
    bool read_latest_base_sample(BaseSample* sample);

    /// 按时间戳用 v/a 外推到 now_ns（外推量限幅），结果写进 out
    void extrapolate_base_sample(const BaseSample& sample, int64_t now_ns, BaseSample* out) const;

    /// 写指令：position = hold_position_，velocity = frozen ? 0 : joint_vel_cmd_，力矩 = 重力前馈
    void write_command(bool frozen);

    /// 记录一帧诊断快照（实时线程 try_lock，拿不到就丢弃本周期）
    void cache_status(const Eigen::Isometry3d& target, const Eigen::Isometry3d& actual, double position_error,
                      double orientation_error, bool base_state_valid, bool ik_converged, int64_t stamp_ns);

    /// 定时器回调（非实时线程）：把 StatusSample 发布成 ~/inertial_hold_status
    void publish_status();

    FSMArmControlFactory* factory{nullptr}; ///< 状态机工厂指针
    std::size_t joint_count_{0};            ///< 关节数量
    bool real_mode_{false};                 ///< 实机模式（本状态不可用）

    std::vector<float> kp_;                 ///< 本状态位置增益
    std::vector<float> kd_;                 ///< 本状态速度增益
    std::vector<double> kp_param_;          ///< 位置增益参数原始值
    std::vector<double> kd_param_;          ///< 速度增益参数原始值
    std::vector<float> hold_position_;      ///< 冻结/保持时下发的关节位置

    Eigen::VectorXd measured_;              ///< 实测关节角（run 内缓存）
    Eigen::VectorXd seed_;                  ///< 逆解种子 = 上一拍解
    Eigen::VectorXd joint_vel_cmd_;         ///< 关节速度前馈
    Eigen::VectorXd task_target_;           ///< base 系 6 维目标
    Eigen::VectorXd task_velocity_;         ///< base 系 6 维期望速度
    Eigen::VectorXd zero_;                  ///< 重力前馈用的零向量
    Eigen::Isometry3d world_target_;        ///< world 系保持目标（进入时快照）

    HoldGravityFeedforward gravity_;        ///< 保持路径共用的重力前馈

    BaseSample latest_base_;                ///< 回调写、run 读的基座状态（受 base_mutex_ 保护）
    BaseSample last_base_;                  ///< run 侧上一次成功读到的基座状态（锁竞争时沿用）
    std::mutex base_mutex_;                 ///< 保护 latest_base_
    std::atomic<bool> has_base_{false};     ///< 是否收到过基座状态（原子：锁竞争时也要能读）
    std::string base_topic_;                ///< 基座话题名（仅用于日志）

    StatusSample status_snapshot_;          ///< 状态快照（实时线程写）
    std::mutex status_mutex_;               ///< 保护 status_snapshot_（实时线程只 try_lock）

    int64_t timeout_ns_{100'000'000};       ///< 基座话题超时（ns），参数 base_state_timeout
    int64_t max_extrapolation_ns_{50'000'000}; ///< 外推上限（ns），参数 inertial_hold_max_extrapolation
    int ik_iterations_{10};                 ///< 单次逆解迭代上限，参数 inertial_hold_ik_iterations
    double velocity_ff_scale_{1.0};         ///< 速度前馈缩放，参数 inertial_hold_velocity_ff（0 = 关闭，用于 A/B）

    rclcpp::Subscription<robot_msgs::msg::BaseMotionState>::SharedPtr base_sub_; ///< 基座状态订阅
    rclcpp::Publisher<robot_msgs::msg::InertialHoldStatus>::SharedPtr status_pub_; ///< 诊断发布器
    rclcpp::TimerBase::SharedPtr status_timer_; ///< 诊断发布定时器（200 Hz，非实时线程）
    robot_msgs::msg::InertialHoldStatus status_msg_; ///< 预分配的发布消息
};
