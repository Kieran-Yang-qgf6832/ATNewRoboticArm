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
#include <array>
#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>

#include "excitation_trajectory.hpp"
#include "paramter_identify.hpp"
#include "trajectory.hpp"

#include <robot_msgs/msg/admittance_cmd.hpp>
#include <robot_msgs/msg/cart_traj_cmd.hpp>
#include <robot_msgs/msg/joint_traj_cmd.hpp>
#include <robot_msgs/msg/servo_ctrl_cmd.hpp>

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
    bool enabled() const { return enabled_; }

private:
    bool enabled_{true};        ///< 是否启用，来自参数 gravity_compensation
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
    FSMArmControlFactory* factory{nullptr};      ///< 状态机工厂指针
    std::size_t joint_count_{0};                 ///< 关节数量
    std::vector<float> start_joint_pos_;         ///< 起始关节位置
    std::vector<float> reset_joint_pos_;         ///< 目标复位位置
    std::vector<float> default_kp_;              ///< 默认位置增益
    std::vector<float> default_kd_;              ///< 默认速度增益
    std::vector<double> reset_joint_pos_param_;  ///< 复位位置参数原始值
    std::vector<double> default_kp_param_;       ///< 位置增益参数原始值
    std::vector<double> default_kd_param_;       ///< 速度增益参数原始值
    rclcpp::Time reset_start_time_;              ///< 复位开始时间
    float reset_duration_{3.0f};                 ///< 复位时长（秒）
    float reset_tolerance_{0.01f};               ///< 到位容差（弧度）
    float reset_settle_{2.0f};                   ///< 复位时长结束后的宽限时长（秒）
    float progress_{0.0f};                       ///< 复位进度 [0, 1]
    bool reset_done_{false};                     ///< 复位完成标志
    bool give_up_warned_{false};                 ///< 是否已就「超时仍未到位」告警（只告警一次）

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
    Point point;                ///< 当前轨迹点（预分配 buffer）
    TrajPhase state{TrajPhase::STOP}; ///< 当前轨迹阶段
    Trajectory traj;            ///< 轨迹对象
    rclcpp::Time traj_start_time_;    ///< 轨迹开始时间
    Eigen::VectorXd joint_pos_;       ///< 关节位置
    Eigen::VectorXd torque_;          ///< 关节力矩
    Eigen::VectorXd task_force_;      ///< 任务空间力
    std::vector<float> default_kp_;    ///< 默认位置增益
    std::vector<float> default_kd_;    ///< 默认速度增益
    std::vector<float> hold_position_; ///< 锁存的保持位置（进入或轨迹结束时锁存一次）
    std::vector<double> default_kp_param_; ///< 位置增益参数原始值
    std::vector<double> default_kd_param_; ///< 速度增益参数原始值
    std::size_t joint_count_{0};       ///< 关节数量
    std::size_t task_dof_{0};          ///< 任务空间自由度（取自工厂参数 task_space_dof）
    FSMArmControlFactory* factory{nullptr}; ///< 状态机工厂指针

    /// 保持路径的重力前馈（docs/bugs.md O7）。等待轨迹到达期间用它补掉重力静差。
    HoldGravityFeedforward gravity_;

    /// 下发前可达性预检（docs/bugs.md O2）。全部在构造阶段分配，run() 中不再分配内存。
    /// 不做预检时，手臂会先沿轨迹走到工作空间边界、再因逆解失败中止（实测末端走约 0.40 m）。
    ///
    /// 只查**终点**：终点不可达是绝大多数实际情况，而且单次逆解实测最坏约 386 us
    /// （200 次迭代），一个控制周期是 2 ms——查 8 个途经点就可能超周期。
    bool precheck_pending_{false};    ///< 是否有一条新轨迹等待预检
    Eigen::VectorXd precheck_target_; ///< 待检终点（task_dof_ 维，已从消息的 6 维位姿投影）
    Eigen::VectorXd precheck_seed_;   ///< 预检用的关节角种子（当前实测关节角）
    Eigen::VectorXd precheck_solution_; ///< 预检的解算结果

    /// 逆解失败日志用的预分配 buffer（实时路径禁止分配内存）。
    Eigen::VectorXd debug_seed_;          ///< 当前实测关节角
    Eigen::VectorXd debug_measured_task_; ///< 实测关节角的正解（任务空间）
    Eigen::VectorXd debug_task_error_;    ///< 目标与实测正解的任务空间误差

    /// 订阅回调（非实时线程）里做 6 维位姿 -> 任务目标投影用的 buffer。
    Eigen::VectorXd load_seed_;       ///< 投影时取 joint4 自转目标的关节角种子
    Eigen::VectorXd load_task_point_; ///< 单点投影结果

    /**
     * @brief 对 @ref precheck_target_ 做一次逆解，确认终点可达
     * @param[out] residual 不可达时写入「最近可达位姿到终点还有多远」，否则写 0
     * @return 终点可达返回 true
     */
    bool check_target_reachable(double* residual);

    rclcpp::Subscription<robot_msgs::msg::CartTrajCmd>::SharedPtr cart_traj_cmd_sub_; ///< 笛卡尔轨迹指令订阅
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
    Point point;                ///< 当前轨迹点（预分配 buffer）
    TrajPhase state{TrajPhase::STOP}; ///< 当前轨迹阶段
    Trajectory traj;            ///< 轨迹对象
    rclcpp::Time traj_start_time_;    ///< 轨迹开始时间
    Eigen::VectorXd torque;            ///< 关节力矩
    std::vector<float> default_kp_;    ///< 默认位置增益
    std::vector<float> default_kd_;    ///< 默认速度增益
    std::vector<float> hold_position_; ///< 锁存的保持位置（进入或轨迹结束时锁存一次）
    std::vector<double> default_kp_param_; ///< 位置增益参数原始值
    std::vector<double> default_kd_param_; ///< 速度增益参数原始值
    std::size_t joint_count_{0};       ///< 关节数量
    FSMArmControlFactory* factory{nullptr}; ///< 状态机工厂指针

    /// 保持路径的重力前馈（docs/bugs.md O7）。等待轨迹到达期间用它补掉重力静差。
    HoldGravityFeedforward gravity_;

    rclcpp::Subscription<robot_msgs::msg::JointTrajCmd>::SharedPtr joint_trajectory_cmd_sub_; ///< 关节轨迹指令订阅
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
    FSMArmControlFactory* factory{nullptr};           ///< 状态机工厂指针
    std::size_t joint_count_{0};                      ///< 关节数量
    std::size_t task_dof_{0};                         ///< 任务空间自由度（本状态只在 6 维下可用）
    rclcpp::Time last_update_time_;                   ///< 上次更新时间
    Eigen::VectorXd joint_pos_;                       ///< 关节位置
    Eigen::VectorXd joint_velocity_;                  ///< 关节速度
    Eigen::VectorXd task_position_;                   ///< 任务空间位置
    Eigen::VectorXd desired_task_position_;           ///< 期望任务空间位置
    Eigen::VectorXd desired_task_velocity_;           ///< 期望任务空间速度
    Eigen::VectorXd desired_task_acceleration_;       ///< 期望任务空间加速度
    Eigen::VectorXd task_force_;                      ///< 任务空间力
    Eigen::VectorXd torque_;                          ///< 关节力矩
    std::vector<float> default_kp_;                   ///< 默认位置增益
    std::vector<float> default_kd_;                   ///< 默认速度增益
    std::vector<double> default_kp_param_;            ///< 位置增益参数原始值
    std::vector<double> default_kd_param_;            ///< 速度增益参数原始值

    rclcpp::Subscription<robot_msgs::msg::ServoCtrlCmd>::SharedPtr servo_ctrl_cmd_sub_; ///< 伺服指令订阅
};

/**
 * @brief 导纳控制状态
 *
 * 两种用法共用同一套力估计、死区、滤波与限速：
 *
 * - **自由导纳**（默认：进来但还没收到 \c arm_admittance 轨迹时）：进入时把期望任务
 *   位置锁到当前位姿，之后每周期按 c M·ẍ + D·ẋ = F_est 积分（没有固定参考点，K 不参与）。
 *   不推不动、推着走、撤力停在原地；速度受 \c admittance_velocity_limit（平移模长）与
 *   \c admittance_rotation_velocity_limit（自转）限制。逆解失败（顶到关节限位 / 工作空间
 *   边界）时回滚到上一拍可行目标并清零速度，**状态保持而不是报错回 idel**。
 * - **轨迹 + 柔顺**（收到 \c arm_admittance 轨迹后）：沿用原语义，导纳在轨迹点附近叠加
 *   柔顺位移；柔顺速度同样受限，但轨迹本身的前馈速度不截断。
 *
 * 力估计：\c τ_est = τ_actuator(实测) − RNEA(q, dq, ddq_实测)。O8 之后控制器能读到真实
 * 加速度（\c state_acceleration_），用它做 RNEA 可以把 M·qdd 从残差里去掉；没有该接口
 * （实机）时退回 ddq = 0。残差再经任务空间雅可比转成力，然后过死区与一阶低通。
 * **死区是必需的**：K = 0 时导纳对力是无穷大直流增益，任何模型偏差（实测参数误差
 * 0.1% 量级）都会被积成一个缓慢爬行。
 *
 * @note 订阅话题 "arm_admittance"，消息类型 AdmittanceCmd；不发轨迹就是自由导纳
 * @warning 导纳参数（M/D/K、限速、死区、滤波）都可在运行时用 \c ros2 param set 热更新
 * @warning 实时约束：buffer 全部在构造阶段分配；\c run() 只读原子参数快照，不分配、不加锁
 * @see AdmittanceCmd
 * @see TeachPendantState
 * @see docs/bugs.md O13 / O14
 */
class AdmittanceState : public FSM {
public:
    /**
     * @brief 参数数组的最大长度（与控制器参数校验里的上限一致）
     */
    static constexpr std::size_t kMaxTaskDof = 6;

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
     * @return true 进入成功；false 表示工厂/求解器未就绪或正运动学求解失败
     * @note 4 轴模型下不再拒绝进入（docs/bugs.md O13 的临时限制已解除）
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

    /**
     * @brief 热更新导纳 M/D/K（由参数回调在非实时线程调用）
     * @param mass      质量参数，空表示不改；长度超过任务维的部分忽略
     * @param damping   阻尼参数，同上
     * @param stiffness 刚度参数，同上
     * @note 只写原子快照，\c run() 下一拍生效；不做内存分配
     */
    void update_impedance_parameters(const std::vector<double>& mass, const std::vector<double>& damping,
                                     const std::vector<double>& stiffness);

    /**
     * @brief 热更新限速 / 死区 / 滤波截止频率（由参数回调在非实时线程调用）
     * @param velocity_limit           平移速度上限（米/秒）
     * @param rotation_velocity_limit  自转速度上限（弧度/秒）
     * @param force_deadband           平移力死区（牛）
     * @param torque_deadband          自转力矩死区（牛·米）
     * @param filter_cutoff            一阶低通截止频率（赫兹），0 = 关闭
     */
    void update_runtime_parameters(double velocity_limit, double rotation_velocity_limit, double force_deadband,
                                   double torque_deadband, double filter_cutoff, double force_limit,
                                   double torque_limit);

    /// 当前生效的平移速度上限（米/秒）
    double velocity_limit() const { return velocity_limit_.load(std::memory_order_relaxed); }
    /// 当前生效的自转速度上限（弧度/秒）
    double rotation_velocity_limit() const { return rotation_velocity_limit_.load(std::memory_order_relaxed); }
    /// 当前生效的平移力死区（牛）
    double force_deadband() const { return force_deadband_.load(std::memory_order_relaxed); }
    /// 当前生效的自转力矩死区（牛·米）
    double torque_deadband() const { return torque_deadband_.load(std::memory_order_relaxed); }
    /// 当前生效的一阶低通截止频率（赫兹，0 = 关闭）
    double filter_cutoff() const { return filter_cutoff_.load(std::memory_order_relaxed); }
    /// 当前生效的平移力上限（牛）
    double force_limit() const { return force_limit_.load(std::memory_order_relaxed); }
    /// 当前生效的自转力矩上限（牛·米）
    double torque_limit() const { return torque_limit_.load(std::memory_order_relaxed); }

    /**
     * @brief 拷贝最近一拍的任务空间力估计（供控制器以非实时频率对外发布）
     * @param raw      输出：死区/滤波之前的力估计，长度需 >= task_dof()
     * @param filtered 输出：死区/滤波之后的力估计，长度需 >= task_dof()
     * @param capacity 两个输出缓冲的长度
     * @return 写入的分量数（= task_dof()）；capacity 不足时返回 0 且不写
     * @note 读的是原子快照，可在任意线程调用；用于 O14 第 1 步「力估计复核」与服务端调参
     */
    std::size_t copy_force_estimate(double* raw, double* filtered, std::size_t capacity) const;

private:
    /**
     * @brief 加载导纳参数
     * @param name       参数名称
     * @param fallback   默认值
     * @param destination 目标向量
     * @note 若参数不存在或长度不匹配，使用 fallback 值
     */
    void load_admittance_parameters(const char* name, const Eigen::VectorXd& fallback, Eigen::VectorXd& destination);

    /**
     * @brief 任务空间速度限幅
     * @param velocity 待限幅的速度（原地修改）；分量 0..2 是平移，其余是自转
     * @note 平移按**向量模长**限，避免斜推时被逐轴截断导致运动方向偏离受力方向
     */
    void limit_task_velocity(Eigen::VectorXd* velocity) const;

    /**
     * @brief 自由导纳：把期望位置积分到 @ref next_task_position_ 并做可达性处理
     * @note 逆解失败时回滚到 @ref last_feasible_task_position_ 并把期望速度清零，
     *       绝不把「顶到边界」升级成故障（否则推到头会瞬间掉回 idel）
     */
    void solve_free_roam_target();

    /**
     * @brief 由期望关节位置/速度 + 逆动力学前馈写 command_
     * @param feedforward_task_force        逆动力学前馈里的任务力；外力由导纳积分体现，
     *                                      这里传零向量，避免 J^T·F 被叠加两次
     * @param feedforward_task_acceleration 逆动力学前馈里的任务加速度。自由导纳传零向量：
     *                                      它的加速度来自力估计，近奇异位形下可能被放大，
     *                                      交给位置环跟随比直接加进力矩前馈安全
     * @return 正常写入了完整前馈返回 true；逆速度/逆动力学局部失败时退回较保守的
     *         命令（速度 0 或纯重力前馈）并返回 false，调用方不得据此判定故障
     */
    bool write_task_command(const Eigen::VectorXd& feedforward_task_force,
                            const Eigen::VectorXd& feedforward_task_acceleration);

    /**
     * @brief 把力估计限幅到物理合理的量级（docs/bugs.md O14）
     *
     * 近奇异位形下 static_force() 的任务空间最小二乘解会被 1/σ_min 放大：实测零位形
     * （完全伸直、z 方向不可控）进入导纳时，四阶残差被放大成 78 N / 12 N·m 的假力，
     * 会把机械臂甩出去。平移按模长限、自转按绝对值限；只作用于进入导纳的那份估计，
     * 对外发布的「原始估计」保持原样，便于诊断。
     */
    void clamp_task_force();

    /**
     * @brief 自由导纳分支（没有 \c arm_admittance 轨迹时）
     * @param time 当前时间
     * @param dt   本拍积分步长（已限幅、非负）
     * @return true 正常；false 表示积分结果非有限值（可恢复故障）
     */
    bool run_free_roam(const rclcpp::Time& time, double dt);

    /**
     * @brief 轨迹 + 柔顺分支（收到 \c arm_admittance 轨迹后，沿用原语义）
     * @param time 当前时间
     * @param dt   本拍积分步长（已限幅、非负）
     * @return true 正常；false 表示采样/逆解/积分失败（可恢复故障）
     */
    bool run_trajectory(const rclcpp::Time& time, double dt);

    FSMArmControlFactory* factory{nullptr};           ///< 状态机工厂指针
    std::size_t joint_count_{0};                      ///< 关节数量
    std::size_t task_dof_{0};                         ///< 任务空间自由度（4 = [x,y,z,joint4]，6 = 位姿）
    bool trajectory_active_{false};                   ///< 轨迹激活标志
    rclcpp::Time traj_start_time_;                    ///< 轨迹开始时间
    rclcpp::Time last_update_time_;                   ///< 上次更新时间
    Trajectory traj;                                  ///< 轨迹对象
    Point point;                                      ///< 当前轨迹点
    Eigen::VectorXd joint_pos_;                       ///< 关节位置（实测）
    Eigen::VectorXd joint_velocity_;                  ///< 关节速度（实测）
    Eigen::VectorXd joint_acceleration_;              ///< 关节加速度（实测，缺失时为 0）
    Eigen::VectorXd model_torque_;                    ///< 模型力矩（RNEA，残差与前馈共用）
    Eigen::VectorXd torque_residual_;                 ///< 外部广义力矩 = RNEA − τ_act（不是 τ_act − RNEA）
    Eigen::VectorXd raw_task_force_;                  ///< 死区/滤波之前的任务空间力
    Eigen::VectorXd task_force_;                      ///< 死区/滤波之后的任务空间力
    Eigen::VectorXd task_position_;                   ///< 任务空间位置（实测正解）
    Eigen::VectorXd desired_task_position_;           ///< 期望任务空间位置
    Eigen::VectorXd next_task_position_;              ///< 本拍候选期望位置（逆解通过才提交）
    Eigen::VectorXd last_feasible_task_position_;     ///< 上一个逆解可行的期望位置
    Eigen::VectorXd desired_task_velocity_;           ///< 期望任务空间速度（已限幅）
    Eigen::VectorXd desired_task_acceleration_;       ///< 期望任务空间加速度
    Eigen::VectorXd compliance_velocity_;             ///< 轨迹模式下只对柔顺部分限速用的缓存
    Eigen::VectorXd position_error_;                  ///< 位置误差
    Eigen::VectorXd velocity_error_;                  ///< 速度误差
    Eigen::VectorXd zero_task_;                       ///< 任务维零向量（逆动力学前馈用）
    Eigen::VectorXd desired_joint_position_;          ///< 期望关节位置（逆解结果）
    Eigen::VectorXd desired_joint_velocity_;          ///< 期望关节速度
    Eigen::VectorXd last_feasible_joint_position_;    ///< 上一个逆解可行的关节位置
    Eigen::VectorXd admittance_mass_;                 ///< 导纳质量参数（本拍快照）
    Eigen::VectorXd admittance_damping_;              ///< 导纳阻尼参数（本拍快照）
    Eigen::VectorXd admittance_stiffness_;            ///< 导纳刚度参数（本拍快照）
    std::vector<float> default_kp_;                   ///< 默认位置增益
    std::vector<float> default_kd_;                   ///< 默认速度增益
    std::vector<float> hold_position_;                ///< 锁存的保持位置（保留给失败兜底）
    std::vector<double> default_kp_param_;            ///< 位置增益参数原始值
    std::vector<double> default_kd_param_;            ///< 速度增益参数原始值
    bool force_filter_initialized_{false};            ///< 低通滤波状态是否已初始化

    /// 参数热更新快照：参数回调线程写、run() 每拍读，单写单读，用原子量避免撕裂。
    std::array<std::atomic<double>, kMaxTaskDof> param_mass_{};
    std::array<std::atomic<double>, kMaxTaskDof> param_damping_{};
    std::array<std::atomic<double>, kMaxTaskDof> param_stiffness_{};
    std::atomic<double> velocity_limit_{0.1};              ///< 平移速度上限（米/秒）
    std::atomic<double> rotation_velocity_limit_{0.5};     ///< 自转速度上限（弧度/秒）
    std::atomic<double> force_deadband_{1.5};              ///< 平移力死区（牛）
    std::atomic<double> torque_deadband_{0.2};             ///< 自转力矩死区（牛·米）
    std::atomic<double> filter_cutoff_{10.0};              ///< 一阶低通截止频率（赫兹），0 = 关闭
    std::atomic<double> force_limit_{20.0};                ///< 平移力估计上限（牛）
    std::atomic<double> torque_limit_{3.0};                ///< 自转力矩估计上限（牛·米）
    /// 最近一拍的力估计快照（原始 / 死区+滤波之后），供控制器 500 ms 定时器对外发布。
    std::array<std::atomic<double>, kMaxTaskDof> raw_force_estimate_{};
    std::array<std::atomic<double>, kMaxTaskDof> filtered_force_estimate_{};

    /// 保持路径的重力前馈（docs/bugs.md O7）。逆动力学前馈求解失败时退回到它。
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
    std::size_t task_dof_{0};               ///< 任务空间维（4 轴模型是 4，不再写死 6，见 O13）
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

    FSMArmControlFactory* factory{nullptr};                  ///< 状态机工厂指针
    std::size_t joint_count_{0};                             ///< 关节数量
    std::shared_ptr<ExcitationTrajectory> excitation_trajectory_; ///< 激励轨迹生成器
    std::unique_ptr<ParamterIdentify> paramter_identify_;    ///< 

    std::atomic_bool trajectory_generation_failed_{false};
    Trajectory move_to_start_trajectory_;
    Point move_to_start_point_;
    Point start_point_;   // run() 内构造 move_to_start 轨迹起点用的预分配 buffer
    Point end_point_;     // run() 内构造 move_to_start 轨迹终点用的预分配 buffer
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
    Eigen::VectorXd measured_acceleration_; ///< 真实关节加速度（O8），来自 state interface
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

