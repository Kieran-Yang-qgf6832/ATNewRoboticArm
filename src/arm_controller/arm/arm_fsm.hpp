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
    std::size_t task_dof_{6};          ///< 任务空间自由度
    FSMArmControlFactory* factory{nullptr}; ///< 状态机工厂指针

    /// 保持路径的重力前馈（docs/bugs.md O7）。等待轨迹到达期间用它补掉重力静差。
    HoldGravityFeedforward gravity_;

    /// 下发前可达性预检（docs/bugs.md O2）。全部在构造阶段分配，run() 中不再分配内存。
    /// 不做预检时，手臂会先沿轨迹走到工作空间边界、再因逆解失败中止（实测末端走约 0.40 m）。
    ///
    /// 只查**终点**：终点不可达是绝大多数实际情况，而且单次逆解实测最坏约 386 us
    /// （200 次迭代），一个控制周期是 2 ms——查 8 个途经点就可能超周期。
    bool precheck_pending_{false};    ///< 是否有一条新轨迹等待预检
    Eigen::VectorXd precheck_target_; ///< 待检终点（6 维）
    Eigen::VectorXd precheck_seed_;   ///< 预检用的关节角种子（当前实测关节角）
    Eigen::VectorXd precheck_solution_; ///< 预检的解算结果

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
    std::size_t task_dof_{6};                         ///< 任务空间自由度
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

    FSMArmControlFactory* factory{nullptr};           ///< 状态机工厂指针
    std::size_t joint_count_{0};                      ///< 关节数量
    std::size_t task_dof_{6};                         ///< 任务空间自由度
    bool trajectory_active_{false};                   ///< 轨迹激活标志
    rclcpp::Time traj_start_time_;                    ///< 轨迹开始时间
    rclcpp::Time last_update_time_;                   ///< 上次更新时间
    Trajectory traj;                                  ///< 轨迹对象
    Point point;                                      ///< 当前轨迹点
    Eigen::VectorXd joint_pos_;                       ///< 关节位置
    Eigen::VectorXd joint_velocity_;                  ///< 关节速度
    Eigen::VectorXd joint_acceleration_;              ///< 关节加速度
    Eigen::VectorXd model_torque_;                    ///< 模型力矩
    Eigen::VectorXd torque_residual_;                 ///< 力矩残差（估计外力）
    Eigen::VectorXd task_force_;                      ///< 任务空间力
    Eigen::VectorXd task_position_;                   ///< 任务空间位置
    Eigen::VectorXd desired_task_position_;           ///< 期望任务空间位置
    Eigen::VectorXd desired_task_velocity_;           ///< 期望任务空间速度
    Eigen::VectorXd desired_task_acceleration_;       ///< 期望任务空间加速度
    Eigen::VectorXd position_error_;                  ///< 位置误差
    Eigen::VectorXd velocity_error_;                  ///< 速度误差
    Eigen::VectorXd torque_;                          ///< 关节力矩
    Eigen::VectorXd admittance_mass_;                 ///< 导纳质量参数
    Eigen::VectorXd admittance_damping_;              ///< 导纳阻尼参数
    Eigen::VectorXd admittance_stiffness_;            ///< 导纳刚度参数
    std::vector<float> default_kp_;                   ///< 默认位置增益
    std::vector<float> default_kd_;                   ///< 默认速度增益
    std::vector<float> hold_position_;                ///< 锁存的保持位置（轨迹未激活时使用）
    std::vector<double> default_kp_param_;            ///< 位置增益参数原始值
    std::vector<double> default_kd_param_;            ///< 速度增益参数原始值

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

