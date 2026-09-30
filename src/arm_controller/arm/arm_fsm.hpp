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
};

/**
 * @brief 机械臂复位状态
 *
 * 控制机械臂从当前位置平滑移动到预设的复位位置。支持可配置的
 * 复位时长和到位容差。复位完成后自动切换到空闲状态。
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
    float progress_{0.0f};                       ///< 复位进度 [0, 1]
    bool reset_done_{false};                     ///< 复位完成标志
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
    std::vector<double> default_kp_param_; ///< 位置增益参数原始值
    std::vector<double> default_kd_param_; ///< 速度增益参数原始值
    std::size_t joint_count_{0};       ///< 关节数量
    std::size_t task_dof_{6};          ///< 任务空间自由度
    FSMArmControlFactory* factory{nullptr}; ///< 状态机工厂指针

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
    std::vector<double> default_kp_param_; ///< 位置增益参数原始值
    std::vector<double> default_kd_param_; ///< 速度增益参数原始值
    std::size_t joint_count_{0};       ///< 关节数量
    FSMArmControlFactory* factory{nullptr}; ///< 状态机工厂指针

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
    std::vector<double> default_kp_param_;            ///< 位置增益参数原始值
    std::vector<double> default_kd_param_;            ///< 速度增益参数原始值

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
    Eigen::VectorXd measured_torque_;
    rclcpp::Time measure_start_time_;
    rclcpp::Time phase_start_time_;
    std::string measure_csv_file_path_{"/tmp/measured_for_identification.csv"};
    double trajectory_period_{10.0};
    double move_to_start_duration_{3.0};
    double measure_record_sample_rate_{500.0};
    int trajectory_repeat_cnt_{1};
    bool measure_done_{false};
    bool csv_save_requested_{false};
    MeasurePhase measure_phase_{MeasurePhase::HoldingEnd};
};

