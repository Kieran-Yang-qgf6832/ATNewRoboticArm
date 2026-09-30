/**
 * @file arm_fsm_factory.hpp
 * @brief 机械臂有限状态机工厂类声明
 *
 * 负责创建并注册所有机械臂控制状态，持有机器人模型、任务空间映射、
 * 运动学/动力学求解器以及当前状态和指令 buffer。
 * 是连接 ROS 2 控制器接口与 FSM 状态调度层的桥梁。
 *
 * @author lyz
 * @date 2026-09-28
 * @version 0.0.0
 *
 * @see FSMFactory
 * @see FSM
 * @see ModelFromURDF
 * @see Default6DofTaskSpaceMapping
 * @see ArmSolve
 * @see arm_fsm.hpp
 */

#pragma once

#include "arm.hpp"
#include "arm_fsm.hpp"
#include "default6dof_task.hpp"
#include "fsm_factory.hpp"
#include "model_from_urdf.hpp"
#include "modelbase.hpp"
#include "task.hpp"

#include <memory>
#include <rclcpp/qos.hpp>
#include <robot_msgs/msg/detail/joint_traj_cmd__struct.hpp>
#include <string>
#include <vector>

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <robot_msgs/msg/motor_cmd.hpp>
#include <robot_msgs/msg/motor_state.hpp>

/**
 * @brief 机械臂 FSM 工厂类
 *
 * 继承自 FSMFactory，在构造时完成以下初始化：
 * - 加载 URDF 机器人模型（ModelFromURDF）
 * - 创建 6 自由度任务空间映射（Default6DofTaskSpaceMapping）
 * - 构建运动学/动力学求解器（ArmSolve）
 * - 注册全部 8 个控制状态
 * - 设置初始状态为 "idel"
 *
 * @note 所有状态共享此工厂持有的模型、求解器和 buffer
 * @warning 构造时需确保 ROS 2 节点已加载所有必要参数
 *
 * @see FSMFactory
 * @see IDELState
 * @see ResetState
 * @see CartTrajState
 * @see JointTrajState
 * @see ServoState
 * @see AdmittanceState
 * @see TeachPendantState
 * @see ParamterMeasureState
 */
class FSMArmControlFactory : public FSMFactory {
public:
    /**
     * @brief 构造函数
     * @param node ROS 2 生命周期节点共享指针，用于参数加载和话题订阅
     * @note 构造时自动加载 URDF 模型并注册所有状态
     * @throws std::runtime_error 当 URDF 文件加载失败时可能抛出
     */
    FSMArmControlFactory(rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
        node_      = node;
        model_     = std::make_shared<ModelFromURDF>(node->get_parameter("urdf_path").as_string(), "link6");
        task_map_  = std::make_shared<Default6DofTaskSpaceMapping>();
        arm_solve_ = std::make_shared<ArmSolve>(model_, task_map_);         // 加载机器人模型


        register_fsm(std::make_unique<IDELState>("idel", this));                  // 机械臂锁定在当前位置
        register_fsm(std::make_unique<ResetState>("reset", this));                // 机械臂复位
        register_fsm(std::make_unique<CartTrajState>("cart_traj", this));         // 执行笛卡尔轨迹
        register_fsm(std::make_unique<JointTrajState>("joint_traj", this));       // 执行关节空间轨迹
        register_fsm(std::make_unique<ServoState>("servo", this));                // 伺服动作，接收速度指令，将指令积分作为期望位置
        register_fsm(std::make_unique<AdmittanceState>("admittance", this));      // 导纳控制
        register_fsm(std::make_unique<TeachPendantState>("teach_pendant", this)); // 示教器，可外力拖动，可配置带阻尼，重力补偿
        register_fsm(std::make_unique<ParamterMeasureState>("measure", this));    // 系统参数辨识

        set_init_state("idel");
    }

    /**
     * @brief 期望切换到的目标状态名称
     * @note 由外部指令设置，各状态的 check_switch() 读取此值决定是否切换
     * @see FSM::check_switch()
     */
    std::string exp_state_name{"idel"};

    /**
     * @brief 运动学/动力学求解器共享指针
     * @note 由模型对象和任务空间映射对象构造
     * @see ArmSolve
     */
    std::shared_ptr<ArmSolve> arm_solve_;

    /**
     * @brief 机器人模型共享指针
     * @note 基于 Pinocchio 的 URDF 模型，提供正逆运动学和逆动力学
     * @see ModelFromURDF
     */
    std::shared_ptr<ModelBase> model_;

    /**
     * @brief 任务空间映射共享指针
     * @note 6 自由度笛卡尔空间与关节空间的映射
     * @see Default6DofTaskSpaceMapping
     */
    std::shared_ptr<TaskMapping> task_map_;

    /**
     * @brief 当前关节状态数组
     * @note 由硬件接口或仿真器更新，包含位置、速度、力矩等信息
     * @see robot_msgs::msg::MotorState
     */
    std::vector<robot_msgs::msg::MotorState> state_;

    /**
     * @brief 当前关节指令数组
     * @note 由各状态的 run() 方法写入，最终下发到硬件或仿真器
     * @see robot_msgs::msg::MotorCmd
     */
    std::vector<robot_msgs::msg::MotorCmd> command_;

    /**
     * @brief ROS 2 生命周期节点共享指针
     * @note 用于参数加载、话题订阅和日志输出
     */
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

private:
};
