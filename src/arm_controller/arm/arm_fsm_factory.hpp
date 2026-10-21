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
#include "default4dof_task.hpp"
#include "default6dof_task.hpp"
#include "fsm_factory.hpp"
#include "model_from_urdf.hpp"
#include "modelbase.hpp"
#include "task.hpp"

#include <memory>
#include <stdexcept>
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
 * - 按参数 @c task_space_dof 创建任务空间映射（4 维 / 6 维）
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
        model_     = std::make_shared<ModelFromURDF>(
            node->get_parameter("urdf_path").as_string(), node->get_parameter("end_effector_frame").as_string());

        // 任务空间维度（docs/bugs.md O13）：4 轴臂的 6 维几何雅可比秩恒为 4，用 6 维
        // 任务空间时笛卡尔目标必然不可达（实测最小二乘「尽力而为」把位置让出 28.6 mm）。
        // 4 = (x, y, z, joint4 自转)；6 = (x, y, z, 旋转向量)。换回 6 轴模型时改参数即可。
        const int64_t task_space_dof = node->get_parameter("task_space_dof").as_int();
        if (task_space_dof == 4) {
            task_map_ = std::make_shared<Default4DofTaskSpaceMapping>();
        } else if (task_space_dof == 6) {
            task_map_ = std::make_shared<Default6DofTaskSpaceMapping>();
        } else {
            throw std::runtime_error("task_space_dof must be 4 or 6");
        }
        task_dof_ = static_cast<std::size_t>(task_map_->task_dim());

        const int model_dof = model_->dof();
        if (task_space_dof > model_dof) {
            RCLCPP_WARN(
                node->get_logger(),
                "task_space_dof=%lld but the model has only %d joints: the task space is over-constrained, every "
                "Cartesian target will be rejected as unreachable",
                static_cast<long long>(task_space_dof), model_dof);
        }
        if (task_space_dof == 4 && model_dof != 4) {
            RCLCPP_WARN(
                node->get_logger(),
                "task_space_dof=4 assumes the 4th joint is the tool roll axis, but the model has %d joints", model_dof);
        }
        RCLCPP_INFO(
            node->get_logger(), "arm_controller: task_space_dof=%zu (%s), model dof=%d", task_dof_,
            task_space_dof == 4 ? "x, y, z, joint4" : "x, y, z, rx, ry, rz", model_dof);

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
     * @note 由外部指令设置，各状态的 check_switch() 读取此值决定是否切换。
     *       该成员已上移到基类 @ref FSMFactory，以便失败路径能在工厂内直接请求切换，
     *       不再依赖状态自身「清理 + 返回 true」来触发。此处仅作说明，不再重复声明。
     * @see FSMFactory::exp_state_name
     * @see FSM::check_switch()
     */

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
     * @brief 任务空间维度（4 = [x, y, z, joint4]，6 = [x, y, z, 旋转向量]）
     * @note 由控制器参数 @c task_space_dof 决定，初始化后不可修改；各 FSM 状态的
     *       任务维与消息投影都以它为准。
     * @see TaskMapping::task_dim
     */
    std::size_t task_dof_{4};

    /**
     * @brief 当前关节状态数组
     * @note 由硬件接口或仿真器更新，包含位置、速度、力矩等信息
     * @see robot_msgs::msg::MotorState
     */
    std::vector<robot_msgs::msg::MotorState> state_;

    /**
     * @brief 当前关节加速度缓存（只读）
     * @note 由 ArmController::update() 从 state interface 的 acceleration 读到；
     *       MuJoCo 仿真里是真实 qacc，实机若不上报则恒为 0。
     *       参数辨识用它替代"对记录速度二次求导"（docs/bugs.md O8）。
     * @see ArmController::update
     * @see ParamterMeasureState
     */
    std::vector<double> state_acceleration_;

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
