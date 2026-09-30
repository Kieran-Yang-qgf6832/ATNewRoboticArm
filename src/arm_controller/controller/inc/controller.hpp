/**
 * @file controller.hpp
 * @brief 机械臂 ros2_control 控制器 @ref arm_controller::ArmController 的类声明
 *
 * 声明 @ref arm_controller::ArmController，它是连接 ros2_control 控制器框架与
 * 机械臂有限状态机 @ref FSMArmControlFactory 的上层控制器：由控制器框架驱动其
 * 生命周期回调与实时更新回调，内部再委托状态机完成具体控制逻辑。
 *
 * 本头文件仅包含类声明，各回调的具体实现见 controller.cpp。
 *
 * @author lyz
 * @date 2026-09-29
 * @version 0.0.0
 *
 * @ingroup arm_controller_control
 * @see ArmController
 * @see controller.cpp
 * @see FSMArmControlFactory
 */

#pragma once


#include <controller_interface/controller_interface.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <../arm/arm_fsm_factory.hpp>
#include <../arm/arm_fsm.hpp>
#include <string>
#include <unordered_map>
#include <controller_interface/controller_interface_base.hpp>

/**
 * @namespace arm_controller
 * @brief 机械臂控制核心命名空间
 *
 * 汇聚 ros2_control 控制器插件、仿真 PID 控制器、实机硬件接口以及机械臂 FSM 工厂等实现。
 */
namespace arm_controller {

/**
 * @class ArmController
 * @brief 机械臂 ros2_control 控制器插件
 *
 * 继承自 @c controller_interface::ControllerInterface，作为 controller_manager
 * 统一管理的标准控制器插件运行。其主要职责：
 * -# 在 @ref ArmController::on_init() 中加载关节列表、URDF 与阻抗参数，
 *    并构造状态机工厂 @ref FSMArmControlFactory；
 * -# 在 @ref ArmController::update() 实时回调中读取硬件反馈、驱动状态机并下发关节命令；
 * -# 通过 @ref ArmController::command_interface_configuration() 与
 *    @ref ArmController::state_interface_configuration() 声明所需的命令/状态接口。
 *
 * 控制器以插件名 @c arm_controller/ArmController 通过 pluginlib 导出，
 * 由控制配置 YAML 引用。
 *
 * @note 接口布局：每关节 3 个状态接口（position/velocity/effort），
 *       6 个命令接口（position/velocity/effort/kp/kd/ki）。
 * @warning @ref ArmController::update() 运行于实时控制线程，其实现中禁止读取 ROS 参数、
 *          禁止动态内存分配与阻塞操作。
 * @see controller_interface::ControllerInterface
 * @see FSMArmControlFactory
 * @see controller.cpp
 */
class ArmController : public controller_interface::ControllerInterface {
public:
    /**
     * @brief 构造函数
     *
     * 仅执行默认初始化，不读取 ROS 参数、不申请系统资源。
     *
     * @note 参数加载、URDF 解析与状态机工厂创建均在 @ref ArmController::on_init() 中完成。
     * @see ArmController::on_init
     */
    ArmController();

    /**
     * @brief 生命周期初始化回调
     *
     * 在 configure 之前由 controller_manager 调用一次，完成参数声明与读取、
     * URDF 路径解析、状态机工厂构造以及参数在线更新回调的注册。
     *
     * @return 初始化结果
     * @retval SUCCESS 初始化成功
     * @retval ERROR   URDF 定位失败或状态机工厂构造失败
     *
     * @note 内部对异常进行了捕获并转换为返回码，不向调用方抛出异常。
     * @warning 仅当本回调返回 @c SUCCESS 后，控制器才会继续进入后续生命周期。
     * @see ArmController::on_configure
     */
    controller_interface::CallbackReturn on_init() override;

    /**
     * @brief 生命周期配置回调
     *
     * 依据已确定的关节数量预分配状态机的状态缓存与命令缓存。
     *
     * @param previous_state 进入本生命周期状态前的状态，本实现未使用。
     * @return 配置结果，始终返回 @c SUCCESS
     *
     * @note 该阶段是实时更新路径以外允许分配内存的时机，@c resize 之后缓冲容量固定。
     * @see ArmController::on_activate
     */
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief 生命周期激活回调
     *
     * 控制器开始周期执行前调用，用当前硬件位置初始化命令缓存，避免上电位置跳变。
     *
     * @param previous_state 进入本生命周期状态前的状态，本实现未使用。
     * @return 激活结果，始终返回 @c SUCCESS
     *
     * @note 速度、力矩与积分项被清零，并使用缓存的默认 kp/kd。
     * @see ArmController::update
     */
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief 生命周期停用回调
     *
     * 控制器停止周期执行时调用，本实现不释放资源。
     *
     * @param previous_state 进入本生命周期状态前的状态，本实现未使用。
     * @return 始终返回 @c SUCCESS
     */
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    /**
     * @brief 实时周期更新回调
     *
     * 每个控制周期读取硬件反馈、推进状态机，并下发关节命令。
     *
     * @param time   当前 ROS 时间戳，用于状态机内的时间相关逻辑。
     * @param period 距上一次更新的周期时长，本实现未使用。
     * @return 控制器执行结果
     * @retval OK    本周期正常，命令已下发
     * @retval ERROR 状态机运行失败，已下发安全保持命令
     *
     * @warning 本回调运行于实时控制线程，实现中禁止读取 ROS 参数、动态内存分配与阻塞操作。
     * @see FSMArmControlFactory::run
     */
    controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

    /**
     * @brief 声明控制器所需的命令接口
     *
     * 为每个关节声明位置、速度、力矩与阻抗增益等命令接口。
     *
     * @return @c INDIVIDUAL 类型的命令接口配置
     * @note 每个关节依次声明 position/velocity/effort/kp/kd/ki 六个命令接口。
     * @see ArmController::state_interface_configuration
     */
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    /**
     * @brief 声明控制器所需的硬件状态接口
     *
     * 为每个关节声明位置、速度、力矩状态接口。
     *
     * @return @c INDIVIDUAL 类型的状态接口配置
     * @note 每个关节依次声明 position/velocity/effort 三个状态接口。
     * @see ArmController::command_interface_configuration
     */
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

private:
    /// 参数在线更新回调句柄，持有它以保证回调在控制器存活期间始终有效
    rclcpp_lifecycle::LifecycleNode::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    /// 状态机工厂共享指针，持有机器人模型、求解器并承载各控制状态
    std::shared_ptr<FSMArmControlFactory> fsm_factory;
    /// 关节名称列表，决定命令/状态接口的数量与命名
    std::vector<std::string> joints_name;
    /// 命令接口名称前缀，为空时命令接口直接使用关节名
    std::string command_interface_prefix_;
    /**
     * @brief 各关节默认阻抗增益
     * @note 依次为位置增益 @c default_kp 与速度增益 @c default_kd，
     *       均在 @ref ArmController::on_init() 中按关节数对齐并缓存，
     *       供 @ref ArmController::on_activate() 与安全保护逻辑使用。
     */
    std::vector<float> default_kp,default_kd;
};

}  // namespace arm_controller
