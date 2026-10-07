/**
 * @defgroup arm_controller_control 机械臂控制器
 * @brief 机械臂 ros2_control 控制器与硬件接口插件的文档分组。
 *
 * 该分组汇集 @c arm_controller 功能包中与 ros2_control 集成相关的实现，
 * 包括上层控制器 @ref arm_controller::ArmController、链式 PID 控制器
 * @c arm_controller::SimPidController 以及实机硬件接口 @c arm_controller::ArmRealInterfaces。
 */

/**
 * @file controller.cpp
 * @brief 机械臂 ros2_control 控制器 @ref arm_controller::ArmController 的实现
 *
 * 本文件实现 @ref arm_controller::ArmController 的生命周期回调、实时更新循环，
 * 以及命令接口与状态接口的声明逻辑。控制器在初始化阶段读取关节列表与阻抗参数，
 * 并构造状态机工厂 @ref FSMArmControlFactory；在实时回调 @ref ArmController::update()
 * 中把硬件反馈送入状态机，再把状态机计算得到的位置/速度/力矩/kp/kd/ki 写入命令接口。
 *
 * 控制器通过 pluginlib 以 @c arm_controller/ArmController 的名称导出，
 * 在 controller_manager 中按如下方式加载：
 * @code{.yaml}
 * controller_manager:
 *   ros__parameters:
 *     arm_controller:
 *       type: arm_controller/ArmController
 * @endcode
 *
 * @author lyz
 * @date 2026-09-29
 * @version 0.0.0
 *
 * @ingroup arm_controller_control
 * @see ArmController
 * @see controller.hpp
 * @see FSMArmControlFactory
 * @see arm_fsm.hpp
 */

#include "../inc/controller.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <controller_interface/controller_interface.hpp>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <std_msgs/msg/string.hpp>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <string>
#include <vector>

/**
 * @namespace arm_controller
 * @brief 机械臂控制核心命名空间
 *
 * 汇聚 ros2_control 控制器插件、仿真 PID 控制器、实机硬件接口以及机械臂 FSM 工厂等实现。
 * 本文件定义的 @ref arm_controller::ArmController 作为上层控制器，
 * 负责把硬件状态送入状态机并下发关节命令。
 */
namespace arm_controller {

/**
 * @brief 构造函数
 *
 * 仅执行默认初始化，不在构造阶段读取 ROS 参数或申请系统资源。
 * 关节列表、机器人模型与状态机工厂统一下沉到生命周期回调
 * @ref ArmController::on_init() 中完成，以符合 ros2_control 的参数声明与错误处理规范。
 *
 * @note 基类 @c controller_interface::ControllerInterface 会被隐式默认构造。
 * @see ArmController::on_init
 */
ArmController::ArmController() {
}




 
/**
 * @brief 生命周期初始化回调，在 configure 之前由控制器管理器调用一次
 *
 * 完成控制器运行所需的全部非实时初始化工作，顺序如下：
 * -# 声明并注册所有 ROS 参数及其默认值，供 controller YAML/命令行覆盖；
 * -# 读取 @c joints 与 @c command_interface_prefix 这两个初始化后不可修改的参数；
 * -# 解析 URDF 路径：若 @c urdf_path 为空，则回退到 @c arm_model 功能包的默认模型并回写参数；
 * -# 构造状态机工厂 @ref FSMArmControlFactory，加载模型并注册全部控制状态；
 * -# 注册参数在线更新回调 @ref param_cb_，对可热更新参数做合法性校验；
 * -# 读取并缓存默认阻抗增益 @c default_kp / @c default_kd（不足部分补 0）；
 * -# 读取 @c exp_state 并写入状态机工厂的期望目标状态；
 * -# 创建状态可观测性出口：话题 @c ~/state 与参数 @c current_state，
 *    由 500 ms wall timer（非实时线程）发布状态机**实际**所处的状态。
 *
 * @return 初始化结果：
 * @retval SUCCESS 初始化成功，控制器可继续进入 configure；
 * @retval ERROR   URDF 定位失败或状态机工厂构造失败。
 *
 * @note 本函数内部捕获 @c get_package_share_directory 与 @ref FSMArmControlFactory
 *       构造抛出的 @c std::exception，记录错误日志后返回 @c ERROR，不向调用方传播异常。
 * @note 参数回调中若发现非法值会拒绝该次更新（@c successful=false），但不会回滚
 *       同一批次中已通过校验的其他参数。
 * @warning @c joints 与 @c command_interface_prefix 初始化完成后禁止修改，
 *          否则会与已建立的接口配置不一致，因此回调中直接拒绝其变更。
 * @see ArmController::on_configure
 * @see FSMArmControlFactory
 * @see ArmController::update
 */
controller_interface::CallbackReturn ArmController::on_init() {
    auto node = get_node();

    // 注册全部可配置参数：以默认值作为兜底，可被 controller YAML 覆盖。
    auto_declare<std::vector<double>>("default_kp", {});
    auto_declare<std::vector<double>>("default_kd", {});
    auto_declare<std::vector<double>>("reset_joint_pos", {});
    auto_declare<double>("reset_duration", 3.0);
    auto_declare<double>("reset_tolerance", 0.01);
    auto_declare<double>("reset_settle", 2.0);
    auto_declare<double>("measure_trajectory_period", 10.0);
    auto_declare<double>("measure_move_to_start_duration", 3.0);
    auto_declare<int>("measure_trajectory_repeat_cnt", 1);
    auto_declare<std::string>("measure_csv_file_path", "/tmp/measured_for_identification.csv");
    auto_declare<double>("measure_record_sample_rate", 500.0);
    auto_declare<std::vector<double>>("admittance_mass", {1.0, 1.0, 1.0, 0.1, 0.1, 0.1});
    auto_declare<std::vector<double>>("admittance_damping", {20.0, 20.0, 20.0, 2.0, 2.0, 2.0});
    auto_declare<std::vector<double>>("admittance_stiffness", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    auto_declare<double>("measure_end_effector_x_lower_limit", -std::numeric_limits<double>::infinity());
    auto_declare<double>("measure_end_effector_x_upper_limit", std::numeric_limits<double>::infinity());
    auto_declare<double>("measure_end_effector_y_lower_limit", -std::numeric_limits<double>::infinity());
    auto_declare<double>("measure_end_effector_y_upper_limit", std::numeric_limits<double>::infinity());
    auto_declare<double>("measure_end_effector_z_lower_limit", -std::numeric_limits<double>::infinity());
    auto_declare<double>("measure_end_effector_z_upper_limit", std::numeric_limits<double>::infinity());
    auto_declare<std::vector<std::string>>("joints", {});
    auto_declare<std::string>("urdf_path", "");
    auto_declare<std::string>("exp_state", "idel");
    auto_declare<std::string>("command_interface_prefix", "");
    auto_declare<std::string>("current_state", "unknown");

    // 读取初始化后不可修改的接口参数：关节名列表与命令接口命名前缀。
    node->get_parameter<std::vector<std::string>>("joints", joints_name);
    node->get_parameter<std::string>("command_interface_prefix", command_interface_prefix_);

    // 解析 URDF 路径：为空时回退到 arm_model 功能包内置模型，并回写参数便于后续复用。
    std::string urdf_path;
    node->get_parameter<std::string>("urdf_path", urdf_path);
    if (urdf_path.empty()) {
        try {
            urdf_path = ament_index_cpp::get_package_share_directory("arm_model") + "/model/robotic_arm.urdf";
            node->set_parameter(rclcpp::Parameter("urdf_path", urdf_path));
        } catch (const std::exception& error) {
            // 无法定位默认模型时初始化失败，避免后续以空路径构造机器人模型。
            RCLCPP_ERROR(node->get_logger(), "Failed to locate default arm_model URDF: %s", error.what());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    // 构造状态机工厂：内部加载 URDF 模型、构建求解器并注册全部控制状态。
    try {
        fsm_factory = std::make_shared<FSMArmControlFactory>(node);
    } catch (const std::exception& error) {
        // 模型加载或状态注册失败会导致实时路径不可用，直接判定初始化失败。
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize arm controller FSM: %s", error.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    // 状态可观测性：~/state 话题 + current_state 参数，发布状态机「实际」所处的状态。
    // 用 wall timer 而非在 update() 里发布，避免在实时线程做 ROS 通信与参数写入。
    state_pub_ = node->create_publisher<std_msgs::msg::String>("~/state", rclcpp::QoS(1).transient_local());
    state_timer_ = node->create_wall_timer(std::chrono::milliseconds(500), [this]() { publish_current_state(); });

    /**
     * @brief 参数在线更新校验回调
     *
     * 遍历本次请求更新的参数并逐项校验，只有全部通过才允许生效：
     * - @c exp_state：直接写入状态机工厂的期望状态名，不做范围校验；
     * - @c joints / @c command_interface_prefix：初始化后不可修改，一律拒绝；
     * - @c default_kp / @c default_kd / @c reset_joint_pos：长度不得超过关节数，
     *   以避免状态切换时重新分配内存；
     * - @c admittance_mass / @c admittance_damping / @c admittance_stiffness：
     *   维度不超过 6，质量必须为正、阻尼与刚度必须非负，且所有元素必须为有限值；
     * - 各时长/采样率参数：必须为正；@c reset_tolerance / @c reset_settle 必须非负；
     *   重复次数必须为正整数；
     * - 末端位置上下限：不允许为 NaN（允许 ±inf 表示不约束）。
     *
     * @param params 本次请求更新的参数列表。
     * @return rcl_interfaces::msg::SetParametersResult 校验结果；
     *         当存在非法值时 @c successful 置为 @c false 并通过 @c reason 说明原因。
     *
     * @note 本回调只做基本合法性检查，不会修改除 @c exp_state 以外的运行态数据；
     *       一旦返回失败，整批参数更新都会被参数服务拒绝。
     * @warning 回调运行于参数服务线程，不应执行耗时或阻塞操作。
     * @see ArmController::on_init
     */
    param_cb_ = node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        // 逐项校验，任一参数不合法即整体拒绝。
        for (const auto& param : params) {
            auto name = param.get_name();
            if (name == "exp_state") {
                // 期望状态名允许热更新，直接写入状态机工厂，由各状态的 check_switch() 消费。
                fsm_factory->exp_state_name = param.as_string();
            } else if (name == "joints" || name == "command_interface_prefix") {
                // 接口相关参数在 on_init() 后固定，禁止再修改。
                result.successful = false;
                result.reason     = name + " cannot be changed after initialization";
                return result;
            } else if (name == "default_kp" || name == "default_kd" || name == "reset_joint_pos") {
                // 数组长度不得超过关节数，避免运行时重新分配内存。
                if (param.as_double_array().size() > joints_name.size()) {
                    result.successful = false;
                    result.reason     = name + " size must be less than or equal to joints size";
                    return result;
                }
            } else if (
                name == "admittance_mass" || name == "admittance_damping" || name == "admittance_stiffness") {
                // 导纳参数最多 6 维；质量必须为正，阻尼/刚度必须非负，且不能为 NaN/Inf。
                const auto values = param.as_double_array();
                if (values.size() > 6) {
                    result.successful = false;
                    result.reason     = name + " size must be less than or equal to 6";
                    return result;
                }
                for (const double value : values) {
                    if (!std::isfinite(value) || (name == "admittance_mass" && value <= 0.0)
                        || (name != "admittance_mass" && value < 0.0)) {
                        result.successful = false;
                        result.reason     = name + " contains an invalid value";
                        return result;
                    }
                }
            } else if (name == "reset_duration") {
                // 复位持续时间必须为正。
                if (param.as_double() <= 0.0) {
                    result.successful = false;
                    result.reason     = "reset_duration must be positive";
                    return result;
                }
            } else if (name == "reset_tolerance") {
                // 复位到位容差允许为 0，但不能为负。
                if (param.as_double() < 0.0) {
                    result.successful = false;
                    result.reason     = "reset_tolerance must be non-negative";
                    return result;
                }
            } else if (name == "reset_settle") {
                // 复位宽限时长允许为 0（到位即退），但不能为负。
                if (param.as_double() < 0.0) {
                    result.successful = false;
                    result.reason     = "reset_settle must be non-negative";
                    return result;
                }
            } else if (name == "measure_trajectory_period") {
                // 辨识轨迹周期必须为正。
                if (param.as_double() <= 0.0) {
                    result.successful = false;
                    result.reason     = "measure_trajectory_period must be positive";
                    return result;
                }
            } else if (name == "measure_move_to_start_duration") {
                // 移动到辨识起点的时间必须为正。
                if (param.as_double() <= 0.0) {
                    result.successful = false;
                    result.reason     = "measure_move_to_start_duration must be positive";
                    return result;
                }
            } else if (name == "measure_trajectory_repeat_cnt") {
                // 辨识轨迹重复次数必须为正整数。
                if (param.as_int() <= 0) {
                    result.successful = false;
                    result.reason     = "measure_trajectory_repeat_cnt must be positive";
                    return result;
                }
            } else if (name == "measure_record_sample_rate") {
                // 记录采样率必须为正。
                if (param.as_double() <= 0.0) {
                    result.successful = false;
                    result.reason     = "measure_record_sample_rate must be positive";
                    return result;
                }
            } else if (name == "measure_end_effector_x_lower_limit"
                       || name == "measure_end_effector_x_upper_limit"
                       || name == "measure_end_effector_y_lower_limit"
                       || name == "measure_end_effector_y_upper_limit"
                       || name == "measure_end_effector_z_lower_limit"
                       || name == "measure_end_effector_z_upper_limit") {
                // 末端位置上下限允许 ±inf（表示不约束），但不允许 NaN。
                if (std::isnan(param.as_double())) {
                    result.successful = false;
                    result.reason     = name + " must not be NaN";
                    return result;
                }
            } else {
                // 其余参数不在此处校验。
            }
        }
        return result;
    });

    // 加载默认阻抗增益并按关节数对齐：先全部置 0，再仅覆盖参数中实际提供的分量，
    // 参数缺失或长度不足的关节保持 0 增益。
    std::vector<double> default_kp_param;
    std::vector<double> default_kd_param;
    node->get_parameter("default_kp", default_kp_param);
    node->get_parameter("default_kd", default_kd_param);
    default_kp.assign(joints_name.size(), 0.0f);
    default_kd.assign(joints_name.size(), 0.0f);
    for (std::size_t i = 0; i < std::min(default_kp.size(), default_kp_param.size()); ++i) {
        default_kp[i] = static_cast<float>(default_kp_param[i]);
    }
    for (std::size_t i = 0; i < std::min(default_kd.size(), default_kd_param.size()); ++i) {
        default_kd[i] = static_cast<float>(default_kd_param[i]);
    }

    // 读取初始期望状态并写入状态机工厂，作为状态机启动后的目标状态。
    std::string exp_state;
    node->get_parameter("exp_state", exp_state);

    fsm_factory->exp_state_name = exp_state;
    return controller_interface::CallbackReturn::SUCCESS;
}

/**
 * @brief 发布状态机当前实际所处的状态
 *
 * 仅在状态名变化时发布一次，负载为零；状态未变化时不做任何 ROS 操作。
 * 话题消息为纯状态名（如 @c "idel" / @c "reset"），参数 @c current_state 同值。
 *
 * @note 运行于 wall timer 回调（非实时线程）。@c set_parameter() 是本地同步调用，
 *       不会等待参数服务往返，因此可以安全地在本回调中调用；
 *       但 @c update() 仍禁止任何 ROS 通信与参数写入。
 * @see ArmController::on_init
 */
void ArmController::publish_current_state() {
    if (!fsm_factory) {
        return;
    }
    const std::string& state = fsm_factory->current_state_name();
    if (state.empty() || state == published_state_name_) {
        return;
    }
    published_state_name_ = state;

    if (state_pub_) {
        std_msgs::msg::String message;
        message.data = state;
        state_pub_->publish(message);
    }
    // current_state 是纯输出参数，参数回调对它不做任何处理，这里写入不会影响 FSM。
    if (get_node()) {
        get_node()->set_parameter(rclcpp::Parameter("current_state", state));
    }
}




/**
 * @brief 生命周期配置回调
 *
 * 依据已确定的关节数量预分配状态机工厂的状态缓存与命令缓存，
 * 确保后续实时更新路径 @ref ArmController::update() 中不再发生动态内存分配。
 *
 * @param previous_state 控制器进入本状态前的生命周期状态；本实现未使用。
 * @return 配置结果，始终返回 @c SUCCESS。
 *
 * @throws std::length_error 当请求的关节数超过容器上限时（正常场景不会发生）。
 * @throws std::bad_alloc    当预分配内存失败时。
 * @note 这是最后一个允许分配内存的阶段之一，@c resize 之后缓冲容量固定不再增长。
 * @warning 必须在 @ref ArmController::on_init() 成功执行之后调用，
 *          否则 @c joints_name 为空且 @c fsm_factory 尚未创建。
 * @see ArmController::on_activate
 * @see ArmController::update
 */
controller_interface::CallbackReturn ArmController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    // 按关节数为状态与命令缓存一次性预分配，保证实时路径零分配。
    fsm_factory->state_.resize(joints_name.size());
    fsm_factory->command_.resize(joints_name.size());
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief 生命周期激活回调
 *
 * 在控制器开始周期执行前，用当前硬件的关节位置初始化状态机命令缓存，
 * 使首个控制周期的期望位置等于当前实际位置，避免上电瞬间产生位置跳变；
 * 同时把速度、力矩与积分项清零，并使用缓存的默认 kp/kd。
 *
 * @param previous_state 控制器进入本状态前的生命周期状态；本实现未使用。
 * @return 激活结果，始终返回 @c SUCCESS。
 *
 * @note 仅当状态接口数量等于 @c joints_name.size()*3 且命令缓存长度与关节数一致时
 *       才执行初始化；条件不满足时静默跳过，由后续接口检查兜底。
 * @warning 若接口数量与关节数不匹配，命令缓存不会被初始化，可能沿用上一次的期望值。
 * @see ArmController::on_configure
 * @see ArmController::update
 */
controller_interface::CallbackReturn ArmController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    // 每个关节恰好暴露 position/velocity/effort 三个状态接口。
    if (state_interfaces_.size() == joints_name.size() * 3 && fsm_factory->command_.size() == joints_name.size()) {
        for (std::size_t i = 0; i < joints_name.size(); ++i) {
            fsm_factory->command_[i].position = static_cast<float>(state_interfaces_[i * 3 + 0].get_value());
            fsm_factory->command_[i].velocity = 0.0f; // 上电初始速度指令清零
            fsm_factory->command_[i].torque   = 0.0f; // 上电初始力矩/前馈清零
            fsm_factory->command_[i].kp       = default_kp[i];
            fsm_factory->command_[i].kd       = default_kd[i];
            fsm_factory->command_[i].ki       = 0.0f; // 不启用积分项
        }
    }
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief 生命周期停用回调
 *
 * 控制器被停用时调用。本实现不释放资源（缓存留待下次 configure 复用），
 * 也不强制输出保持命令，交由 controller_manager 完成。
 *
 * @param previous_state 控制器进入本状态前的生命周期状态；本实现未使用。
 * @return 始终返回 @c SUCCESS。
 * @note 停用后 @ref ArmController::update() 将不再被周期调用。
 */
controller_interface::CallbackReturn ArmController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

/**
 * @brief 实时周期更新回调（核心控制环）
 *
 * 由 controller_manager 以固定周期（项目默认 500 Hz）调用，须保持实时安全：
 * 仅进行接口读写、标量运算与状态机调度，禁止读取 ROS 参数、禁止动态内存分配。
 * 处理流程：
 * -# 从 @c state_interfaces_ 读取每个关节的位置/速度/力矩，写入状态机工厂的状态缓存；
 * -# 调用 @ref FSMArmControlFactory::run() 推进状态机并取得运行结果；
 * -# 若运行失败（如轨迹求解失败、限位越界），向命令接口写入保持当前位置的安全命令，
 *    并返回 @c ERROR；
 * -# 否则将状态机计算出的位置/速度/力矩/kp/kd/ki 逐项写入命令接口，返回 @c OK。
 *
 * @param time   当前 ROS 时间戳，传给状态机用于轨迹/计时等时间相关逻辑。
 * @param period 距上一次更新的周期时长；本实现未使用（状态机内部自行管理时间）。
 * @return 控制器执行结果：
 * @retval OK    本周期正常，命令已成功下发；
 * @retval ERROR 状态机运行失败，已下发安全保持命令。
 *
 * @note 状态接口布局为每关节 3 个（position/velocity/effort），
 *       命令接口布局为每关节 6 个（position/velocity/effort/kp/kd/ki）。
 * @warning 本函数运行于实时控制线程，除错误日志外不得进行阻塞或分配内存的操作。
 * @see ArmController::on_activate
 * @see FSMArmControlFactory::run
 */
controller_interface::return_type ArmController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)period;

    // 读取硬件反馈，刷新状态机工厂的关节状态缓存（每关节 3 个状态接口）。
    for (std::size_t i = 0; i < joints_name.size(); i++) {
        fsm_factory->state_[i].position = static_cast<float>(state_interfaces_[i * 3 + 0].get_value());
        fsm_factory->state_[i].velocity = static_cast<float>(state_interfaces_[i * 3 + 1].get_value());
        fsm_factory->state_[i].torque   = static_cast<float>(state_interfaces_[i * 3 + 2].get_value());
    }

    // 推进状态机：内部完成状态切换、轨迹求解与控制量计算。
    bool ret = fsm_factory->run(time);
    if (!ret) {
        // 状态机失败：进入安全保护——位置锁定为当前实际位置，速度/力矩/积分项清零，
        // 仅保留默认 kp/kd，避免失控运动。
        for (std::size_t i = 0; i < joints_name.size(); i++) {
            command_interfaces_[i * 6 + 0].set_value(state_interfaces_[i * 3 + 0].get_value());
            command_interfaces_[i * 6 + 1].set_value(0.0);
            command_interfaces_[i * 6 + 2].set_value(0.0);
            command_interfaces_[i * 6 + 3].set_value(static_cast<double>(default_kp[i]));
            command_interfaces_[i * 6 + 4].set_value(static_cast<double>(default_kd[i]));
            command_interfaces_[i * 6 + 5].set_value(0.0);
        }
        return controller_interface::return_type::ERROR;
    }

    // 状态机运行正常：下发每关节 6 个命令，顺序为 position/velocity/effort/kp/kd/ki。
    for (std::size_t i = 0; i < joints_name.size(); i++) {
        command_interfaces_[i * 6 + 0].set_value(static_cast<double>(fsm_factory->command_[i].position));
        command_interfaces_[i * 6 + 1].set_value(static_cast<double>(fsm_factory->command_[i].velocity));
        command_interfaces_[i * 6 + 2].set_value(static_cast<double>(fsm_factory->command_[i].torque));
        command_interfaces_[i * 6 + 3].set_value(static_cast<double>(fsm_factory->command_[i].kp));
        command_interfaces_[i * 6 + 4].set_value(static_cast<double>(fsm_factory->command_[i].kd));
        command_interfaces_[i * 6 + 5].set_value(static_cast<double>(fsm_factory->command_[i].ki));
    }

    return controller_interface::return_type::OK;
}

/**
 * @brief 声明控制器所需的命令接口
 *
 * 为每个关节声明 6 个 @c INDIVIDUAL 类型的命令接口，命名格式为
 * @c <prefix>/<joint>/<interface>，其中 @c prefix 取自 @c command_interface_prefix
 * （为空时省略）；接口名依次为 @c position、@c velocity、@c effort、@c kp、@c kd、@c ki。
 *
 * @return 命令接口配置，类型为 @c interface_configuration_type::INDIVIDUAL。
 *
 * @note 该声明在控制器加载时用于接口匹配，依赖 @ref ArmController::on_init() 中读取到的
 *       @c joints_name 与 @c command_interface_prefix_。
 * @warning 若 @c command_interface_prefix_ 在运行时被修改，将导致与实际硬件接口不匹配。
 * @see ArmController::state_interface_configuration
 * @see ArmController::update
 */
controller_interface::InterfaceConfiguration ArmController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joints_name) {
        // 前缀为空时直接使用关节名，否则拼接为 <prefix>/<joint>。
        const auto command_name = command_interface_prefix_.empty() ? name : command_interface_prefix_ + "/" + name;
        cfg.names.push_back(command_name + "/position");
        cfg.names.push_back(command_name + "/velocity");
        cfg.names.push_back(command_name + "/effort");
        cfg.names.push_back(command_name + "/kp");
        cfg.names.push_back(command_name + "/kd");
        cfg.names.push_back(command_name + "/ki");
    }
    return cfg;
}

/**
 * @brief 声明控制器所需的硬件状态接口
 *
 * 为每个关节声明 3 个 @c INDIVIDUAL 类型的状态接口，命名格式为
 * @c <joint>/<interface>，接口名依次为 @c position、@c velocity、@c effort。
 *
 * @return 状态接口配置，类型为 @c interface_configuration_type::INDIVIDUAL。
 *
 * @note 状态接口不使用 @c command_interface_prefix_ 前缀，直接以关节名命名。
 * @see ArmController::command_interface_configuration
 * @see ArmController::on_activate
 * @see ArmController::update
 */
controller_interface::InterfaceConfiguration ArmController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joints_name) {
        cfg.names.push_back(name + "/position");
        cfg.names.push_back(name + "/velocity");
        cfg.names.push_back(name + "/effort");
    }
    return cfg;
}

} // namespace arm_controller

/**
 * @brief 通过 pluginlib 导出控制器插件
 *
 * 将 @ref arm_controller::ArmController 注册为
 * @c controller_interface::ControllerInterface 的实现，
 * 使 controller_manager 能够以插件名 @c arm_controller/ArmController 动态加载本控制器。
 *
 * @note 插件名与其基类在 @c arm_ctrl_plugin.xml 中声明，需与本宏导出的类保持一致。
 * @see arm_controller::ArmController
 * @see arm_ctrl_plugin.xml
 */
PLUGINLIB_EXPORT_CLASS(arm_controller::ArmController, controller_interface::ControllerInterface)
