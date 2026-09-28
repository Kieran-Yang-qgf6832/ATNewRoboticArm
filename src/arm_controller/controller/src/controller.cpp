#include "../inc/controller.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <controller_interface/controller_interface.hpp>
#include <memory>
#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <string>
#include <vector>

namespace arm_controller {

ArmController::ArmController() {
}

controller_interface::CallbackReturn ArmController::on_init() {
    auto node = get_node();

    auto_declare<std::vector<double>>("default_kp", {});
    auto_declare<std::vector<double>>("default_kd", {});
    auto_declare<std::vector<double>>("reset_joint_pos", {});
    auto_declare<double>("reset_duration", 3.0);
    auto_declare<double>("reset_tolerance", 0.01);
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

    node->get_parameter<std::vector<std::string>>("joints", joints_name);
    node->get_parameter<std::string>("command_interface_prefix", command_interface_prefix_);

    std::string urdf_path;
    node->get_parameter<std::string>("urdf_path", urdf_path);
    if (urdf_path.empty()) {
        try {
            urdf_path = ament_index_cpp::get_package_share_directory("arm_model") + "/model/robotic_arm.urdf";
            node->set_parameter(rclcpp::Parameter("urdf_path", urdf_path));
        } catch (const std::exception& error) {
            RCLCPP_ERROR(node->get_logger(), "Failed to locate default arm_model URDF: %s", error.what());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    try {
        fsm_factory = std::make_shared<FSMArmControlFactory>(node);
    } catch (const std::exception& error) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize arm controller FSM: %s", error.what());
        return controller_interface::CallbackReturn::ERROR;
    }

    param_cb_ = node->add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto& param : params) {
            auto name = param.get_name();
            if (name == "exp_state") {
                fsm_factory->exp_state_name = param.as_string();
            } else if (name == "joints" || name == "command_interface_prefix") {
                result.successful = false;
                result.reason     = name + " cannot be changed after initialization";
                return result;
            } else if (name == "default_kp" || name == "default_kd" || name == "reset_joint_pos") {
                if (param.as_double_array().size() > joints_name.size()) {
                    result.successful = false;
                    result.reason     = name + " size must be less than or equal to joints size";
                    return result;
                }
            } else if (
                name == "admittance_mass" || name == "admittance_damping" || name == "admittance_stiffness") {
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
                if (param.as_double() <= 0.0) {
                    result.successful = false;
                    result.reason     = "reset_duration must be positive";
                    return result;
                }
            } else if (name == "reset_tolerance") {
                if (param.as_double() < 0.0) {
                    result.successful = false;
                    result.reason     = "reset_tolerance must be non-negative";
                    return result;
                }
            } else if (name == "measure_trajectory_period") {
                if (param.as_double() <= 0.0) {
                    result.successful = false;
                    result.reason     = "measure_trajectory_period must be positive";
                    return result;
                }
            } else if (name == "measure_move_to_start_duration") {
                if (param.as_double() <= 0.0) {
                    result.successful = false;
                    result.reason     = "measure_move_to_start_duration must be positive";
                    return result;
                }
            } else if (name == "measure_trajectory_repeat_cnt") {
                if (param.as_int() <= 0) {
                    result.successful = false;
                    result.reason     = "measure_trajectory_repeat_cnt must be positive";
                    return result;
                }
            } else if (name == "measure_record_sample_rate") {
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
                if (std::isnan(param.as_double())) {
                    result.successful = false;
                    result.reason     = name + " must not be NaN";
                    return result;
                }
            } else {
            }
        }
        return result;
    });

    // 加载默认kp和kd参数
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

    std::string exp_state;
    node->get_parameter("exp_state", exp_state);

    fsm_factory->exp_state_name = exp_state;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmController::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    fsm_factory->state_.resize(joints_name.size());
    fsm_factory->command_.resize(joints_name.size());
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmController::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    if (state_interfaces_.size() == joints_name.size() * 3 && fsm_factory->command_.size() == joints_name.size()) {
        for (std::size_t i = 0; i < joints_name.size(); ++i) {
            fsm_factory->command_[i].position = static_cast<float>(state_interfaces_[i * 3 + 0].get_value());
            fsm_factory->command_[i].velocity = 0.0f;
            fsm_factory->command_[i].torque   = 0.0f;
            fsm_factory->command_[i].kp       = default_kp[i];
            fsm_factory->command_[i].kd       = default_kd[i];
            fsm_factory->command_[i].ki       = 0.0f;
        }
    }
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ArmController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return controller_interface::ControllerInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ArmController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)period;

    for (std::size_t i = 0; i < joints_name.size(); i++) {     // 更新状态机工场的状态值
        fsm_factory->state_[i].position = static_cast<float>(state_interfaces_[i * 3 + 0].get_value());
        fsm_factory->state_[i].velocity = static_cast<float>(state_interfaces_[i * 3 + 1].get_value());
        fsm_factory->state_[i].torque   = static_cast<float>(state_interfaces_[i * 3 + 2].get_value());
    }

    bool ret = fsm_factory->run(time);
    if (!ret) {
        for (std::size_t i = 0; i < joints_name.size(); i++) { // 安全保护
            command_interfaces_[i * 6 + 0].set_value(state_interfaces_[i * 3 + 0].get_value());
            command_interfaces_[i * 6 + 1].set_value(0.0);
            command_interfaces_[i * 6 + 2].set_value(0.0);
            command_interfaces_[i * 6 + 3].set_value(static_cast<double>(default_kp[i]));
            command_interfaces_[i * 6 + 4].set_value(static_cast<double>(default_kd[i]));
            command_interfaces_[i * 6 + 5].set_value(0.0);
        }
        return controller_interface::return_type::ERROR;
    }

    for (std::size_t i = 0; i < joints_name.size(); i++) {     // 更新状态机工场的状态值
        command_interfaces_[i * 6 + 0].set_value(static_cast<double>(fsm_factory->command_[i].position));
        command_interfaces_[i * 6 + 1].set_value(static_cast<double>(fsm_factory->command_[i].velocity));
        command_interfaces_[i * 6 + 2].set_value(static_cast<double>(fsm_factory->command_[i].torque));
        command_interfaces_[i * 6 + 3].set_value(static_cast<double>(fsm_factory->command_[i].kp));
        command_interfaces_[i * 6 + 4].set_value(static_cast<double>(fsm_factory->command_[i].kd));
        command_interfaces_[i * 6 + 5].set_value(static_cast<double>(fsm_factory->command_[i].ki));
    }

    return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration ArmController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joints_name) {
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

PLUGINLIB_EXPORT_CLASS(arm_controller::ArmController, controller_interface::ControllerInterface)
