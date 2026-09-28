#include "../inc/sim_pid_controller.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace arm_controller {

SimPidController::SimPidController() = default;

controller_interface::CallbackReturn SimPidController::on_init() {
    auto_declare<std::vector<std::string>>("joints", {});
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimPidController::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    auto node = get_node();
    node->get_parameter<std::vector<std::string>>("joints", joints_name_);

    reference_interfaces_.assign(joints_name_.size() * kReferenceInterfaceCount, 0.0);
    integral_error_.assign(joints_name_.size(), 0.0);

    RCLCPP_INFO(
        node->get_logger(),
        "Configured SimPidController for %zu joints.",
        joints_name_.size());
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimPidController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SimPidController::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    for (auto& command_interface : command_interfaces_) {
        command_interface.set_value(0.0);
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SimPidController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joints_name_) {
        cfg.names.push_back(name + "/" + "effort");
    }
    return cfg;
}

controller_interface::InterfaceConfiguration SimPidController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& name : joints_name_) {
        cfg.names.push_back(name + "/" + "position");
        cfg.names.push_back(name + "/" + "velocity");
        cfg.names.push_back(name + "/" + "effort");
    }
    return cfg;
}

controller_interface::return_type SimPidController::update_and_write_commands(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {
    (void)time;

    if (command_interfaces_.size() != joints_name_.size()
        || state_interfaces_.size() != joints_name_.size() * kStateInterfaceCount
        || reference_interfaces_.size() != joints_name_.size() * kReferenceInterfaceCount) {
        return controller_interface::return_type::ERROR;
    }

    const double dt = period.seconds();
    for (std::size_t i = 0; i < joints_name_.size(); ++i) {
        const std::size_t ref_base = i * kReferenceInterfaceCount;
        const std::size_t state_base = i * kStateInterfaceCount;

        const double position_ref = reference_interfaces_[ref_base + 0];
        const double velocity_ref = reference_interfaces_[ref_base + 1];
        const double effort_ref = reference_interfaces_[ref_base + 2];
        const double kp = reference_interfaces_[ref_base + 3];
        const double kd = reference_interfaces_[ref_base + 4];
        const double ki = reference_interfaces_[ref_base + 5];

        const double position = state_interfaces_[state_base + 0].get_value();
        const double velocity = state_interfaces_[state_base + 1].get_value();

        const double position_error = position_ref - position;
        const double velocity_error = velocity_ref - velocity;

        if (dt > 0.0 && std::isfinite(dt)) {
            integral_error_[i] += position_error * dt;
        }

        const double effort_command =
            effort_ref + kp * position_error + kd * velocity_error + ki * integral_error_[i];
        command_interfaces_[i].set_value(effort_command);
    }

    return controller_interface::return_type::OK;
}

controller_interface::return_type SimPidController::update_reference_from_subscribers(
    const rclcpp::Time& time,
    const rclcpp::Duration& period) {
    (void)time;
    (void)period;
    return controller_interface::return_type::OK;
}

std::vector<hardware_interface::CommandInterface> SimPidController::on_export_reference_interfaces() {
    std::vector<hardware_interface::CommandInterface> reference_interfaces;
    reference_interfaces.reserve(joints_name_.size() * kReferenceInterfaceCount);

    for (std::size_t i = 0; i < joints_name_.size(); ++i) {
        const std::size_t base = i * kReferenceInterfaceCount;
        const auto prefix = std::string(get_node()->get_name());

        reference_interfaces.emplace_back(prefix, joints_name_[i] + "/position", &reference_interfaces_[base + 0]);
        reference_interfaces.emplace_back(prefix, joints_name_[i] + "/velocity", &reference_interfaces_[base + 1]);
        reference_interfaces.emplace_back(prefix, joints_name_[i] + "/effort", &reference_interfaces_[base + 2]);
        reference_interfaces.emplace_back(prefix, joints_name_[i] + "/kp", &reference_interfaces_[base + 3]);
        reference_interfaces.emplace_back(prefix, joints_name_[i] + "/kd", &reference_interfaces_[base + 4]);
        reference_interfaces.emplace_back(prefix, joints_name_[i] + "/ki", &reference_interfaces_[base + 5]);
    }

    return reference_interfaces;
}

} // namespace arm_controller

PLUGINLIB_EXPORT_CLASS(arm_controller::SimPidController, controller_interface::ChainableControllerInterface)
