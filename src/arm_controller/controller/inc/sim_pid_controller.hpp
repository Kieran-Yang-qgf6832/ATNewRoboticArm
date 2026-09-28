#pragma once

#include <controller_interface/chainable_controller_interface.hpp>
#include <controller_interface/controller_interface_base.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <string>
#include <vector>

namespace arm_controller {

class SimPidController : public controller_interface::ChainableControllerInterface {
public:
    SimPidController();

    controller_interface::CallbackReturn on_init() override;
    controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update_and_write_commands(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;

protected:
    controller_interface::return_type update_reference_from_subscribers(
        const rclcpp::Time& time,
        const rclcpp::Duration& period) override;
    std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;

private:
    static constexpr std::size_t kReferenceInterfaceCount = 6;
    static constexpr std::size_t kStateInterfaceCount = 3;

    std::vector<std::string> joints_name_;
    std::vector<double> integral_error_;
};

} // namespace arm_controller
