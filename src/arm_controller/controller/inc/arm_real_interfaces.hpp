#pragma once

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class CDCTrans;

namespace arm_controller {

class ArmRealInterfaces : public hardware_interface::SystemInterface {
public:
    ArmRealInterfaces();
    ~ArmRealInterfaces() override;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
    struct JointStateData {
        double position{0.0};
        double velocity{0.0};
        double effort{0.0};
    };

    struct JointCommandData {
        double position{0.0};
        double velocity{0.0};
        double effort{0.0};
        double kp{0.0};
        double kd{0.0};
        double ki{0.0};
    };

    static constexpr std::size_t kStateInterfaceCount = 3;
    static constexpr std::size_t kCommandInterfaceCount = 6;

    void on_pack_praser(const uint8_t* data, int size);

    // 关节名在 on_init 中从 URDF 的 <ros2_control> 段读取，不再硬编码。
    std::vector<std::string> joint_names_;
    std::vector<JointStateData> states_;
    std::vector<JointStateData> received_states_;
    std::vector<JointCommandData> commands_;
    std::mutex received_states_mutex_;

    std::unique_ptr<CDCTrans> cdc_;
    std::shared_ptr<std::thread> cdc_event_thread_;
    bool event_thread_running{true};
    uint16_t vid_{0x0483};
    uint16_t pid_{0x5740};
};

} // namespace arm_controller
