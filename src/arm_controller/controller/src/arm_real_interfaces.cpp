#include "../inc/arm_real_interfaces.hpp"

#include "../inc/data_pack.h"
#include "cdc_trans.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <memory>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <exception>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

namespace arm_controller {

namespace {

// MCU 通信帧在 data_pack.h 中固定为 6 个电机。这里从结构体定义反推，避免帧长在两处各写一个数字而失配。
constexpr std::size_t kProtocolMotorCount = sizeof(MCUTarget2Pack::motor) / sizeof(MCUTarget2Pack::motor[0]);

static_assert(
    sizeof(MCUStatePack::motor) / sizeof(MCUStatePack::motor[0]) == kProtocolMotorCount,
    "MCU 收发帧的电机数量不一致，请检查 data_pack.h");

} // namespace

ArmRealInterfaces::ArmRealInterfaces() = default;

ArmRealInterfaces::~ArmRealInterfaces() {
}

hardware_interface::CallbackReturn ArmRealInterfaces::on_init(const hardware_interface::HardwareInfo& hardware_info) {
    if (hardware_interface::SystemInterface::on_init(hardware_info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // 关节名必须以 URDF 为准。此前硬编码为 joint1..joint6，URDF 里改了名字或数量时，
    // 导出的接口名会和 URDF 对不上，而下面的循环又假定一定有 6 个元素。
    joint_names_.clear();
    joint_names_.reserve(hardware_info.joints.size());
    for (const auto& joint : hardware_info.joints) {
        joint_names_.push_back(joint.name);
    }

    // MCU 通信帧长度固定为 kProtocolMotorCount。数量不符时，收发循环会越界读写
    // commands_ / received_states_，把垃圾数据当指令发给电机（docs/bugs.md O3）。
    // 这里直接拒绝启动，而不是靠截断把问题藏起来。
    if (joint_names_.size() != kProtocolMotorCount) {
        RCLCPP_ERROR(
            rclcpp::get_logger("arm_real_interfaces"),
            "URDF 声明了 %zu 个关节，但 MCU 通信帧固定为 %zu 个电机，拒绝启动。",
            joint_names_.size(),
            kProtocolMotorCount);
        return hardware_interface::CallbackReturn::ERROR;
    }

    states_.assign(joint_names_.size(), JointStateData{});
    received_states_.assign(joint_names_.size(), JointStateData{});
    commands_.assign(joint_names_.size(), JointCommandData{});

    RCLCPP_INFO(
        rclcpp::get_logger("arm_real_interfaces"),
        "Configured ArmRealInterfaces for %zu joints.",
        joint_names_.size());

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmRealInterfaces::on_configure(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    cdc_ = std::make_unique<CDCTrans>();
    cdc_->regeiser_recv_cb([this](const uint8_t* data, int size) {
        on_pack_praser(data, size);
    });

    if (!cdc_->open(vid_, pid_)) {
        RCLCPP_ERROR(rclcpp::get_logger("arm_real_interfaces"), "Failed to open USB CDC device vid=0x%04x pid=0x%04x.", vid_, pid_);
        cdc_.reset();
        return hardware_interface::CallbackReturn::ERROR;
    }

    cdc_event_thread_ = std::make_shared<std::thread>([this]() {
        while (event_thread_running) {
            cdc_->process_once();
        }
    });

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmRealInterfaces::on_activate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmRealInterfaces::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmRealInterfaces::on_cleanup(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    if(cdc_)
    {
        event_thread_running=false;
        if(cdc_event_thread_->joinable())
            cdc_event_thread_->join();
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmRealInterfaces::on_shutdown(const rclcpp_lifecycle::State& previous_state) {
    (void)previous_state;
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ArmRealInterfaces::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    state_interfaces.reserve(joint_names_.size() * kStateInterfaceCount);

    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        state_interfaces.emplace_back(joint_names_[i], "position", &states_[i].position);
        state_interfaces.emplace_back(joint_names_[i], "velocity", &states_[i].velocity);
        state_interfaces.emplace_back(joint_names_[i], "effort", &states_[i].effort);
    }

    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ArmRealInterfaces::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    command_interfaces.reserve(joint_names_.size() * kCommandInterfaceCount);

    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
        command_interfaces.emplace_back(joint_names_[i], "position", &commands_[i].position);
        command_interfaces.emplace_back(joint_names_[i], "velocity", &commands_[i].velocity);
        command_interfaces.emplace_back(joint_names_[i], "effort", &commands_[i].effort);
        command_interfaces.emplace_back(joint_names_[i], "kp", &commands_[i].kp);
        command_interfaces.emplace_back(joint_names_[i], "kd", &commands_[i].kd);
        command_interfaces.emplace_back(joint_names_[i], "ki", &commands_[i].ki);
    }

    return command_interfaces;
}

hardware_interface::return_type ArmRealInterfaces::read(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    (void)period;

    std::lock_guard<std::mutex> lock(received_states_mutex_);
    const auto joint_count = std::min(states_.size(), received_states_.size());
    for (std::size_t i = 0; i < joint_count; ++i) {
        states_[i] = received_states_[i];
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type ArmRealInterfaces::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    (void)time;
    (void)period;

    if (!cdc_) {
        return hardware_interface::return_type::ERROR;
    }

    MCUTarget2Pack pack{};
    pack.head = 0x5B;

    const auto joint_count = std::min(commands_.size(), kProtocolMotorCount);
    for (std::size_t i = 0; i < joint_count; ++i) {
        pack.motor[i].position = static_cast<float>(commands_[i].position);
        pack.motor[i].velocity = static_cast<float>(commands_[i].velocity);
        pack.motor[i].torque = static_cast<float>(commands_[i].effort);
        pack.motor[i].kp = static_cast<float>(commands_[i].kp);
        pack.motor[i].kd = static_cast<float>(commands_[i].kd);
        pack.motor[i].ki = static_cast<float>(commands_[i].ki);
    }
    cdc_->send_struct(pack);
    return hardware_interface::return_type::OK;
}

void ArmRealInterfaces::on_pack_praser(const uint8_t* data, int size) {
    if (data == nullptr || size < static_cast<int>(sizeof(MCUStatePack))) {
        return;
    }

    MCUStatePack pack{};
    std::memcpy(&pack, data, sizeof(pack));
    if (pack.head != 0x4A) {
        return;
    }

    std::lock_guard<std::mutex> lock(received_states_mutex_);
    const auto joint_count = std::min(received_states_.size(), kProtocolMotorCount);
    for (std::size_t i = 0; i < joint_count; ++i) {
        received_states_[i].position = pack.motor[i].position;
        received_states_[i].velocity = pack.motor[i].velocity;
        received_states_[i].effort = pack.motor[i].torque;
    }
}

} // namespace arm_controller

PLUGINLIB_EXPORT_CLASS(arm_controller::ArmRealInterfaces, hardware_interface::SystemInterface)
