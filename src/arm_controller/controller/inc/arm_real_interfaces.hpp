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
        // MCU 协议不上报加速度；恒 0，仅为满足控制器对 <joint>/acceleration 的请求。
        double acceleration{0.0};
    };

    struct JointCommandData {
        double position{0.0};
        double velocity{0.0};
        double effort{0.0};
        double kp{0.0};
        double kd{0.0};
        double ki{0.0};
    };

    static constexpr std::size_t kStateInterfaceCount = 4;
    static constexpr std::size_t kCommandInterfaceCount = 6;

    /// 固件实际驱动的电机数。MCU 通信帧固定 6 槽，但第 6 槽既不应答也不驱动
    /// （实机末端回转 joint6 无电机、机械锁死）。上位机对它的处理见 read()。
    static constexpr std::size_t kDrivenJointCount = 5;

    /// 接收缓冲：USB FS bulk 上限 64 字节，77 字节的状态帧必然被拆包，
    /// 这里按字节流累加后再切帧（见 on_pack_praser）。
    static constexpr std::size_t kRxBufferSize = 256;

    void on_pack_praser(const uint8_t* data, int size);

    /// 从 joint_calibration.yaml 载入 dir/zero（joint1..joint5）与 joint6 的固定角。
    bool load_calibration(const std::string& path);

    // 关节名在 on_init 中从 URDF 的 <ros2_control> 段读取，不再硬编码。
    std::vector<std::string> joint_names_;
    std::vector<JointStateData> states_;
    // 固件上报的**原始**角度（未做 dir/zero 换算），换算在 read() 里做。
    std::vector<JointStateData> received_states_;
    std::vector<JointCommandData> commands_;
    std::mutex received_states_mutex_;
    // 最近一次收到状态包（或回环刷新）的时刻；read() 用它判断是否掉线。
    std::chrono::steady_clock::time_point last_state_time_{};

    // ---- 关节标定：q_urdf = dir * (q_raw - zero) ----
    // 固件上报的角度零位是它自己上电标定的物理姿态，与 URDF 的 SolidWorks 零位无关；
    // 方向也可能相反（固件 joint[2] 的 inv_motor = -1）。这两组值必须现场标定，
    // 缺省为 dir = 1、zero = 0（等同于直接透传）。
    std::array<double, kDrivenJointCount> calib_dir_{{1.0, 1.0, 1.0, 1.0, 1.0}};
    std::array<double, kDrivenJointCount> calib_zero_{{0.0, 0.0, 0.0, 0.0, 0.0}};
    // joint6 无电机：它的状态位由指令回写（见 read()），此值仅用于日志与上电占位。
    double joint6_fixed_{0.0};

    // 接收字节流缓冲与已填充长度。
    std::array<uint8_t, kRxBufferSize> rx_buffer_{};
    std::size_t rx_len_{0};

    std::unique_ptr<CDCTrans> cdc_;
    std::shared_ptr<std::thread> cdc_event_thread_;
    bool event_thread_running{true};
    // 来自 URDF <param name="loopback">，默认关闭。开启时不打开 USB 设备，
    // 把上一帧指令直接读回状态，仅用于无硬件冒烟。
    bool loopback_{false};
    // 来自 URDF <param name="state_timeout_ms">，默认 100 ms。
    std::chrono::milliseconds state_timeout_{100};
    uint16_t vid_{0x0483};
    // 与下位机固件一致：USB_DEVICE/App/usbd_desc.c 里 USBD_PID_FS = 22337 = 0x5741。
    // ST 官方模板常写 0x5740，本固件不是，按 0x5740 打开会找不到设备。
    uint16_t pid_{0x5741};
};

} // namespace arm_controller
