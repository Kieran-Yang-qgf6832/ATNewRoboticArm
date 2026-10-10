// 离线校验 ArmRealInterfaces 的关节数处理（docs/bugs.md O3）。
//
// 只走 on_init 与接口导出路径，不连接硬件、不打开 USB：
//   1. 关节数与 MCU 通信帧不符时，on_init 必须拒绝启动，而不是越界读写；
//   2. 关节数相符时，导出的接口名必须来自 URDF，而不是硬编码的 joint1..joint6。
//
// 不安装，仅供排查使用。

#include "../controller/inc/arm_real_interfaces.hpp"

#include <hardware_interface/component_parser.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstddef>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

/// 生成一个关节数为 joint_count、关节名带 prefix 的合法 URDF，并挂上 ArmRealInterfaces 插件。
std::string make_urdf(std::size_t joint_count, const std::string& prefix) {
    std::ostringstream urdf;
    urdf << "<robot name=\"test_arm\">\n";
    urdf << "  <link name=\"base_link\"/>\n";
    for (std::size_t i = 1; i <= joint_count; ++i) {
        urdf << "  <link name=\"link" << i << "\"/>\n";
    }

    for (std::size_t i = 1; i <= joint_count; ++i) {
        const std::string parent = (i == 1) ? "base_link" : ("link" + std::to_string(i - 1));
        urdf << "  <joint name=\"" << prefix << i << "\" type=\"revolute\">\n";
        urdf << "    <parent link=\"" << parent << "\"/>\n";
        urdf << "    <child link=\"link" << i << "\"/>\n";
        urdf << "    <axis xyz=\"0 0 1\"/>\n";
        urdf << "    <limit lower=\"-3.0\" upper=\"3.0\" effort=\"10.0\" velocity=\"10.0\"/>\n";
        urdf << "  </joint>\n";
    }

    urdf << "  <ros2_control name=\"arm_hw\" type=\"system\">\n";
    urdf << "    <hardware>\n";
    urdf << "      <plugin>arm_controller/ArmRealInterfaces</plugin>\n";
    urdf << "    </hardware>\n";
    for (std::size_t i = 1; i <= joint_count; ++i) {
        urdf << "    <joint name=\"" << prefix << i << "\">\n";
        urdf << "      <command_interface name=\"position\"/>\n";
        urdf << "      <command_interface name=\"velocity\"/>\n";
        urdf << "      <command_interface name=\"effort\"/>\n";
        urdf << "      <command_interface name=\"kp\"/>\n";
        urdf << "      <command_interface name=\"kd\"/>\n";
        urdf << "      <command_interface name=\"ki\"/>\n";
        urdf << "      <state_interface name=\"position\"/>\n";
        urdf << "      <state_interface name=\"velocity\"/>\n";
        urdf << "      <state_interface name=\"effort\"/>\n";
        urdf << "    </joint>\n";
    }
    urdf << "  </ros2_control>\n";
    urdf << "</robot>\n";

    return urdf.str();
}

struct CaseOutcome {
    bool init_ok{false};
    std::size_t state_interfaces{0};
    std::size_t command_interfaces{0};
    std::string first_joint_name;
};

CaseOutcome run_case(std::size_t joint_count, const std::string& prefix) {
    CaseOutcome outcome;

    const auto hardware_infos = hardware_interface::parse_control_resources_from_urdf(make_urdf(joint_count, prefix));
    if (hardware_infos.size() != 1) {
        std::cerr << "  URDF 解析失败，得到 " << hardware_infos.size() << " 个 ros2_control 段\n";
        return outcome;
    }

    arm_controller::ArmRealInterfaces hardware;
    outcome.init_ok = (hardware.on_init(hardware_infos.front()) == hardware_interface::CallbackReturn::SUCCESS);
    if (!outcome.init_ok) {
        return outcome;
    }

    const auto state_interfaces = hardware.export_state_interfaces();
    const auto command_interfaces = hardware.export_command_interfaces();
    outcome.state_interfaces = state_interfaces.size();
    outcome.command_interfaces = command_interfaces.size();
    if (!state_interfaces.empty()) {
        outcome.first_joint_name = state_interfaces.front().get_prefix_name();
    }

    return outcome;
}

bool expect(bool condition, const std::string& what) {
    std::cout << (condition ? "  [PASS] " : "  [FAIL] ") << what << "\n";
    return condition;
}

} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    bool all_ok = true;

    // 情况 1：5 个关节，少于通信帧的 6 个电机 —— 必须拒绝启动。
    // 修复前：on_pack_praser 会写 received_states_[5]，越界写内存。
    std::cout << "[情况 1] 5 关节（少于通信帧）\n";
    {
        const auto outcome = run_case(5, "joint");
        all_ok &= expect(!outcome.init_ok, "on_init 拒绝启动");
    }

    // 情况 2：7 个关节，多于通信帧的 6 个电机 —— 必须拒绝启动。
    // 修复前：第 7 个关节被静默丢弃，接口名却照旧导出 6 个。
    std::cout << "[情况 2] 7 关节（多于通信帧）\n";
    {
        const auto outcome = run_case(7, "joint");
        all_ok &= expect(!outcome.init_ok, "on_init 拒绝启动");
    }

    // 情况 3：6 个关节但名字不是 joint1..joint6 —— 必须成功，且接口名来自 URDF。
    // 修复前：joint_names_ 硬编码为 joint1..joint6，导出的接口名与 URDF 对不上。
    std::cout << "[情况 3] 6 关节，关节名为 arm_joint_1..6（非默认名）\n";
    {
        const auto outcome = run_case(6, "arm_joint_");
        all_ok &= expect(outcome.init_ok, "on_init 成功");
        all_ok &= expect(outcome.state_interfaces == 18, "导出 18 个状态接口（6 关节 x 3）");
        all_ok &= expect(outcome.command_interfaces == 36, "导出 36 个命令接口（6 关节 x 6）");
        all_ok &= expect(outcome.first_joint_name == "arm_joint_1", "首个关节名取自 URDF（实际：" + outcome.first_joint_name + "）");
    }

    std::cout << (all_ok ? "\n结果：全部通过\n" : "\n结果：存在失败项\n");
    rclcpp::shutdown();
    return all_ok ? 0 : 1;
}
