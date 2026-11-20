/**
 * @file joint_calibrate.cpp
 * @brief 实机关节标定工具：一次只动一个关节，用于确认 dir（符号）与 zero（零位）。
 *
 * 为什么需要它：固件上报的角度是 inv_motor × (编码器 − pos_offset)，它的零位是固件
 * 上电标定的物理姿态，与 URDF 的 SolidWorks 零位没有共同基准；符号也可能相反。
 * ArmRealInterfaces 用 joint_calibration.yaml 的 dir/zero 做换算：
 * @code
 *   q_urdf = dir * (q_raw - zero)
 * @endcode
 *
 * 关节数不写死：工具从 /joint_states 的关节名里自动识别 joint1..jointN，
 * 因此 4 轴（joint1..joint4）与 6 轴旧硬件（joint1..joint5）共用同一份工具。
 * 也可以用参数 @c joints 显式指定顺序。
 *
 * 使用流程（机械臂已在 idel、arm_real.launch.py 已启动）：
 * 1. `ros2 run arm_controller joint_calibrate --ros-args -p joint:=1 -p step:=0.05`
 *    工具把控制器切到 joint_traj，向 +q 方向走一步；松手前**盯着实物**：
 *    - 实物转动的方向与 URDF 里该关节正向一致 → dir = +1；
 *    - 相反 → dir = -1（记下来，Ctrl-C 结束）。
 * 2. 逐关节重复 1，得到全部 dir。
 * 3. 手动把机械臂摆到 URDF 零位，用 `-p print_zero:=true` 读当前 /joint_states
 *    与标定 YAML 的 dir/zero，工具按 zero_new = zero_old + dir × q_urdf 打印
 *    可粘贴片段（只打印、不写文件、不动作）。
 *
 * 安全约束：
 * - 单步默认 0.05 rad、2 s 走完，速度很低，肉眼可判；
 * - Ctrl-C 立即停止下发，并把控制器切回 idel（保持当前位置）；
 * - 本工具不安装（与 fk_dump / ik_solve 一样留在 build/arm_controller/ 下）。
 */

#include <robot_msgs/msg/joint_traj_cmd.hpp>
#include <robot_msgs/msg/vector.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

std::atomic_bool g_stop{false};

void handle_signal(int) {
    g_stop.store(true);
}

/// 从 /joint_states 的名字列表里筛出 jointN，按编号升序。
/// @note 关节数由实际模型决定（4 轴 = joint1..joint4，6 轴 = joint1..joint6），
///       工具里不写死；名字不是 joint<数字> 的项（夹爪等）直接忽略。
std::vector<std::string> select_joint_names(const std::vector<std::string>& names) {
    constexpr std::size_t kPrefixLen = 5; // "joint"
    std::vector<std::pair<int, std::string>> indexed;
    for (const auto& name : names) {
        if (name.size() <= kPrefixLen || name.compare(0, kPrefixLen, "joint") != 0) {
            continue;
        }
        int index  = 0;
        bool digit = true;
        for (std::size_t i = kPrefixLen; i < name.size(); ++i) {
            if (std::isdigit(static_cast<unsigned char>(name[i])) == 0) {
                digit = false;
                break;
            }
            index = index * 10 + (name[i] - '0');
        }
        if (digit && index > 0) {
            indexed.emplace_back(index, name);
        }
    }
    std::sort(indexed.begin(), indexed.end(), [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });

    std::vector<std::string> joint_names;
    joint_names.reserve(indexed.size());
    for (const auto& item : indexed) {
        joint_names.push_back(item.second);
    }
    return joint_names;
}

/// 标定表里单个关节的换算参数。
struct CalibrationEntry {
    double dir{1.0};   ///< 符号，只允许 +1 / -1
    double zero{0.0};  ///< 固件零位对应的 q_raw
    bool valid{false}; ///< false = 文件里没有这项或字段非法
};

/// 从标定 YAML 的 joints 段读取一个关节；读不到或非法时返回 valid=false。
CalibrationEntry read_calibration(const std::string& path, const std::string& joint_name) {
    CalibrationEntry entry;
    try {
        const YAML::Node root   = YAML::LoadFile(path);
        const YAML::Node joints = root["joints"];
        if (!joints || !joints[joint_name] || !joints[joint_name].IsMap()) {
            return entry;
        }
        const YAML::Node node = joints[joint_name];
        entry.dir             = node["dir"] ? node["dir"].as<double>() : 0.0;
        entry.zero            = node["zero"] ? node["zero"].as<double>() : 0.0;
        if (std::abs(std::abs(entry.dir) - 1.0) > 1e-9 || !std::isfinite(entry.zero)) {
            return entry;
        }
        entry.valid = true;
    } catch (const std::exception&) {
        return entry;
    }
    return entry;
}

} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("joint_calibrate");

    const int joint_index  = node->declare_parameter<int>("joint", 1);      // 1..N
    const double step      = node->declare_parameter<double>("step", 0.05); // rad
    const double duration  = node->declare_parameter<double>("duration", 2.0); // s
    const double dwell     = node->declare_parameter<double>("dwell", 1.0);    // s
    const int steps        = node->declare_parameter<int>("steps", 1);         // 连续走几步
    const std::string ctrl = node->declare_parameter<std::string>("controller", "arm_controller");
    // print_zero：只读标定表与 /joint_states，打印可粘贴的 zero 片段，不运动。
    const bool print_zero  = node->declare_parameter<bool>("print_zero", false);
    // calibration_file：为空时读 install/arm_controller/share/arm_controller/config/joint_calibration_4dof.yaml。
    const std::string calibration_file = node->declare_parameter<std::string>("calibration_file", "");
    // joints：可选，显式指定关节顺序；为空时从 /joint_states 自动识别 joint1..jointN。
    std::vector<std::string> joint_names =
        node->declare_parameter<std::vector<std::string>>("joints", std::vector<std::string>{});

    if (step <= 0.0 || duration <= 0.0 || steps <= 0) {
        RCLCPP_ERROR(node->get_logger(), "step/duration/steps 必须为正。");
        return 1;
    }

    // ---- 反馈：/joint_states（已经过 dir/zero 换算的 URDF 角）----
    std::vector<double> measured;
    std::atomic_bool has_state{false};
    auto state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(), [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
            if (msg->name.size() != msg->position.size()) {
                return;
            }
            if (joint_names.empty()) {
                joint_names = select_joint_names(msg->name);
                if (joint_names.empty()) {
                    return;
                }
            }
            if (measured.size() != joint_names.size()) {
                measured.assign(joint_names.size(), 0.0);
            }
            for (std::size_t i = 0; i < joint_names.size(); ++i) {
                const auto it = std::find(msg->name.begin(), msg->name.end(), joint_names[i]);
                if (it != msg->name.end()) {
                    measured[i] = msg->position[static_cast<std::size_t>(std::distance(msg->name.begin(), it))];
                }
            }
            has_state.store(true);
        });

    std::signal(SIGINT, handle_signal);

    // 等一帧真实反馈：没有 /joint_states 就不要动，也识别不出关节数。
    RCLCPP_INFO(node->get_logger(), "等待 /joint_states ...");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (rclcpp::ok() && !has_state.load() && !g_stop.load()) {
        rclcpp::spin_some(node);
        if (std::chrono::steady_clock::now() > deadline) {
            RCLCPP_ERROR(node->get_logger(), "10 s 内没有收到 /joint_states，退出（确认 joint_state_broadcaster 已启动）。");
            rclcpp::shutdown();
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    if (joint_names.empty()) {
        RCLCPP_ERROR(node->get_logger(), "/joint_states 里没有 joint<编号> 形式的关节，退出。");
        rclcpp::shutdown();
        return 1;
    }

    // joint 参数在 print_zero 下可以省略（= 打印全部关节）；显式指定时只打印那一个。
    const bool joint_overridden =
        node->get_node_parameters_interface()->get_parameter_overrides().count("joint") > 0;

    if (print_zero) {
        std::string path = calibration_file;
        if (path.empty()) {
            try {
                path = ament_index_cpp::get_package_share_directory("arm_controller") +
                       "/config/joint_calibration_4dof.yaml";
            } catch (const std::exception& error) {
                RCLCPP_ERROR(node->get_logger(), "找不到 arm_controller 的安装目录：%s", error.what());
                rclcpp::shutdown();
                return 1;
            }
        }

        RCLCPP_INFO(
            node->get_logger(),
            "print_zero：请确认机械臂**已人工摆到 URDF 零位**。\n"
            "  读取标定表：%s\n"
            "  换算关系：zero_new = zero_old + dir × q_urdf（只打印，不写文件、不运动）",
            path.c_str());

        if (joint_overridden && (joint_index < 1 || joint_index > static_cast<int>(joint_names.size()))) {
            RCLCPP_ERROR(
                node->get_logger(), "joint 必须在 1..%zu 之间（当前模型 %zu 个关节）。", joint_names.size(),
                joint_names.size());
            rclcpp::shutdown();
            return 1;
        }

        std::vector<std::string> snippets;
        for (std::size_t i = 0; i < joint_names.size(); ++i) {
            if (joint_overridden && static_cast<int>(i) + 1 != joint_index) {
                continue;
            }
            const CalibrationEntry entry = read_calibration(path, joint_names[i]);
            if (!entry.valid) {
                RCLCPP_WARN(
                    node->get_logger(), "  %s: 标定表里没有合法的 dir/zero，跳过（先在 joints 段补占位值）。",
                    joint_names[i].c_str());
                continue;
            }
            const double zero_new = entry.zero + entry.dir * measured[i];
            RCLCPP_INFO(
                node->get_logger(), "  %s: 当前 q_urdf = %+.9f, zero_old = %+.9f, dir = %+.1f", joint_names[i].c_str(),
                measured[i], entry.zero, entry.dir);
            char line[160];
            std::snprintf(line, sizeof(line), "    %s: {dir: %.1f, zero: %.9f}", joint_names[i].c_str(), entry.dir, zero_new);
            snippets.emplace_back(line);
        }

        if (snippets.empty()) {
            RCLCPP_ERROR(node->get_logger(), "没有生成任何片段：检查标定表路径与 joints 段是否写全。");
            rclcpp::shutdown();
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "  可粘贴片段（未写文件）：");
        for (const auto& line : snippets) {
            RCLCPP_INFO(node->get_logger(), "%s", line.c_str());
        }

        rclcpp::shutdown();
        return 0;
    }

    if (joint_index < 1 || joint_index > static_cast<int>(joint_names.size())) {
        RCLCPP_ERROR(
            node->get_logger(), "joint 必须在 1..%zu 之间（当前模型 %zu 个关节）。", joint_names.size(),
            joint_names.size());
        return 1;
    }

    auto traj_pub  = node->create_publisher<robot_msgs::msg::JointTrajCmd>("arm_joint_traj", 10);
    auto param_cli = node->create_client<rcl_interfaces::srv::SetParameters>("/" + ctrl + "/set_parameters");

    const auto set_state = [&](const std::string& state) {
        if (!param_cli->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_WARN(node->get_logger(), "参数服务 /%s/set_parameters 不可用，无法切换状态。", ctrl.c_str());
            return false;
        }
        rcl_interfaces::msg::Parameter param;
        param.name                          = "exp_state";
        param.value.type                    = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
        param.value.string_value            = state;
        auto request                        = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
        request->parameters.push_back(param);
        auto future = param_cli->async_send_request(request);
        // 关键：async_send_request 的响应只有在 node 被 spin 时才会被投递给 future。
        // 只调 future.wait_for() 而不 spin，响应永远到不了，任何 set_parameters 都会
        // 在超时后失败（本工具首版就是这样，一切状态切换都报「超时」）。
        if (rclcpp::spin_until_future_complete(node, future, std::chrono::seconds(3)) != rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_WARN(node->get_logger(), "切换 exp_state=%s 超时。", state.c_str());
            return false;
        }
        const auto response = future.get();
        if (!response) {
            RCLCPP_WARN(node->get_logger(), "切换 exp_state=%s 无响应。", state.c_str());
            return false;
        }
        if (!response->results.empty() && !response->results.front().successful) {
            RCLCPP_WARN(
                node->get_logger(), "切换 exp_state=%s 被拒绝：%s", state.c_str(),
                response->results.front().reason.c_str());
            return false;
        }
        RCLCPP_INFO(node->get_logger(), "exp_state -> %s", state.c_str());
        return true;
    };

    RCLCPP_INFO(
        node->get_logger(), "标定预览（不动作）：joint%d 当前 q_urdf=%+.6f，本步将走到 %+.6f", joint_index,
        measured[static_cast<std::size_t>(joint_index - 1)],
        measured[static_cast<std::size_t>(joint_index - 1)] + step);

    if (!set_state("joint_traj")) {
        RCLCPP_ERROR(node->get_logger(), "无法把控制器切到 joint_traj，退出。");
        rclcpp::shutdown();
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    // 轨迹维度必须等于实际关节数：4 轴发 4 维、6 轴发 6 维，控制器按 joint_count 校验。
    const std::size_t dof = joint_names.size();
    for (int k = 0; k < steps && rclcpp::ok() && !g_stop.load(); ++k) {
        // 用实测值做起点：每步都从“实际在哪”出发，避免累积误差。
        std::vector<float> start(dof, 0.0f);
        std::vector<float> goal(dof, 0.0f);
        for (std::size_t i = 0; i < dof; ++i) {
            start[i] = static_cast<float>(measured[i]);
            goal[i]  = static_cast<float>(measured[i]);
        }
        goal[static_cast<std::size_t>(joint_index - 1)] += static_cast<float>(step);

        robot_msgs::msg::JointTrajCmd msg;
        msg.seconds = {0.0f, static_cast<float>(duration)};
        for (const auto& point : {start, goal}) {
            robot_msgs::msg::Vector vector;
            vector.pos = point;
            msg.position.push_back(vector);
        }
        traj_pub->publish(msg);

        RCLCPP_INFO(
            node->get_logger(), "[%d/%d] joint%d: %+.4f -> %+.4f（请盯住实物方向）", k + 1, steps, joint_index,
            start[static_cast<std::size_t>(joint_index - 1)], goal[static_cast<std::size_t>(joint_index - 1)]);

        const auto done = std::chrono::steady_clock::now() + std::chrono::duration<double>(duration + dwell);
        while (rclcpp::ok() && !g_stop.load() && std::chrono::steady_clock::now() < done) {
            rclcpp::spin_some(node);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        RCLCPP_INFO(
            node->get_logger(), "       实测 q_urdf(joint%d) = %+.6f", joint_index,
            measured[static_cast<std::size_t>(joint_index - 1)]);
    }

    // 退出前回到 idel：锁定当前位置，不再接受新的关节轨迹。
    set_state("idel");
    RCLCPP_INFO(
        node->get_logger(),
        "已切回 idel。把观察到的结果记进标定 YAML：\n"
        "  dir  = 实物转动方向与 URDF 正向一致 ? +1 : -1\n"
        "  zero = 机械臂摆到 URDF 零位后用 -p print_zero:=true 生成的片段");

    rclcpp::shutdown();
    return 0;
}
