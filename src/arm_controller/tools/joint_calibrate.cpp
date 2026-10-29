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
 * 使用流程（机械臂已在 idel、arm_real.launch.py 已启动）：
 * 1. `ros2 run arm_controller joint_calibrate --ros-args -p joint:=1 -p step:=0.05`
 *    工具把控制器切到 joint_traj，向 +q 方向走一步；松手前**盯着实物**：
 *    - 实物转动的方向与 URDF 里该关节正向一致 → dir = +1；
 *    - 相反 → dir = -1（记下来，Ctrl-C 结束）。
 * 2. 逐关节重复 1，得到 5 个 dir。
 * 3. 手动把机械臂摆到 URDF 零位，读 /joint_states 里各关节的 q（就是 q_urdf）。
 *    zero = dir * q_raw_zero，而 q_raw_zero = dir * q_urdf_zero（dir = ±1），
 *    所以 **zero = q_urdf_zero**。工具每次打印的就是这个量，直接抄进
 *    arm_controller/config/joint_calibration.yaml。
 *
 * 安全约束：
 * - 单步默认 0.05 rad、2 s 走完，速度很低，肉眼可判；
 * - Ctrl-C 立即停止下发，并把控制器切回 idel（保持当前位置）；
 * - 本工具不安装（与 fk_dump / ik_solve 一样留在 build/arm_controller/ 下）。
 */

#include <robot_msgs/msg/joint_traj_cmd.hpp>
#include <robot_msgs/msg/vector.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

namespace {

std::atomic_bool g_stop{false};

void handle_signal(int) {
    g_stop.store(true);
}

/// 控制器里的关节顺序（与 URDF / controller YAML 一致）。
const std::vector<std::string> kJointNames = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

} // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("joint_calibrate");

    const int joint_index  = node->declare_parameter<int>("joint", 1);            // 1..5
    const double step      = node->declare_parameter<double>("step", 0.05);       // rad
    const double duration  = node->declare_parameter<double>("duration", 2.0);    // s
    const double dwell     = node->declare_parameter<double>("dwell", 1.0);       // s
    const int steps        = node->declare_parameter<int>("steps", 1);            // 连续走几步
    const std::string ctrl = node->declare_parameter<std::string>("controller", "arm_controller");

    if (joint_index < 1 || joint_index > 5) {
        RCLCPP_ERROR(node->get_logger(), "joint 必须在 1..5 之间（joint6 没有电机）。");
        return 1;
    }
    if (step <= 0.0 || duration <= 0.0 || steps <= 0) {
        RCLCPP_ERROR(node->get_logger(), "step/duration/steps 必须为正。");
        return 1;
    }

    // ---- 反馈：/joint_states（已经过 dir/zero 换算的 URDF 角）----
    std::vector<double> measured(kJointNames.size(), 0.0);
    std::atomic_bool has_state{false};
    auto state_sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(), [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
            if (msg->name.size() != msg->position.size()) {
                return;
            }
            for (std::size_t i = 0; i < kJointNames.size(); ++i) {
                const auto it = std::find(msg->name.begin(), msg->name.end(), kJointNames[i]);
                if (it != msg->name.end()) {
                    measured[i] = msg->position[static_cast<std::size_t>(std::distance(msg->name.begin(), it))];
                }
            }
            has_state.store(true);
        });

    auto traj_pub = node->create_publisher<robot_msgs::msg::JointTrajCmd>("arm_joint_traj", 10);
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

    std::signal(SIGINT, handle_signal);

    // 等一帧真实反馈：没有 /joint_states 就不要动。
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

    RCLCPP_INFO(
        node->get_logger(), "标定预览（不动作）：joint%d 当前 q_urdf=%+.6f，本步将走到 %+.6f",
        joint_index, measured[static_cast<std::size_t>(joint_index - 1)],
        measured[static_cast<std::size_t>(joint_index - 1)] + step);

    if (!set_state("joint_traj")) {
        RCLCPP_ERROR(node->get_logger(), "无法把控制器切到 joint_traj，退出。");
        rclcpp::shutdown();
        return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    const std::size_t dof = kJointNames.size();
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
        "已切回 idel。把观察到的结果记进 joint_calibration.yaml：\n"
        "  dir  = 实物转动方向与 URDF 正向一致 ? +1 : -1\n"
        "  zero = 机械臂摆到 URDF 零位时读到的 q_urdf（同号）");

    rclcpp::shutdown();
    return 0;
}
