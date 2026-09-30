#include "task_context.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <utility>

#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/parameter_value.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.hpp>

#include <robot_msgs/msg/vector.hpp>

namespace {

/// 旋转角过小（sin(angle/2) 趋近 0）时旋转向量直接取零，避免除零。
constexpr double kMinSinHalfAngle = 1e-9;

/// 任务空间自由度：3 位置 + 3 旋转向量。
constexpr std::size_t kTaskDof = 6;

/**
 * @brief 把轨迹点填充为 *TrajCmd 消息
 * @tparam MessageT 消息类型（CartTrajCmd / JointTrajCmd，字段布局一致）
 * @param points  轨迹点，每点长度必须等于 @p dof
 * @param seconds 与 @p points 一一对应的时刻（秒）
 * @param dof     单点自由度
 * @param msg     输出消息
 * @return true 填充成功；false 参数非法
 */
template <typename MessageT>
bool fill_trajectory_message(
    const std::vector<std::vector<float>>& points, const std::vector<float>& seconds, std::size_t dof,
    MessageT* msg) {
    if (msg == nullptr || points.empty() || points.size() != seconds.size()) {
        return false;
    }

    msg->seconds = seconds;
    msg->position.reserve(points.size());
    for (const auto& point : points) {
        if (point.size() != dof) {
            return false;
        }
        robot_msgs::msg::Vector vector;
        vector.pos = point;
        msg->position.push_back(std::move(vector));
    }
    return true;
}

/**
 * @brief 四元数转旋转向量（angle * axis）
 * @param x 四元数 x 分量
 * @param y 四元数 y 分量
 * @param z 四元数 z 分量
 * @param w 四元数 w 分量
 * @param rotation_vector 输出：长度 3 的旋转向量
 *
 * 先把四元数统一到 w >= 0 的等价形式，保证转角落在 [0, pi]，
 * 与控制器 Default6DofTaskSpaceMapping 使用的 Eigen::AngleAxisd 语义一致。
 */
void quaternion_to_rotation_vector(double x, double y, double z, double w, std::vector<double>* rotation_vector) {
    if (rotation_vector == nullptr || rotation_vector->size() != 3) {
        return;
    }

    if (w < 0.0) {
        x = -x;
        y = -y;
        z = -z;
        w = -w;
    }
    w = std::clamp(w, -1.0, 1.0);

    const double half_angle = std::acos(w);
    const double sin_half   = std::sin(half_angle);
    if (std::abs(sin_half) < kMinSinHalfAngle) {
        rotation_vector->assign(3, 0.0);
        return;
    }

    const double scale = 2.0 * half_angle / sin_half;
    (*rotation_vector)[0] = scale * x;
    (*rotation_vector)[1] = scale * y;
    (*rotation_vector)[2] = scale * z;
}

}  // namespace

TaskContext::TaskContext(const rclcpp::Node::SharedPtr& node, Options options)
    : node_(node)
    , options_(std::move(options)) {
    joint_positions_.assign(options_.joints.size(), 0.0);

    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) { on_joint_state(msg); });

    cart_traj_pub_  = node_->create_publisher<robot_msgs::msg::CartTrajCmd>(options_.cart_traj_topic, 10);
    joint_traj_pub_ = node_->create_publisher<robot_msgs::msg::JointTrajCmd>(options_.joint_traj_topic, 10);

    set_param_cli_ = node_->create_client<rcl_interfaces::srv::SetParameters>(
        "/" + options_.controller_node + "/set_parameters");

    tf_buffer_   = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
}

bool TaskContext::wait_for_controller(double timeout_sec) const {
    if (!set_param_cli_->wait_for_service(std::chrono::duration<double>(timeout_sec))) {
        RCLCPP_ERROR(
            node_->get_logger(), "service /%s/set_parameters is unavailable; is the controller loaded and active?",
            options_.controller_node.c_str());
        return false;
    }
    return true;
}

bool TaskContext::set_exp_state(const std::string& state_name, double timeout_sec) {
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    rcl_interfaces::msg::Parameter param;
    param.name               = "exp_state";
    param.value.type         = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    param.value.string_value = state_name;
    request->parameters.push_back(param);

    auto future = set_param_cli_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::duration<double>(timeout_sec))
        != rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_ERROR(node_->get_logger(), "failed to set exp_state=%s", state_name.c_str());
        return false;
    }

    const auto response = future.get();
    if (!response->results.empty() && !response->results.front().successful) {
        RCLCPP_ERROR(
            node_->get_logger(), "set exp_state=%s rejected: %s", state_name.c_str(),
            response->results.front().reason.c_str());
        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "exp_state -> %s", state_name.c_str());
    return true;
}

bool TaskContext::wait_for_joint_states(double timeout_sec) {
    rclcpp::WallRate rate(100.0);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
        spin_some();
        if (have_joint_states_) {
            return true;
        }
        rate.sleep();
    }
    RCLCPP_ERROR(node_->get_logger(), "no usable /joint_states received; is joint_state_broadcaster running?");
    return false;
}

bool TaskContext::end_effector_pose(std::vector<double>* pose) const {
    if (pose == nullptr) {
        return false;
    }

    try {
        const auto transform = tf_buffer_->lookupTransform(options_.base_frame, options_.ee_frame, tf2::TimePointZero);
        pose->assign(6, 0.0);
        (*pose)[0] = transform.transform.translation.x;
        (*pose)[1] = transform.transform.translation.y;
        (*pose)[2] = transform.transform.translation.z;

        std::vector<double> rotation_vector(3, 0.0);
        quaternion_to_rotation_vector(
            transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z,
            transform.transform.rotation.w, &rotation_vector);
        (*pose)[3] = rotation_vector[0];
        (*pose)[4] = rotation_vector[1];
        (*pose)[5] = rotation_vector[2];
        return true;
    } catch (const tf2::TransformException& error) {
        RCLCPP_WARN_THROTTLE(
            node_->get_logger(), *node_->get_clock(), 2000, "TF %s -> %s lookup failed: %s",
            options_.base_frame.c_str(), options_.ee_frame.c_str(), error.what());
        return false;
    }
}

bool TaskContext::publish_cart_traj(const std::vector<std::vector<float>>& points, const std::vector<float>& seconds) const {
    robot_msgs::msg::CartTrajCmd msg;
    if (!fill_trajectory_message(points, seconds, kTaskDof, &msg)) {
        return false;
    }

    cart_traj_pub_->publish(msg);
    return true;
}

bool TaskContext::publish_joint_traj(const std::vector<std::vector<float>>& points, const std::vector<float>& seconds) const {
    robot_msgs::msg::JointTrajCmd msg;
    if (!fill_trajectory_message(points, seconds, options_.joints.size(), &msg)) {
        return false;
    }

    joint_traj_pub_->publish(msg);
    return true;
}

void TaskContext::fail(const std::string& message) {
    if (!failed_) {
        failure_message_ = message;
    }
    failed_ = true;
}

void TaskContext::spin_some() {
    rclcpp::spin_some(node_);
}

void TaskContext::on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (msg == nullptr || msg->name.size() != msg->position.size()) {
        return;
    }

    std::vector<double> positions;
    positions.reserve(options_.joints.size());
    for (const auto& joint : options_.joints) {
        const auto it = std::find(msg->name.begin(), msg->name.end(), joint);
        if (it == msg->name.end()) {
            return;
        }
        const auto index = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
        positions.push_back(msg->position[index]);
    }

    joint_positions_   = std::move(positions);
    have_joint_states_ = true;
}
