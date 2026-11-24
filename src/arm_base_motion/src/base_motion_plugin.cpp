#include "arm_base_motion/base_motion_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <pluginlib/class_list_macros.hpp>

namespace arm_base_motion
{

namespace
{

constexpr double kTwoPi = 6.283185307179586476925286766559;
constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

/// 插件参数名带 mujoco_plugins.<sub_namespace>. 前缀，与上游插件保持同一约定。
std::string namespaced_param_name(const rclcpp::Node::SharedPtr& node, const std::string& name) {
    const std::string sub_namespace = node->get_sub_namespace();
    return sub_namespace.empty() ? name : ("mujoco_plugins." + sub_namespace + "." + name);
}

template <typename T>
T declare_or_get(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& default_value) {
    const std::string full_name = namespaced_param_name(node, name);
    if (!node->has_parameter(full_name)) {
        node->declare_parameter(full_name, default_value);
    }
    return node->get_parameter(full_name).get_value<T>();
}

} // namespace

bool BaseMotionPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) {
    node_   = node;
    logger_ = node->get_logger().get_child(node->get_sub_namespace());
    model_  = model;

    if (node_ == nullptr || model_ == nullptr || data == nullptr) {
        RCLCPP_ERROR(logger_, "BaseMotionPlugin init called with null node/model/data");
        return false;
    }

    const std::string body_name = declare_or_get<std::string>(node_, "body", "");
    if (body_name.empty()) {
        RCLCPP_ERROR(logger_, "BaseMotionPlugin requires the 'body' parameter (MJCF body with mocap=\"true\")");
        return false;
    }

    const int body_id = mj_name2id(model_, mjOBJ_BODY, body_name.c_str());
    if (body_id < 0) {
        RCLCPP_ERROR(logger_, "body '%s' not found in the MuJoCo model", body_name.c_str());
        return false;
    }

    mocap_id_ = model_->body_mocapid[body_id];
    if (mocap_id_ < 0) {
        RCLCPP_ERROR(
            logger_, "body '%s' is not a mocap body; BaseMotionPlugin drives a kinematic base "
                     "(add mocap=\"true\" to the body and use the generated MJCF)",
            body_name.c_str());
        return false;
    }

    // 初始位姿作为轨迹零点：z 与 roll/pitch 全程不变，只有 x/y/yaw 随轨迹变化。
    init_time_ = data->time;
    x0_        = data->mocap_pos[3 * mocap_id_ + 0];
    y0_        = data->mocap_pos[3 * mocap_id_ + 1];
    z0_        = data->mocap_pos[3 * mocap_id_ + 2];
    const double qw = data->mocap_quat[4 * mocap_id_ + 0];
    const double qx = data->mocap_quat[4 * mocap_id_ + 1];
    const double qy = data->mocap_quat[4 * mocap_id_ + 2];
    const double qz = data->mocap_quat[4 * mocap_id_ + 3];
    yaw0_           = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    start_delay_    = declare_or_get<double>(node_, "start_delay", 5.0);
    x_amplitude_    = declare_or_get<double>(node_, "x_amplitude", 0.0);
    x_frequency_    = declare_or_get<double>(node_, "x_frequency", 0.0);
    x_phase_        = declare_or_get<double>(node_, "x_phase", 0.0);
    y_amplitude_    = declare_or_get<double>(node_, "y_amplitude", 0.0);
    y_frequency_    = declare_or_get<double>(node_, "y_frequency", 0.0);
    y_phase_        = declare_or_get<double>(node_, "y_phase", 0.0);
    yaw_amplitude_  = declare_or_get<double>(node_, "yaw_amplitude_deg", 0.0) * kDegToRad;
    yaw_frequency_  = declare_or_get<double>(node_, "yaw_frequency", 0.0);
    yaw_phase_      = declare_or_get<double>(node_, "yaw_phase", 0.0);

    if (start_delay_ < 0.0) {
        RCLCPP_WARN(logger_, "start_delay=%.2f s is negative; clamped to 0", start_delay_);
        start_delay_ = 0.0;
    }

    const std::string topic = declare_or_get<std::string>(node_, "topic", "base_motion_state");
    publisher_ = std::make_unique<realtime_tools::RealtimePublisher<robot_msgs::msg::BaseMotionState>>(
        node_->create_publisher<robot_msgs::msg::BaseMotionState>(topic, rclcpp::QoS(rclcpp::KeepLast(5))));

    RCLCPP_INFO(
        logger_,
        "BaseMotionPlugin initialised on mocap body '%s' (mocap id %d, initial pose %.3f, %.3f, %.3f). topic='%s', "
        "start_delay=%.1f s, x: %.3f m @ %.3f Hz (phase %.2f), y: %.3f m @ %.3f Hz (phase %.2f), "
        "yaw: %.2f deg @ %.3f Hz (phase %.2f)",
        body_name.c_str(), mocap_id_, x0_, y0_, z0_, topic.c_str(), start_delay_, x_amplitude_, x_frequency_, x_phase_,
        y_amplitude_, y_frequency_, y_phase_, yaw_amplitude_ / kDegToRad, yaw_frequency_, yaw_phase_);

    // 先把基座钉在初始位姿：延时阶段基座不动，也不让重力把自由基座拉下去。
    write_state(data, MotionState{});
    return true;
}

BaseMotionPlugin::MotionState BaseMotionPlugin::evaluate(double tau) const {
    MotionState motion;

    const double wx = kTwoPi * x_frequency_;
    const double wy = kTwoPi * y_frequency_;
    const double wz = kTwoPi * yaw_frequency_;

    // 每条轴都减去 sin(φ)：保证 τ = 0 时位移为 0（基座从初始位形连续起步），
    // 否则非零相位会在 start_delay 到期的那一拍给出一个位置阶跃（实测 10 cm）。
    motion.x         = x_amplitude_ * (std::sin(wx * tau + x_phase_) - std::sin(x_phase_));
    motion.vx        = x_amplitude_ * wx * std::cos(wx * tau + x_phase_);
    motion.ax        = -x_amplitude_ * wx * wx * std::sin(wx * tau + x_phase_);

    motion.y         = y_amplitude_ * (std::sin(wy * tau + y_phase_) - std::sin(y_phase_));
    motion.vy        = y_amplitude_ * wy * std::cos(wy * tau + y_phase_);
    motion.ay        = -y_amplitude_ * wy * wy * std::sin(wy * tau + y_phase_);

    motion.yaw       = yaw_amplitude_ * (std::sin(wz * tau + yaw_phase_) - std::sin(yaw_phase_));
    motion.yaw_rate  = yaw_amplitude_ * wz * std::cos(wz * tau + yaw_phase_);
    motion.yaw_accel = -yaw_amplitude_ * wz * wz * std::sin(wz * tau + yaw_phase_);

    return motion;
}

void BaseMotionPlugin::write_state(mjData* data, const MotionState& motion) const {
    if (data == nullptr || mocap_id_ < 0) {
        return;
    }

    const double yaw                    = yaw0_ + motion.yaw;
    data->mocap_pos[3 * mocap_id_ + 0]  = x0_ + motion.x;
    data->mocap_pos[3 * mocap_id_ + 1]  = y0_ + motion.y;
    data->mocap_pos[3 * mocap_id_ + 2]  = z0_;
    // 绕 world z 轴 yaw：四元数 (w, x, y, z)。
    data->mocap_quat[4 * mocap_id_ + 0] = std::cos(0.5 * yaw);
    data->mocap_quat[4 * mocap_id_ + 1] = 0.0;
    data->mocap_quat[4 * mocap_id_ + 2] = 0.0;
    data->mocap_quat[4 * mocap_id_ + 3] = std::sin(0.5 * yaw);
}

void BaseMotionPlugin::pre_step(mjData* data) {
    if (data == nullptr || mocap_id_ < 0) {
        return;
    }

    // τ = max(0, t − start_delay)：延时阶段停在轨迹零点（初始位姿），位置/速度都连续。
    const double tau = std::max(0.0, data->time - init_time_ - start_delay_);
    write_state(data, evaluate(tau));
}

void BaseMotionPlugin::update(const mjModel* /*model*/, mjData* data) {
    if (publisher_ == nullptr || data == nullptr || !publisher_->trylock()) {
        return;
    }

    const double tau           = std::max(0.0, data->time - init_time_ - start_delay_);
    const MotionState motion   = evaluate(tau);
    const bool motion_active   = (data->time - init_time_) >= start_delay_;

    auto& message                     = publisher_->msg_;
    message.header.stamp              = rclcpp::Time(static_cast<int64_t>(std::llround(data->time * 1e9)));
    message.header.frame_id           = "world";

    const double yaw                  = yaw0_ + motion.yaw;
    message.pose.position.x           = x0_ + motion.x;
    message.pose.position.y           = y0_ + motion.y;
    message.pose.position.z           = z0_;
    message.pose.orientation.w        = std::cos(0.5 * yaw);
    message.pose.orientation.x        = 0.0;
    message.pose.orientation.y        = 0.0;
    message.pose.orientation.z        = std::sin(0.5 * yaw);

    message.twist.linear.x            = motion.vx;
    message.twist.linear.y            = motion.vy;
    message.twist.linear.z            = 0.0;
    message.twist.angular.x           = 0.0;
    message.twist.angular.y           = 0.0;
    message.twist.angular.z           = motion.yaw_rate;

    message.accel.linear.x            = motion.ax;
    message.accel.linear.y            = motion.ay;
    message.accel.linear.z            = 0.0;
    message.accel.angular.x           = 0.0;
    message.accel.angular.y           = 0.0;
    message.accel.angular.z           = motion.yaw_accel;

    message.motion_active             = motion_active;

    publisher_->unlockAndPublish();
}

void BaseMotionPlugin::cleanup() {
    publisher_.reset();
}

} // namespace arm_base_motion

PLUGINLIB_EXPORT_CLASS(arm_base_motion::BaseMotionPlugin, mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
