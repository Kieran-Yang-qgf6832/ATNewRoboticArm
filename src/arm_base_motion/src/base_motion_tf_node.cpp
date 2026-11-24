#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

#include <robot_msgs/msg/base_motion_state.hpp>

namespace arm_base_motion
{

/**
 * @brief 把基座状态话题转成 world→base_link 的动态 TF
 *
 * 为什么需要它：URDF 的 floating_base 分支去掉了 world_to_base 固定关节，让 base_link
 * 成为 TF 树的根（robot_state_publisher 不再发布基座变换）。基座在 MuJoCo 里是 mocap
 * 运动学体，这里按同一份基座状态话题补上 world→base_link，RViz 里才能看到「基座在动、末端
 * 钉在原地」，而不是「基座不动、机械臂在乱动」。
 *
 * 参数：
 * - @c topic        ：基座状态话题（默认 base_motion_state）
 * - @c world_frame  ：父坐标系（默认 world）
 * - @c base_frame   ：子坐标系（默认 base_link）
 * - @c tf_rate      ：TF 发布频率上限（Hz，默认 100；基座状态是 500 Hz，全转发没必要）
 *
 * @note 时间戳直接取消息 header.stamp（与 /joint_states 同一仿真时钟），不做插值。
 */
class BaseMotionTfNode : public rclcpp::Node
{
public:
  BaseMotionTfNode()
    : rclcpp::Node("base_motion_tf")
  {
    topic_ = declare_parameter<std::string>("topic", "base_motion_state");
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const double tf_rate = declare_parameter<double>("tf_rate", 100.0);
    min_period_ = tf_rate > 0.0 ? rclcpp::Duration::from_seconds(1.0 / tf_rate) : rclcpp::Duration::from_seconds(0.0);

    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    subscription_ = create_subscription<robot_msgs::msg::BaseMotionState>(
      topic_, rclcpp::QoS(rclcpp::KeepLast(5)),
      [this](const robot_msgs::msg::BaseMotionState::SharedPtr message) { on_base_motion_state(*message); });

    RCLCPP_INFO(
      get_logger(), "base_motion_tf: %s -> %s from '%s' (<= %.0f Hz)", world_frame_.c_str(), base_frame_.c_str(),
      topic_.c_str(), tf_rate);
  }

private:
  void on_base_motion_state(const robot_msgs::msg::BaseMotionState& message)
  {
    const rclcpp::Time stamp(message.header.stamp);
    if (last_stamp_set_ && (stamp - last_stamp_) < min_period_) {
      return;
    }
    last_stamp_ = stamp;
    last_stamp_set_ = true;

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = message.header.stamp;
    transform.header.frame_id = world_frame_;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = message.pose.position.x;
    transform.transform.translation.y = message.pose.position.y;
    transform.transform.translation.z = message.pose.position.z;
    transform.transform.rotation = message.pose.orientation;
    broadcaster_->sendTransform(transform);
  }

  std::string topic_;
  std::string world_frame_;
  std::string base_frame_;
  rclcpp::Duration min_period_{ 0, 0 };
  rclcpp::Time last_stamp_{ 0, 0, RCL_ROS_TIME };
  bool last_stamp_set_{ false };

  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<robot_msgs::msg::BaseMotionState>::SharedPtr subscription_;
};

}  // namespace arm_base_motion

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm_base_motion::BaseMotionTfNode>());
  rclcpp::shutdown();
  return 0;
}
