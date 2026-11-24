#pragma once

#include <memory>
#include <string>

#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp>
#include <robot_msgs/msg/base_motion_state.hpp>

namespace arm_base_motion
{

/**
 * @brief 脚本化基座运动驱动插件（MuJoCo mocap 基座，运动学驱动）
 *
 * 基座 body 在 MJCF 里是 `mocap="true"` 的运动学体，每个物理步之前直接写
 * @c data->mocap_pos / @c mocap_quat，使基座按参数化轨迹运动：
 *
 *   x(τ)   = x0 + Ax·sin(2π·fx·τ + φx)
 *   y(τ)   = Ay·sin(2π·fy·τ + φy)
 *   yaw(τ) = yaw0 + Ayaw·sin(2π·fyaw·τ + φyaw)      τ = max(0, t − start_delay)
 *
 * 为什么用 mocap 而不是 freejoint：
 * - freejoint 的基座仍然有质量，手臂反力会让它产生加速度；把 qpos/qvel 每步「钉回去」
 *   并不能消除这项加速度，手臂会看到一个被放大/反向的重力（实测 joint qacc 与固定基座
 *   差 4 倍以上，控制器开重力前馈后直接发散）；
 * - mocap 体没有自由度、没有加速度，手臂的关节动力学与固定基座模型**逐位一致**
 *   （离线对比：初始 joint qacc 完全相同），控制的物理假设与控制器里的固定基座
 *   Pinocchio 模型正好对齐。
 * 代价是基座速度/加速度只体现在解析轨迹与话题里，不进入 MuJoCo 的科氏/离心项——
 * 本实验的基座速度（<0.3 m/s、<0.25 rad/s）下这一项相对 g 小于 0.3%，可忽略。
 *
 * 同时在 @c update() 里以控制频率发布 @c robot_msgs/msg/BaseMotionState（world 系位姿 +
 * 线/角速度 + 线/角加速度），供控制器 inertial_hold 状态订阅。
 *
 * 参数（YAML 段 @c mujoco_plugins.<name>.*）：
 * - @c body                ：mocap 基座 body 名（必填，例如 base_link）
 * - @c topic               ：基座状态话题（默认 base_motion_state）
 * - @c start_delay         ：轨迹启动延时（秒，默认 5.0），延时期内基座钉在初始位姿
 * - @c x_amplitude / @c x_frequency / @c x_phase
 * - @c y_amplitude / @c y_frequency / @c y_phase
 * - @c yaw_amplitude_deg   / @c yaw_frequency / @c yaw_phase（角度用度，内部转弧度）
 */
class BaseMotionPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
  BaseMotionPlugin() = default;
  ~BaseMotionPlugin() override = default;

  bool init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* data) override;
  void pre_step(mjData* data) override;
  void update(const mjModel* model, mjData* data) override;
  void cleanup() override;

private:
  /// 轨迹在 τ（秒）处的状态与一、二阶导数
  struct MotionState {
    double x{ 0.0 };
    double y{ 0.0 };
    double yaw{ 0.0 };
    double vx{ 0.0 };
    double vy{ 0.0 };
    double yaw_rate{ 0.0 };
    double ax{ 0.0 };
    double ay{ 0.0 };
    double yaw_accel{ 0.0 };
  };

  /// 在 τ（已扣除 start_delay，>= 0）处求值
  MotionState evaluate(double tau) const;

  /// 把轨迹状态写进 live mjData 的 mocap 位姿
  void write_state(mjData* data, const MotionState& motion) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Logger logger_{ rclcpp::get_logger("BaseMotionPlugin") };
  const mjModel* model_{ nullptr };

  int mocap_id_{ -1 };  ///< base body 的 mocap 下标（body_mocapid）

  double init_time_{ 0.0 };  ///< init 时刻的仿真时间，作为轨迹时间原点
  double x0_{ 0.0 };         ///< 初始位姿（轨迹的零点）
  double y0_{ 0.0 };
  double z0_{ 0.0 };
  double yaw0_{ 0.0 };

  double start_delay_{ 5.0 };
  double x_amplitude_{ 0.0 };
  double x_frequency_{ 0.0 };
  double x_phase_{ 0.0 };
  double y_amplitude_{ 0.0 };
  double y_frequency_{ 0.0 };
  double y_phase_{ 0.0 };
  double yaw_amplitude_{ 0.0 };  ///< 弧度
  double yaw_frequency_{ 0.0 };
  double yaw_phase_{ 0.0 };

  std::unique_ptr<realtime_tools::RealtimePublisher<robot_msgs::msg::BaseMotionState>> publisher_;
};

}  // namespace arm_base_motion
