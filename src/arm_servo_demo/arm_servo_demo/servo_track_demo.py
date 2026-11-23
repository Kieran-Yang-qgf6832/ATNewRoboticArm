"""伺服模式跟踪演示。

一个虚拟球沿水平圆周缓慢移动，末端以速度伺服跟随。演示节点负责全流程：

  1. 等待 TF、/joint_states 与控制器参数服务就绪；
  2. 切到 joint_traj，发一条关节轨迹把机械臂摆到 ready 位形并等到位；
  3. 切到 servo，等状态确认后开始 50 Hz 发速度指令：
         v = v_ball + gain * (p_ball - p_ee)，限幅 max_speed，角速度恒 0
  4. 跑满 duration 后发 0.5 s 零速度，再切回 idel，打印误差统计并给出通过/失败。

球的轨迹：圆心 = 进入伺服时的末端位置 + radius * base_link 的 +x 方向，
半径 radius、周期 period，因此 t=0 时球与末端重合，没有启动追赶。
"""

import math
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from robot_msgs.msg import JointTrajCmd, ServoCtrlCmd, Vector

# 状态话题由控制器的 500 ms 墙钟定时器发布，并带 transient_local，
# 晚加入的订阅者也能立刻拿到当前状态。
STATE_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ServoTrackDemo(Node):
    """演示流程与追踪控制律。"""

    def __init__(self):
        super().__init__('servo_track_demo')

        # ------------------------------------------------------------------
        # 参数
        # ------------------------------------------------------------------
        self.controller_node = self.declare_parameter('controller_node', 'arm_controller').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.ee_frame = self.declare_parameter('ee_frame', 'link6').value
        self.joint_names = list(self.declare_parameter(
            'joints', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']).value)
        self.ready_joints = [float(v) for v in self.declare_parameter(
            'ready_joints', [0.0, 0.3, -0.3, 0.0, 0.3, 0.0]).value]

        self.radius = float(self.declare_parameter('radius', 0.05).value)
        self.period = float(self.declare_parameter('period', 20.0).value)
        self.gain = float(self.declare_parameter('gain', 2.0).value)
        self.max_speed = float(self.declare_parameter('max_speed', 0.05).value)
        self.command_rate = float(self.declare_parameter('command_rate', 50.0).value)
        self.duration = float(self.declare_parameter('duration', 40.0).value)

        self.preposition_duration = float(self.declare_parameter('preposition_duration', 3.0).value)
        self.reach_tolerance = float(self.declare_parameter('reach_tolerance', 0.02).value)
        self.arrive_mean = float(self.declare_parameter('arrive_mean', 0.01).value)
        self.arrive_peak = float(self.declare_parameter('arrive_peak', 0.02).value)
        self.max_tf_failures = int(self.declare_parameter('max_tf_failures', 20).value)
        self.report_period = float(self.declare_parameter('report_period', 2.0).value)
        self.ball_marker_size = float(self.declare_parameter('ball_marker_size', 0.04).value)

        if self.radius <= 0.0 or self.period <= 0.0 or self.command_rate <= 0.0:
            raise ValueError('radius / period / command_rate must be positive')

        # ------------------------------------------------------------------
        # ROS 接口
        # ------------------------------------------------------------------
        state_topic = '/%s/state' % self.controller_node
        self.create_subscription(String, state_topic, self._on_state, STATE_QOS)
        # 用 SensorDataQoS（best effort）：对 reliable 与 best-effort 两种发布端都兼容，
        # 避免因 QoS 不匹配而收不到 /joint_states。
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, qos_profile_sensor_data)

        self.servo_pub = self.create_publisher(ServoCtrlCmd, '/arm_servo', 10)
        # 关节轨迹话题是控制器节点内的相对名，控制器的节点命名空间为根命名空间。
        self.joint_traj_pub = self.create_publisher(JointTrajCmd, '/arm_joint_traj', 10)
        # RViz 的 MarkerArray 显示已在默认配置里启用，这里直接复用该话题。
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)
        self.ball_pose_pub = self.create_publisher(PoseStamped, '/servo_demo/ball_pose', 10)

        self.set_param_cli = self.create_client(
            SetParameters, '/%s/set_parameters' % self.controller_node)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ------------------------------------------------------------------
        # 运行态
        # ------------------------------------------------------------------
        self.controller_state = ''
        self.joint_positions = {}
        self.zero_sent = False
        self.in_servo = False

    # ----------------------------------------------------------------------
    # 回调
    # ----------------------------------------------------------------------
    def _on_state(self, msg):
        self.controller_state = msg.data

    def _on_joint_state(self, msg):
        self.joint_positions = dict(zip(msg.name, msg.position))

    # ----------------------------------------------------------------------
    # 通用工具
    # ----------------------------------------------------------------------
    def spin_for(self, seconds):
        """在保持回调运转的前提下等待一段墙钟时间。"""
        deadline = time.monotonic() + max(0.0, seconds)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=min(0.02, max(0.0, deadline - time.monotonic())))

    def wait_for(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if predicate():
                return True
            self.spin_for(0.02)
        if predicate():
            return True
        self.get_logger().error('timed out waiting for %s (%.1f s)' % (what, timeout))
        return False

    def set_exp_state(self, state_name):
        """通过参数服务请求控制器切换期望状态。"""
        if not self.set_param_cli.service_is_ready() and not self.set_param_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('service %s is unavailable' % self.set_param_cli.srv_name)
            return False

        request = SetParameters.Request()
        parameter = ParameterMsg()
        parameter.name = 'exp_state'
        parameter.value = ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=state_name)
        request.parameters = [parameter]

        future = self.set_param_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None:
            self.get_logger().error('no response when setting exp_state=%s' % state_name)
            return False
        if not all(result.successful for result in response.results):
            reason = '; '.join(result.reason for result in response.results if not result.successful)
            self.get_logger().error('exp_state=%s rejected: %s' % (state_name, reason))
            return False
        return True

    def joint_vector(self):
        if not self.joint_positions:
            return None
        try:
            return [float(self.joint_positions[name]) for name in self.joint_names]
        except KeyError:
            return None

    def max_ready_residual(self):
        current = self.joint_vector()
        if current is None:
            return float('inf')
        return max(abs(current[i] - self.ready_joints[i]) for i in range(len(self.ready_joints)))

    def ee_position(self):
        transform = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, Time())
        translation = transform.transform.translation
        return (translation.x, translation.y, translation.z)

    # ----------------------------------------------------------------------
    # 流程
    # ----------------------------------------------------------------------
    def wait_ready(self):
        if not self.wait_for(lambda: bool(self.joint_positions), 20.0, '/joint_states'):
            return False
        # 先空转半秒，让 TF 监听器攒到第一帧（仿真启动初期 /clock 与 TF 都还没就绪）。
        self.spin_for(0.5)
        return self.wait_for(lambda: self._tf_available(), 20.0,
                             'TF %s -> %s' % (self.base_frame, self.ee_frame))

    def _tf_available(self):
        try:
            self.ee_position()
            return True
        except Exception:
            return False

    def preposition(self):
        start = self.joint_vector()
        if start is None:
            self.get_logger().error('no valid /joint_states before the preposition move')
            return False

        if not self.set_exp_state('joint_traj'):
            return False
        if not self.wait_for(lambda: self.controller_state == 'joint_traj', 5.0, 'state joint_traj'):
            return False
        self.spin_for(0.3)

        message = JointTrajCmd()
        message.seconds = [0.0, float(self.preposition_duration)]
        first = Vector()
        first.pos = [float(value) for value in start]
        target = Vector()
        target.pos = [float(value) for value in self.ready_joints]
        message.position = [first, target]
        self.joint_traj_pub.publish(message)
        self.get_logger().info(
            'preposition: moving to ready in %.1f s, target=%s' % (self.preposition_duration, self.ready_joints))

        reached = self.wait_for(lambda: self.max_ready_residual() <= self.reach_tolerance,
                                self.preposition_duration + 3.0, 'ready configuration')
        if not reached:
            self.get_logger().error('ready residual is %.4f rad (tolerance %.4f)' % (
                self.max_ready_residual(), self.reach_tolerance))
            return False
        if self.controller_state != 'joint_traj':
            self.get_logger().error('controller left joint_traj during the preposition move (now "%s")'
                                    % self.controller_state)
            return False
        return True

    def enter_servo(self):
        if not self.set_exp_state('servo'):
            return False
        if not self.wait_for(lambda: self.controller_state == 'servo', 5.0, 'state servo'):
            return False
        self.in_servo = True
        return True

    def track(self):
        """跑满 duration，返回 True 表示通过验收。"""
        try:
            start = self.ee_position()
        except Exception as error:
            self.get_logger().error('cannot read the end-effector pose: %s' % error)
            return False

        # 圆心偏向 +x，相位从 π 起算：t=0 时球与末端重合，之后末端最多离开起点 2*radius。
        center = (start[0] + self.radius, start[1], start[2])
        omega = 2.0 * math.pi / self.period
        self.get_logger().info(
            'tracking: center=(%.4f, %.4f, %.4f) radius=%.3f m period=%.1f s duration=%.1f s '
            '(ball speed %.2f mm/s)' % (center[0], center[1], center[2], self.radius, self.period,
                                        self.duration, self.radius * omega * 1000.0))

        period = 1.0 / self.command_rate
        start_time = self.get_clock().now()
        next_tick = time.monotonic()
        next_report = self.report_period

        error_sum = 0.0
        error_peak = 0.0
        sample_count = 0
        tf_failures = 0
        consecutive_tf_failures = 0
        servo_exited = False

        while rclpy.ok():
            elapsed = (self.get_clock().now() - start_time).nanoseconds * 1e-9
            if elapsed >= self.duration:
                break

            if self.controller_state != 'servo':
                self.get_logger().error(
                    'controller left servo at t=%.1f s (now "%s"); see the controller log for the failing stage'
                    % (elapsed, self.controller_state))
                servo_exited = True
                break

            # 相位从 π 开始：圆心在起点 +radius*x 方向，theta=π 时球正好落在起点。
            theta = math.pi + omega * elapsed
            ball = (center[0] + self.radius * math.cos(theta),
                    center[1] + self.radius * math.sin(theta),
                    center[2])
            feedforward = (-self.radius * omega * math.sin(theta),
                           self.radius * omega * math.cos(theta),
                           0.0)

            try:
                end_effector = self.ee_position()
                consecutive_tf_failures = 0
            except Exception as error:
                tf_failures += 1
                consecutive_tf_failures += 1
                if consecutive_tf_failures == 1:
                    self.get_logger().warning('TF lookup failed: %s' % error)
                if consecutive_tf_failures > self.max_tf_failures:
                    self.get_logger().error(
                        'TF lookup failed %d times in a row; aborting' % consecutive_tf_failures)
                    servo_exited = True
                    break
                self._publish_ball(ball)
                next_tick = self._wait_next_tick(next_tick, period)
                continue

            error = (ball[0] - end_effector[0],
                     ball[1] - end_effector[1],
                     ball[2] - end_effector[2])
            command = (feedforward[0] + self.gain * error[0],
                       feedforward[1] + self.gain * error[1],
                       feedforward[2] + self.gain * error[2])
            speed = math.sqrt(sum(component * component for component in command))
            if speed > self.max_speed:
                command = tuple(component * self.max_speed / speed for component in command)
                speed = self.max_speed

            self._publish_servo(command)
            self._publish_ball(ball)

            distance = math.sqrt(sum(component * component for component in error))
            error_sum += distance
            error_peak = max(error_peak, distance)
            sample_count += 1

            if elapsed >= next_report:
                mean = error_sum / max(1, sample_count)
                self.get_logger().info(
                    't=%5.1f s  err=%5.1f mm  mean=%5.1f mm  peak=%5.1f mm  cmd=%5.1f mm/s'
                    % (elapsed, distance * 1000.0, mean * 1000.0, error_peak * 1000.0, speed * 1000.0))
                next_report += self.report_period

            next_tick = self._wait_next_tick(next_tick, period)

        mean_error = error_sum / sample_count if sample_count else float('inf')
        self.get_logger().info(
            'tracking finished: samples=%d mean=%.2f mm peak=%.2f mm tf_failures=%d'
            % (sample_count, mean_error * 1000.0, error_peak * 1000.0, tf_failures))

        if servo_exited:
            return False
        if sample_count < 0.5 * self.duration * self.command_rate:
            self.get_logger().error('too few samples (%d); the loop did not run at the expected rate' % sample_count)
            return False
        if mean_error > self.arrive_mean or error_peak > self.arrive_peak:
            self.get_logger().error(
                'acceptance failed: mean %.2f mm (limit %.0f mm), peak %.2f mm (limit %.0f mm)'
                % (mean_error * 1000.0, self.arrive_mean * 1000.0,
                   error_peak * 1000.0, self.arrive_peak * 1000.0))
            return False
        self.get_logger().info('acceptance passed: mean %.2f mm, peak %.2f mm'
                               % (mean_error * 1000.0, error_peak * 1000.0))
        return True

    # ----------------------------------------------------------------------
    # 发布与收尾
    # ----------------------------------------------------------------------
    def _wait_next_tick(self, next_tick, period):
        """等到下一个节拍并返回再下一个节拍；落后太多就重新对齐，避免突发发送。"""
        delay = next_tick - time.monotonic()
        if delay > 0.0:
            self.spin_for(delay)
            return next_tick + period
        # 已经晚了：至少处理一次回调，否则状态与 TF 会饿死。
        rclpy.spin_once(self, timeout_sec=0.0)
        if delay < -period:
            return time.monotonic() + period
        return next_tick + period

    def _publish_servo(self, velocity):
        message = ServoCtrlCmd()
        vector = Vector()
        vector.pos = [float(velocity[0]), float(velocity[1]), float(velocity[2]), 0.0, 0.0, 0.0]
        message.velocity = vector
        self.servo_pub.publish(message)

    def _publish_ball(self, position):
        stamp = self.get_clock().now().to_msg()

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.w = 1.0
        self.ball_pose_pub.publish(pose)

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.base_frame
        marker.ns = 'servo_demo'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = self.ball_marker_size
        marker.scale.y = self.ball_marker_size
        marker.scale.z = self.ball_marker_size
        marker.color.r = 1.0
        marker.color.g = 0.55
        marker.color.b = 0.1
        marker.color.a = 0.9

        array = MarkerArray()
        array.markers = [marker]
        self.marker_pub.publish(array)

    def stop(self):
        """发零速度把参考停住，然后请求回到空闲状态。"""
        if self.zero_sent:
            return
        self.zero_sent = True

        if self.in_servo and rclpy.ok():
            period = 1.0 / self.command_rate
            for _ in range(max(1, int(0.5 * self.command_rate))):
                self._publish_servo((0.0, 0.0, 0.0))
                self.spin_for(period)
        if self.in_servo and rclpy.ok():
            self.set_exp_state('idel')
            self.in_servo = False

    def run(self):
        self.get_logger().info(
            'servo track demo: controller=%s frames=%s->%s radius=%.3f m period=%.1f s'
            % (self.controller_node, self.base_frame, self.ee_frame, self.radius, self.period))

        if not self.wait_ready():
            return 1
        if not self.preposition():
            return 1
        if not self.enter_servo():
            return 1
        passed = self.track()
        return 0 if passed else 1


def main(argv=None):
    rclpy.init(args=argv)
    node = ServoTrackDemo()
    exit_code = 1
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        node.get_logger().warning('interrupted by the user')
    finally:
        try:
            node.stop()
        except Exception as error:  # 收尾阶段不允许再抛异常
            node.get_logger().warning('cleanup failed: %s' % error)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
