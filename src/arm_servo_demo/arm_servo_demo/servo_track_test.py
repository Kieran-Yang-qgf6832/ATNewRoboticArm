"""伺服跟踪试验：订阅外部球位姿，末端追踪，并把全过程写成 CSV。

与圆周演示的区别：
  - 球的位置与速度来自外部（键盘节点），不再由本节点生成；
  - 不设活动边界，允许把球推到不可达处，用于测量可达域边界；
  - 每个控制周期写一行 CSV，结束时打印 RMS、峰值与失败点。

流程与圆周演示一致：摆到 ready -> 切入 servo -> 等第一条球消息 -> 50 Hz 追踪
-> 停止并切回 idel。
"""

import csv
import math
import os
import sys
import time

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TwistStamped
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from robot_msgs.msg import JointTrajCmd, ServoCtrlCmd, Vector

STATE_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class ServoTrackTest(Node):
    def __init__(self):
        super().__init__('servo_track_test')

        self.controller_node = self.declare_parameter('controller_node', 'arm_controller').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.ee_frame = self.declare_parameter('ee_frame', 'link6').value
        self.joint_names = list(self.declare_parameter(
            'joints', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']).value)
        self.ready_joints = [float(v) for v in self.declare_parameter(
            'ready_joints', [0.0, 0.3, -0.3, 0.0, 0.3, 0.0]).value]

        self.gain = float(self.declare_parameter('gain', 2.0).value)
        self.max_speed = float(self.declare_parameter('max_speed', 0.08).value)
        self.command_rate = float(self.declare_parameter('command_rate', 50.0).value)
        self.ball_timeout = float(self.declare_parameter('ball_timeout', 1.0).value)
        self.ball_stale = float(self.declare_parameter('ball_stale', 0.1).value)
        self.steady_settle = float(self.declare_parameter('steady_settle', 1.0).value)
        self.preposition_duration = float(self.declare_parameter('preposition_duration', 3.0).value)
        self.reach_tolerance = float(self.declare_parameter('reach_tolerance', 0.02).value)
        self.max_tf_failures = int(self.declare_parameter('max_tf_failures', 20).value)

        self.output_dir = os.path.abspath(self.declare_parameter(
            'output_dir', os.path.join('.ros', 'servo_trials')).value)
        self.steady_speed_min = float(self.declare_parameter('steady_speed_min', 0.055).value)
        self.steady_speed_max = float(self.declare_parameter('steady_speed_max', 0.07).value)
        self.steady_rms_limit = float(self.declare_parameter('steady_rms_limit', 0.003).value)

        self.create_subscription(
            String, '/%s/state' % self.controller_node, self._on_state, STATE_QOS)
        self.create_subscription(JointState, '/joint_states', self._on_joint_state, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/servo_demo/ball_pose', self._on_ball_pose, 10)
        self.create_subscription(TwistStamped, '/servo_demo/ball_velocity', self._on_ball_velocity, 10)

        self.servo_pub = self.create_publisher(ServoCtrlCmd, '/arm_servo', 10)
        self.joint_traj_pub = self.create_publisher(JointTrajCmd, '/arm_joint_traj', 10)
        self.set_param_cli = self.create_client(
            SetParameters, '/%s/set_parameters' % self.controller_node)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.controller_state = ''
        self.joint_positions = {}
        self.joint_velocities = {}
        self.ball_position = None
        self.ball_velocity = (0.0, 0.0, 0.0)
        self.ball_seen_at = None
        self.in_servo = False
        self.stopped = False

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------
    def _on_state(self, msg):
        self.controller_state = msg.data

    def _on_joint_state(self, msg):
        self.joint_positions = dict(zip(msg.name, msg.position))
        self.joint_velocities = dict(zip(msg.name, msg.velocity))

    def _on_ball_pose(self, msg):
        self.ball_position = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
        self.ball_seen_at = time.monotonic()

    def _on_ball_velocity(self, msg):
        self.ball_velocity = (msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z)

    # ------------------------------------------------------------------
    # 通用
    # ------------------------------------------------------------------
    def spin_for(self, seconds):
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

    def joint_velocity_vector(self):
        if not self.joint_velocities:
            return [0.0] * len(self.joint_names)
        return [float(self.joint_velocities.get(name, 0.0)) for name in self.joint_names]

    def max_joint_speed(self):
        if not self.joint_velocities:
            return float('inf')
        return max(abs(value) for value in self.joint_velocities.values())

    def max_ready_residual(self):
        current = self.joint_vector()
        if current is None:
            return float('inf')
        return max(abs(current[i] - self.ready_joints[i]) for i in range(len(self.ready_joints)))

    def ee_position(self):
        transform = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, Time())
        translation = transform.transform.translation
        return (translation.x, translation.y, translation.z)

    def _tf_available(self):
        try:
            self.ee_position()
            return True
        except Exception:
            return False

    # ------------------------------------------------------------------
    # 流程
    # ------------------------------------------------------------------
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
        self.get_logger().info('preposition: moving to ready in %.1f s' % self.preposition_duration)

        reached = self.wait_for(lambda: self.max_ready_residual() <= self.reach_tolerance,
                                self.preposition_duration + 3.0, 'ready configuration')
        if not reached:
            self.get_logger().error('ready residual is %.4f rad' % self.max_ready_residual())
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

    def track(self, writer, handle):
        if not self.wait_for(lambda: self.ball_position is not None, 20.0, 'the first ball message'):
            return False, 'no ball message'

        self.get_logger().info('tracking: gain=%.1f /s max_speed=%.1f cm/s'
                               % (self.gain, self.max_speed * 100.0))

        period = 1.0 / self.command_rate
        started_at = time.monotonic()
        next_tick = started_at
        next_report = 2.0

        error_sum = 0.0
        error_square_sum = 0.0
        error_peak = 0.0
        samples = 0
        steady_square_sum = 0.0
        steady_peak = 0.0
        steady_samples = 0
        steady_since = None
        tf_failures = 0
        consecutive_tf_failures = 0
        failure = None
        last_command = (0.0, 0.0, 0.0)
        last_ee = None

        while rclpy.ok():
            now = time.monotonic()
            elapsed = now - started_at

            if self.controller_state != 'servo':
                failure = dict(reason='controller left servo (now "%s")' % self.controller_state,
                               time=elapsed, ball=self.ball_position, ee=last_ee)
                break

            if self.ball_seen_at is None or now - self.ball_seen_at > self.ball_timeout:
                self.get_logger().info('ball stopped publishing for %.1f s; ending the trial' % self.ball_timeout)
                break

            try:
                end_effector = self.ee_position()
                consecutive_tf_failures = 0
            except Exception as error:
                tf_failures += 1
                consecutive_tf_failures += 1
                if consecutive_tf_failures == 1:
                    self.get_logger().warning('TF lookup failed: %s' % error)
                if consecutive_tf_failures > self.max_tf_failures:
                    failure = dict(reason='TF lookup failed %d times in a row' % consecutive_tf_failures,
                                   time=elapsed, ball=self.ball_position, ee=None, error=None)
                    break
                next_tick = self._wait_next_tick(next_tick, period)
                continue

            last_ee = end_effector
            ball = self.ball_position
            ball_velocity = self.ball_velocity
            if now - self.ball_seen_at > self.ball_stale:
                # 发布端卡住或刚退出时，最后一条速度可能是旧的；过期的速度不能当前馈用。
                ball_velocity = (0.0, 0.0, 0.0)
            error = (ball[0] - end_effector[0], ball[1] - end_effector[1], ball[2] - end_effector[2])
            command = (ball_velocity[0] + self.gain * error[0],
                       ball_velocity[1] + self.gain * error[1],
                       ball_velocity[2] + self.gain * error[2])
            speed = math.sqrt(sum(value * value for value in command))
            if speed > self.max_speed:
                command = tuple(value * self.max_speed / speed for value in command)
                speed = self.max_speed
            last_command = command

            self._publish_servo(command)

            distance = math.sqrt(sum(value * value for value in error))
            error_sum += distance
            error_square_sum += distance * distance
            error_peak = max(error_peak, distance)
            samples += 1

            ball_speed = math.sqrt(sum(value * value for value in ball_velocity))
            in_band = (self.steady_speed_min <= ball_speed <= self.steady_speed_max
                       and speed < 0.9 * self.max_speed)
            if in_band:
                if steady_since is None:
                    steady_since = elapsed
                # 只统计进入等速段 steady_settle 秒之后的样本：起步追赶不算稳态。
                if elapsed - steady_since >= self.steady_settle:
                    steady_square_sum += distance * distance
                    steady_peak = max(steady_peak, distance)
                    steady_samples += 1
            else:
                steady_since = None

            joints = self.joint_vector() or [float('nan')] * len(self.joint_names)
            joint_rates = self.joint_velocity_vector()
            writer.writerow([
                '%.4f' % elapsed,
                '%.6f' % ball[0], '%.6f' % ball[1], '%.6f' % ball[2],
                '%.6f' % ball_velocity[0], '%.6f' % ball_velocity[1], '%.6f' % ball_velocity[2],
                '%.6f' % end_effector[0], '%.6f' % end_effector[1], '%.6f' % end_effector[2],
                '%.6f' % distance,
                '%.6f' % command[0], '%.6f' % command[1], '%.6f' % command[2], '%.6f' % speed,
            ] + ['%.6f' % value for value in joints] + ['%.6f' % value for value in joint_rates]
              + [self.controller_state])
            if samples % 50 == 0:
                handle.flush()

            if elapsed >= next_report:
                mean = error_sum / max(1, samples)
                self.get_logger().info(
                    't=%5.1f s  err=%5.1f mm  mean=%5.1f mm  peak=%5.1f mm  |v_ball|=%.1f cm/s  |cmd|=%.1f cm/s'
                    % (elapsed, distance * 1000.0, mean * 1000.0, error_peak * 1000.0,
                       ball_speed * 100.0, speed * 100.0))
                next_report += 2.0

            next_tick = self._wait_next_tick(next_tick, period)

        mean_error = error_sum / samples if samples else float('inf')
        rms = math.sqrt(error_square_sum / samples) if samples else float('inf')
        steady_rms = math.sqrt(steady_square_sum / steady_samples) if steady_samples else float('inf')
        self.get_logger().info(
            'trial finished: samples=%d mean=%.2f mm rms=%.2f mm peak=%.2f mm '
            'steady_samples=%d steady_rms=%.2f mm steady_peak=%.2f mm tf_failures=%d'
            % (samples, mean_error * 1000.0, rms * 1000.0, error_peak * 1000.0,
               steady_samples, steady_rms * 1000.0, steady_peak * 1000.0, tf_failures))

        if failure is not None:
            self.get_logger().error('FAILURE at t=%.2f s: %s' % (failure['time'], failure['reason']))
            if failure['ball'] is not None:
                self.get_logger().error('  ball at (% .4f, % .4f, % .4f)'
                                        % (failure['ball'][0], failure['ball'][1], failure['ball'][2]))
            if failure['ee'] is not None:
                self.get_logger().error('  end effector at (% .4f, % .4f, % .4f), '
                                        'last command (% .4f, % .4f, % .4f)'
                                        % (failure['ee'][0], failure['ee'][1], failure['ee'][2],
                                           last_command[0], last_command[1], last_command[2]))
            return False, failure['reason']

        if steady_samples >= 100:
            if steady_rms > self.steady_rms_limit:
                self.get_logger().error('steady RMS %.2f mm exceeds the %.1f mm limit'
                                        % (steady_rms * 1000.0, self.steady_rms_limit * 1000.0))
                return False, 'steady RMS'
            self.get_logger().info('steady RMS passed: %.2f mm over %d samples'
                                   % (steady_rms * 1000.0, steady_samples))
        else:
            self.get_logger().warning('only %d steady samples; the RMS check was skipped' % steady_samples)
        return True, None

    def _wait_next_tick(self, next_tick, period):
        delay = next_tick - time.monotonic()
        if delay > 0.0:
            self.spin_for(delay)
            return next_tick + period
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

    def stop(self):
        if self.stopped:
            return
        self.stopped = True
        if self.in_servo and rclpy.ok():
            period = 1.0 / self.command_rate
            for _ in range(max(1, int(0.5 * self.command_rate))):
                self._publish_servo((0.0, 0.0, 0.0))
                self.spin_for(period)
        if self.in_servo and rclpy.ok():
            self.set_exp_state('idel')
            self.in_servo = False

    def run(self):
        if not self.wait_for(lambda: bool(self.joint_positions), 20.0, '/joint_states'):
            return 1
        # 仿真启动初期机械臂会从初始位形塌到限位，必须等它静止再摆位，
        # 否则关节轨迹的起点会取在剧烈运动的中途。
        if not self.wait_for(lambda: self.max_joint_speed() < 0.1, 15.0,
                             'the arm to settle before the preposition move'):
            return 1
        self.spin_for(0.5)
        if not self.wait_for(self._tf_available, 20.0, 'end-effector TF'):
            return 1
        if not self.preposition():
            return 1
        if not self.enter_servo():
            return 1

        os.makedirs(self.output_dir, exist_ok=True)
        path = os.path.join(self.output_dir,
                            'servo_kb_%s.csv' % time.strftime('%Y%m%d-%H%M%S'))
        self.get_logger().info('writing trial data to %s' % path)

        header = ['t', 'ball_x', 'ball_y', 'ball_z', 'ball_vx', 'ball_vy', 'ball_vz',
                  'ee_x', 'ee_y', 'ee_z', 'error',
                  'cmd_x', 'cmd_y', 'cmd_z', 'cmd_norm'] \
            + ['q%d' % (i + 1) for i in range(len(self.joint_names))] \
            + ['qd%d' % (i + 1) for i in range(len(self.joint_names))] + ['state']

        with open(path, 'w', newline='', encoding='utf-8') as handle:
            writer = csv.writer(handle)
            writer.writerow(header)
            ok, reason = self.track(writer, handle)
            handle.flush()
        self.get_logger().info('trial data saved to %s' % path)
        return 0 if ok else 1


def main(argv=None):
    rclpy.init(args=argv)
    node = ServoTrackTest()
    exit_code = 1
    try:
        exit_code = node.run()
    except KeyboardInterrupt:
        node.get_logger().warning('interrupted by the user')
    finally:
        try:
            node.stop()
        except Exception as error:
            node.get_logger().warning('cleanup failed: %s' % error)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
