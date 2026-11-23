"""键盘控制虚拟球。

在终端里按住按键移动球（基座系方向），松开后按受限加速度减速停下：

    W/S  基座系 ±x
    A/D  基座系 ±y
    E/Q  基座系 ±z
    X    结束试验（也可用 Ctrl-C）

球的速度先按 key_accel 斜坡逼近按键给定的目标速度，再积分成位置；位姿与速度
一起发布，追踪节点不需要自己差分。终端没有“松开”事件，这里用 release_timeout
内是否还收到该键的自动重复来判断。

进入 servo 状态后，节点读一次末端 TF，把球放在末端当前位置，保证开局零误差。
"""

import math
import os
import select
import sys
import termios
import time
import tty

import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

KEY_DIRECTIONS = {
    'w': (1.0, 0.0, 0.0),
    's': (-1.0, 0.0, 0.0),
    'a': (0.0, 1.0, 0.0),
    'd': (0.0, -1.0, 0.0),
    'e': (0.0, 0.0, 1.0),
    'q': (0.0, 0.0, -1.0),
}
QUIT_KEYS = {'x', '\x03'}  # x / Ctrl-C（终端为 cbreak 时 Ctrl-C 仍是信号，保留兜底）

STATE_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class BallModel:
    """球的状态：速度受限斜坡 + 位置积分。"""

    def __init__(self, position, key_speed, accel):
        self.position = [float(value) for value in position]
        self.velocity = [0.0, 0.0, 0.0]
        self.key_speed = float(key_speed)
        self.accel = float(accel)

    def update(self, dt, direction):
        """direction 是各按键单位向量之和；不归一化，双键自然得到更大的速度。"""
        target = [direction[i] * self.key_speed for i in range(3)]
        delta = [target[i] - self.velocity[i] for i in range(3)]
        norm = math.sqrt(sum(value * value for value in delta))
        max_step = self.accel * dt
        if norm <= max_step or norm == 0.0:
            self.velocity = target
        else:
            scale = max_step / norm
            self.velocity = [self.velocity[i] + delta[i] * scale for i in range(3)]
        for i in range(3):
            self.position[i] += self.velocity[i] * dt


class BallKeyboard(Node):
    def __init__(self):
        super().__init__('ball_keyboard')

        self.controller_node = self.declare_parameter('controller_node', 'arm_controller').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        self.ee_frame = self.declare_parameter('ee_frame', 'link6').value
        self.key_speed = float(self.declare_parameter('key_speed', 0.06).value)
        self.key_accel = float(self.declare_parameter('key_accel', 0.30).value)
        self.release_timeout = float(self.declare_parameter('release_timeout', 0.15).value)
        self.publish_rate = float(self.declare_parameter('publish_rate', 50.0).value)
        self.max_duration = float(self.declare_parameter('max_duration', 120.0).value)
        self.ball_marker_size = float(self.declare_parameter('ball_marker_size', 0.04).value)

        self.create_subscription(
            String, '/%s/state' % self.controller_node, self._on_state, STATE_QOS)

        self.pose_pub = self.create_publisher(PoseStamped, '/servo_demo/ball_pose', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, '/servo_demo/ball_velocity', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.controller_state = ''
        self.model = None
        self.last_seen = {}
        self.terminal_settings = None

    def _on_state(self, msg):
        self.controller_state = msg.data

    # ------------------------------------------------------------------
    # 终端
    # ------------------------------------------------------------------
    def enter_terminal(self):
        if not sys.stdin.isatty():
            self.get_logger().error(
                'stdin is not a TTY; run this node in its own terminal with `ros2 run`')
            return False
        self.terminal_settings = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        return True

    def restore_terminal(self):
        if self.terminal_settings is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.terminal_settings)
            self.terminal_settings = None

    def read_keys(self):
        """把当前可读的按键读空，返回按下的字符列表。"""
        pressed = []
        while True:
            try:
                ready, _, _ = select.select([sys.stdin], [], [], 0.0)
            except (OSError, ValueError):
                return pressed
            if not ready:
                return pressed
            char = os.read(sys.stdin.fileno(), 1)
            if not char:
                return pressed
            pressed.append(char.decode('utf-8', errors='ignore').lower())

    # ------------------------------------------------------------------
    # 流程
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
        return predicate()

    def ee_position(self):
        transform = self.tf_buffer.lookup_transform(self.base_frame, self.ee_frame, Time())
        translation = transform.transform.translation
        return (translation.x, translation.y, translation.z)

    def wait_ready(self):
        if not self.wait_for(lambda: self.controller_state == 'servo', 30.0, 'state servo'):
            return False
        self.spin_for(0.2)
        return self.wait_for(self._tf_available, 10.0, 'end-effector TF')

    def _tf_available(self):
        try:
            self.ee_position()
            return True
        except Exception:
            return False

    def run(self):
        self.get_logger().info('waiting for the controller to enter servo ...')
        if not self.wait_ready():
            return 1

        start = self.ee_position()
        self.model = BallModel(start, self.key_speed, self.key_accel)
        self.get_logger().info(
            'ball starts at (%.4f, %.4f, %.4f); key speed %.1f cm/s, accel %.1f cm/s^2, release timeout %.0f ms'
            % (start[0], start[1], start[2], self.key_speed * 100.0, self.key_accel * 100.0,
               self.release_timeout * 1000.0))
        self.get_logger().info('keys: W/S +-x, A/D +-y, E/Q +-z, X quit')

        period = 1.0 / self.publish_rate
        started_at = time.monotonic()
        last_tick = time.monotonic()
        next_tick = last_tick
        next_report = 2.0

        while rclpy.ok():
            now = time.monotonic()
            elapsed = now - started_at
            if elapsed >= self.max_duration:
                self.get_logger().warning('max duration %.0f s reached' % self.max_duration)
                break

            pressed = self.read_keys()
            for char in pressed:
                if char in QUIT_KEYS:
                    self.get_logger().info('quit key pressed')
                    self.stop_ball()
                    return 0
                if char in KEY_DIRECTIONS:
                    self.last_seen[char] = now

            active = [key for key in KEY_DIRECTIONS if now - self.last_seen.get(key, -1e9) <= self.release_timeout]
            direction = [0.0, 0.0, 0.0]
            for key in active:
                for i in range(3):
                    direction[i] += KEY_DIRECTIONS[key][i]

            dt = max(0.0, now - last_tick)
            last_tick = now
            self.model.update(dt, direction)
            self.publish_ball()

            if elapsed >= next_report:
                speed = math.sqrt(sum(value * value for value in self.model.velocity)) * 100.0
                self.get_logger().info(
                    't=%5.1f s  ball=(%6.3f, %6.3f, %6.3f)  |v|=%.1f cm/s  keys=%s'
                    % (elapsed, self.model.position[0], self.model.position[1], self.model.position[2],
                       speed, ''.join(sorted(active)) or '-'))
                next_report += 2.0

            if self.controller_state != 'servo':
                self.get_logger().warning('controller left servo; stopping the ball')
                self.stop_ball()
                return 1

            next_tick += period
            delay = next_tick - time.monotonic()
            if delay > 0.0:
                self.spin_for(delay)
            else:
                rclpy.spin_once(self, timeout_sec=0.0)
                if delay < -period:
                    next_tick = time.monotonic() + period

        self.stop_ball()
        return 0

    def stop_ball(self):
        """收尾时必须把速度清零再发最后一帧，否则追踪侧会拿着旧的速度前馈继续推。"""
        if self.model is not None:
            self.model.velocity = [0.0, 0.0, 0.0]
        self.publish_ball()

    # ------------------------------------------------------------------
    # 发布
    # ------------------------------------------------------------------
    def publish_ball(self):
        if self.model is None:
            return
        stamp = self.get_clock().now().to_msg()
        position = self.model.position
        velocity = self.model.velocity

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        twist = TwistStamped()
        twist.header.stamp = stamp
        twist.header.frame_id = self.base_frame
        twist.twist.linear.x = velocity[0]
        twist.twist.linear.y = velocity[1]
        twist.twist.linear.z = velocity[2]
        self.velocity_pub.publish(twist)

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


def main(argv=None):
    rclpy.init(args=argv)
    node = BallKeyboard()
    exit_code = 1
    try:
        if node.enter_terminal():
            exit_code = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == '__main__':
    sys.exit(main())
