#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist            # ⬅️ tu zmiana
from moteus_msgs.msg import PositionCommand
from rcl_interfaces.msg import SetParametersResult

class BoxyGoMoteus(Node):
    def __init__(self):
        super().__init__('boxygo_moteus')

        # --- Parametry ---
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('joint_names', [
            'left_wheel_1_joint',
            'left_wheel_2_joint',
            'left_wheel_3_joint',
            'right_wheel_1_joint',
            'right_wheel_2_joint',
            'right_wheel_3_joint'
        ])
        self.declare_parameter('wheel_radius', 0.085)        # [m]
        self.declare_parameter('wheel_separation', 0.55)     # [m]
        self.declare_parameter('cmd_topic', '/diff_cont/cmd_vel_out')
        self.declare_parameter('loop_rate_hz', 100.0)
        self.declare_parameter('timeout_ms', 300)
        self.declare_parameter('max_rpm', 0.0)
        self.declare_parameter('dir_factors', [1, 1, 1, -1, -1, -1])
        self.declare_parameter('enabled', True)

        self.servo_ids = self.get_parameter('servo_ids').value
        self.joint_names = self.get_parameter('joint_names').value
        self.r = float(self.get_parameter('wheel_radius').value)
        self.b = float(self.get_parameter('wheel_separation').value)
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.loop_rate = float(self.get_parameter('loop_rate_hz').value)
        self.timeout_ms = int(self.get_parameter('timeout_ms').value)
        self.max_rpm = float(self.get_parameter('max_rpm').value)
        self.max_radps = (self.max_rpm * 2.0 * math.pi / 60.0) if self.max_rpm > 0.0 else None
        self.dir_factors = self.get_parameter('dir_factors').value
        self.dir_for_id = {sid: float(df) for sid, df in zip(self.servo_ids, self.dir_factors)}
        self.enabled = bool(self.get_parameter('enabled').value)

        if len(self.servo_ids) != len(self.joint_names):
            raise ValueError("servo_ids i joint_names muszą mieć tę samą długość")

        self.add_on_set_parameters_callback(self._on_set_params)

        self.side_for_id, self.pubs = {}, {}
        for sid, jn in zip(self.servo_ids, self.joint_names):
            topic = f'/boxygo_moteus/id_{sid}/cmd_position'
            self.pubs[sid] = self.create_publisher(PositionCommand, topic, 10)
            self.side_for_id[sid] = 'left' if 'left' in jn else ('right' if 'right' in jn else 'left')

        self.w_target = {'left': 0.0, 'right': 0.0}
        self.last_cmd_time = time.monotonic()

        # ⬅️ subskrypcja Twist (nie TwistStamped)
        self.create_subscription(Twist, self.cmd_topic, self._cmd_cb, 10)

        self.dt = 1.0 / self.loop_rate
        self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            f"Using {self.cmd_topic} (Twist), timeout={self.timeout_ms}ms, loop={self.loop_rate}Hz, enabled={self.enabled}"
        )

    def _on_set_params(self, params):
        for p in params:
            if p.name == 'enabled':
                self.enabled = bool(p.value)
                self.get_logger().warn(f"[boxygo_moteus] enabled={self.enabled}")
        return SetParametersResult(successful=True)

    # ⬅️ przyjmuje Twist
    def _cmd_cb(self, msg: Twist):
        v = float(msg.linear.x)   # [m/s]
        w = float(msg.angular.z)  # [rad/s]

        w_r = (v + w * self.b * 0.5) / self.r
        w_l = (v - w * self.b * 0.5) / self.r

        if self.max_radps is not None:
            w_r = max(-self.max_radps, min(self.max_radps, w_r))
            w_l = max(-self.max_radps, min(self.max_radps, w_l))

        self.w_target['right'] = w_r
        self.w_target['left']  = w_l
        self.last_cmd_time = time.monotonic()

    def _tick(self):
        if not self.enabled:
            return

        if (time.monotonic() - self.last_cmd_time) * 1000.0 > self.timeout_ms:
            w_left = 0.0
            w_right = 0.0
        else:
            w_left = self.w_target['left']
            w_right = self.w_target['right']

        for sid in self.servo_ids:
            side = self.side_for_id[sid]
            w = w_left if side == 'left' else w_right
            w *= self.dir_for_id[sid]
            msg = PositionCommand()
            msg.position = [math.nan]
            msg.velocity = [w]
            self.pubs[sid].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BoxyGoMoteus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
