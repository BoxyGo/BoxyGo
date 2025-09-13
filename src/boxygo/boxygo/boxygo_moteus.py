#!/usr/bin/env python3

import math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from moteus_msgs.msg import PositionCommand

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
        self.declare_parameter('loop_rate_hz', 100.0)        # wysyłanie do sterowników
        self.declare_parameter('timeout_ms', 300)            # watchdog: brak komend => STOP
        # (opcjonalnie) twarda saturacja prędkości koła
        self.declare_parameter('max_rpm', 0.0)               # 0 = wyłączona (brak saturacji)

        self.servo_ids = self.get_parameter('servo_ids').value
        self.joint_names = self.get_parameter('joint_names').value
        self.r = float(self.get_parameter('wheel_radius').value)
        self.b = float(self.get_parameter('wheel_separation').value)
        self.cmd_topic = self.get_parameter('cmd_topic').value
        self.loop_rate = float(self.get_parameter('loop_rate_hz').value)
        self.timeout_ms = int(self.get_parameter('timeout_ms').value)
        self.max_rpm = float(self.get_parameter('max_rpm').value)
        self.max_radps = (self.max_rpm * 2.0 * math.pi / 60.0) if self.max_rpm > 0.0 else None

        if len(self.servo_ids) != len(self.joint_names):
            raise ValueError("servo_ids i joint_names muszą mieć tę samą długość")

        self.declare_parameter('dir_factors', [1, 1, 1, -1, -1, -1])  # domyślnie odwrócona prawa strona
        self.dir_factors = self.get_parameter('dir_factors').value
        self.dir_for_id = {sid: float(df) for sid, df in zip(self.servo_ids, self.dir_factors)}
            

        # mapowanie side i publishery
        self.side_for_id, self.pubs = {}, {}
        for sid, jn in zip(self.servo_ids, self.joint_names):
            topic = f'/boxygo_moteus/id_{sid}/cmd_position'
            self.pubs[sid] = self.create_publisher(PositionCommand, topic, 10)
            self.side_for_id[sid] = 'left' if 'left' in jn else ('right' if 'right' in jn else 'left')

        # aktualne cele prędkości kół [rad/s]
        self.w_target = {'left': 0.0, 'right': 0.0}
        self.last_cmd_time = time.monotonic()

        # subskrypcja TwistStamped
        self.create_subscription(TwistStamped, self.cmd_topic, self._cmd_cb, 10)

        # timer wysyłki do sterowników
        self.dt = 1.0 / self.loop_rate
        self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            f"Using {self.cmd_topic} (TwistStamped), timeout={self.timeout_ms}ms, loop={self.loop_rate}Hz"
        )

    def _cmd_cb(self, msg: TwistStamped):
        v = float(msg.twist.linear.x)   # [m/s]
        w = float(msg.twist.angular.z)  # [rad/s]

        # kinematyka: ωr/ωl [rad/s]
        w_r = (v + w * self.b * 0.5) / self.r
        w_l = (v - w * self.b * 0.5) / self.r

        # opcjonalna saturacja
        if self.max_radps is not None:
            w_r = max(-self.max_radps, min(self.max_radps, w_r))
            w_l = max(-self.max_radps, min(self.max_radps, w_l))

        self.w_target['right'] = w_r
        self.w_target['left']  = w_l
        self.last_cmd_time = time.monotonic()

    def _tick(self):
        # watchdog: brak komend => STOP (0 rad/s)
        if (time.monotonic() - self.last_cmd_time) * 1000.0 > self.timeout_ms:
            w_left = 0.0
            w_right = 0.0
        else:
            w_left = self.w_target['left']
            w_right = self.w_target['right']

        # publikacja do wszystkich serw
        for sid in self.servo_ids:
            side = self.side_for_id[sid]
            w = w_left if side == 'left' else w_right
            w *= self.dir_for_id[sid]
            msg = PositionCommand()
            msg.position = [math.nan]   # velocity-only
            msg.velocity = [w]          # [rad/s]
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
