#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time

class Joy2Twist(Node):
    def __init__(self):
        super().__init__('joy2twist')

        # ---- PARAMETRY Z YAML ----
        self.declare_parameter('axis_linear', 1)
        self.declare_parameter('axis_angular', 0)

        self.declare_parameter('scale_linear', 0.2)
        self.declare_parameter('scale_angular', 0.4)

    
        self.declare_parameter('enable_drive_button', 4)   # L1
        self.declare_parameter('enable_config_button', 5)  # R1

        self.declare_parameter('require_enable_button', True)

        self.declare_parameter('inc_step_linear', 0.05)
        self.declare_parameter('inc_step_angular', 0.05)

        self.declare_parameter('min_linear', 0.0)
        self.declare_parameter('max_linear', 2.0)
        self.declare_parameter('min_angular', 0.0)
        self.declare_parameter('max_angular', 4.0)

        self.declare_parameter('joy_timeout', 0.5)

        # ---- Wczytaj parametry ----
        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_angular = self.get_parameter('axis_angular').value

        self.scale_linear = self.get_parameter('scale_linear').value
        self.scale_angular = self.get_parameter('scale_angular').value

        self.enable_drive_button = self.get_parameter('enable_drive_button').value
        self.enable_config_button = self.get_parameter('enable_config_button').value
        self.require_enable = self.get_parameter('require_enable_button').value

        self.inc_lin = self.get_parameter('inc_step_linear').value
        self.inc_ang = self.get_parameter('inc_step_angular').value

        self.min_lin = self.get_parameter('min_linear').value
        self.max_lin = self.get_parameter('max_linear').value
        self.min_ang = self.get_parameter('min_angular').value
        self.max_ang = self.get_parameter('max_angular').value

        self.joy_timeout = self.get_parameter('joy_timeout').value

        # ---- Przypisanie przycisków PlayStation ----
        self.btn_triangle = 2
        self.btn_circle = 1
        self.btn_cross = 0
        self.btn_square = 3

        self.last_joy_time = time.time()

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)

        self.create_timer(0.1, self.watchdog)

    def watchdog(self):
        if time.time() - self.last_joy_time > self.joy_timeout:
            self.cmd_pub.publish(Twist())

    def joy_callback(self, msg):
        self.last_joy_time = time.time()

        # --- (1) Enable do konfiguracji (R1) ---
        config_enabled = (
            self.enable_config_button < len(msg.buttons)
            and msg.buttons[self.enable_config_button] == 1
        )

        if config_enabled:
            # regulacja liniowej
            if msg.buttons[self.btn_triangle]:
                self.scale_linear = min(self.max_lin, self.scale_linear + self.inc_lin)

            if msg.buttons[self.btn_cross]:
                self.scale_linear = max(self.min_lin, self.scale_linear - self.inc_lin)

            # regulacja kątowej
            if msg.buttons[self.btn_circle]:
                self.scale_angular = min(self.max_ang, self.scale_angular + self.inc_ang)

            if msg.buttons[self.btn_square]:
                self.scale_angular = max(self.min_ang, self.scale_angular - self.inc_ang)

        # --- (2) Enable do sterowania robotem (L1) ---
        drive_enabled = (
            not self.require_enable or
            (self.enable_drive_button < len(msg.buttons)
             and msg.buttons[self.enable_drive_button] == 1)
        )

        twist = Twist()

        if drive_enabled:
            if self.axis_linear < len(msg.axes):
                twist.linear.x = msg.axes[self.axis_linear] * self.scale_linear

            if self.axis_angular < len(msg.axes):
                twist.angular.z = msg.axes[self.axis_angular] * self.scale_angular

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = Joy2Twist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
