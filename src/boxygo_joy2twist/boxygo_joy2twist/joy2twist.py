#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class Joy2Twist(Node):
    def __init__(self):
        super().__init__('joy2twist')

        # <<< SKONFIGURUJ TUTAJ NUMERY OSI I PRZYCISKU ENABLE >>>
        self.axis_linear = 1        # Lewy drążek przód/tył (zwykle axis 1)
        self.axis_angular = 0       # Lewy drążek lewo/prawo (zwykle axis 0)
        self.scale_linear = 0.1
        self.scale_angular = 0.15
        self.enable_button = 4      # L1 lub inny wygodny przycisk (jstest-gtk!)
        self.require_enable = True  # Ustaw False, by nie wymagać przycisku

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/diff_cont/cmd_vel_unstamped', 10)




    def joy_callback(self, msg):
        # Jeżeli require_enable: sprawdzamy przycisk enable
        if not self.require_enable or (self.enable_button < len(msg.buttons) and msg.buttons[self.enable_button]):
            twist = Twist()
            if self.axis_linear < len(msg.axes):
                twist.linear.x = msg.axes[self.axis_linear] * self.scale_linear
            if self.axis_angular < len(msg.axes):
                twist.angular.z = msg.axes[self.axis_angular] * self.scale_angular
            self.cmd_pub.publish(twist)
        else:
            # Wypisz zerowe cmd_vel, żeby robot się zatrzymał po puszczeniu enable
            twist = Twist()
            self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = Joy2Twist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
