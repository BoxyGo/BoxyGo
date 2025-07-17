#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from moteus_msgs.msg import PositionCommand, ControllerState


class FakeMoteus(Node):
    def __init__(self):
        super().__init__('moteus_node')
        self.declare_parameter('id', 1)

        self.id = self.get_parameter('id').value
        self.namespace = f"/moteus/id_{self.id}"

        self.position = 0.0

        self.cmd_pos_sub = self.create_subscription(
            PositionCommand,
            f"{self.namespace}/cmd_position",
            self.cmd_position_cb,
            10)

        self.cmd_stop_sub = self.create_subscription(
            Empty,
            f"{self.namespace}/cmd_stop",
            self.cmd_stop_cb,
            10)

        self.state_pub = self.create_publisher(
            ControllerState,
            f"{self.namespace}/state",
            10)

        self.timer = self.create_timer(0.02, self.publish_state)

        self.get_logger().info(f"[SIM] Fake Moteus node ID {self.id} started")

    def cmd_position_cb(self, msg):
        if msg.position:
            self.position = msg.position[0]

    def cmd_stop_cb(self, msg):
        self.get_logger().info("[SIM] Stop command received")
        self.position = 0.0

    def publish_state(self):
        msg = ControllerState()
        msg.position = self.position
        msg.velocity = 0.0
        msg.torque = 0.0
        self.state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeMoteus()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
