#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from copy import deepcopy

class RealsenseImuRemap(Node):
    def __init__(self):
        super().__init__("realsense_imu_remap")

        self.input_topic  = self.declare_parameter("input_topic", "/camera/imu").value
        self.output_topic = self.declare_parameter("output_topic", "/imu/corrected").value

        self.sub = self.create_subscription(
            Imu,
            self.input_topic,
            self.imu_callback,
            10
        )

        self.pub = self.create_publisher(Imu, self.output_topic, 10)

    def imu_callback(self, msg_in: Imu):

        msg_out = deepcopy(msg_in)

        ax = msg_in.linear_acceleration.x
        ay = msg_in.linear_acceleration.y
        az = msg_in.linear_acceleration.z

        msg_out.linear_acceleration.x =  az      # X_bl
        msg_out.linear_acceleration.y = -ax      # Y_bl
        msg_out.linear_acceleration.z = -ay      # Z_bl

        gx = msg_in.angular_velocity.x
        gy = msg_in.angular_velocity.y
        gz = msg_in.angular_velocity.z

        msg_out.angular_velocity.x =  gz         # X_bl
        msg_out.angular_velocity.y = -gx         # Y_bl
        msg_out.angular_velocity.z = -gy         # Z_bl

        self.pub.publish(msg_out)

def main():
    rclpy.init()
    node = RealsenseImuRemap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
