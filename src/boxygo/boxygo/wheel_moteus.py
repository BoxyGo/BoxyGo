#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from moteus_msgs.msg import PositionCommand
from sensor_msgs.msg import JointState

class WheelMoteus(Node):
    def __init__(self):
        super().__init__('wheel_moteus')

        # Parametry: ID , nazwa jointa i użycie velocity/position
        self.declare_parameter('servo_id', 1)
        self.declare_parameter('joint_name', 'left_wheel_1_joint')
        self.declare_parameter('use_velocity', True)

        servo_id = self.get_parameter('servo_id').value
        self.joint_name = self.get_parameter('joint_name').value
        self.use_velocity = self.get_parameter('use_velocity').value

        # Publisher jednego ID
        moteus_topic = f'/boxygo_moteus/id_{servo_id}/cmd_position'
        self._pub = self.create_publisher(PositionCommand, moteus_topic, 10)

        # Subskrypcja joint_states
        self._joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_callback,
            10
        )

        # Przechowywana ostatnia wartość stawu (vel lub pos)
        self._joint_value = None

        # Timer 50 Hz ⇒ 0.02 s
        timer_period = 1.0 / 50.0
        self.create_timer(timer_period, self._publish_cmd)

        self.get_logger().info(
            f"Ready: servo={servo_id}, joint='{self.joint_name}', use_velocity={self.use_velocity} @50Hz"
        )

    def _joint_states_callback(self, msg: JointState):
        if self.joint_name in msg.name:
            idx = msg.name.index(self.joint_name)
            if self.use_velocity:
                if idx < len(msg.velocity):
                    self._joint_value = msg.velocity[idx]
                else:
                    self.get_logger().warn(
                        f"No velocity data for joint '{self.joint_name}'"
                    )
            else:
                if idx < len(msg.position):
                    self._joint_value = msg.position[idx]
                else:
                    self.get_logger().warn(
                        f"No position data for joint '{self.joint_name}'"
                    )
        else:
            self.get_logger().debug(
                f"Joint '{self.joint_name}' not in /joint_states message"
            )

    def _publish_cmd(self):
        # Jeśli brak danych ze stawu, pomiń
        if self._joint_value is None:
            self.get_logger().debug('Brak danych stawu, pomijam publikację')
            return

        cmd = PositionCommand()
        cmd.position = [math.nan]
        cmd.velocity = [self._joint_value]
        self._pub.publish(cmd)
        self.get_logger().debug(
            f"→ position=NaN, velocity={self._joint_value}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = WheelMoteus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
