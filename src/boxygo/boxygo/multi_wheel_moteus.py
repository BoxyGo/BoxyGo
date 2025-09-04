#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from moteus_msgs.msg import PositionCommand
from sensor_msgs.msg import JointState

class MultiWheelMoteus(Node):
    def __init__(self):
        super().__init__('multi_wheel_moteus')

        # Parametry: ID , nazwa jointa i użycie velocity/position
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6])
        self.declare_parameter('joint_names', [
            'left_wheel_1_joint',
            'left_wheel_2_joint',
            'left_wheel_3_joint',
            'right_wheel_1_joint',
            'right_wheel_2_joint',
            'right_wheel_3_joint'
        ])
        self.declare_parameter('use_velocity', True)
        self.use_velocity = self.get_parameter('use_velocity').value
        servo_ids = self.get_parameter('servo_ids').value
        joint_names = self.get_parameter('joint_names').value

        if len(servo_ids) != len(joint_names):
            raise ValueError("servo_ids i joint_names muszą mieć tę samą długość")

        # Publisher dla każdego ID
        self._pubs = {}
        self._joint_for_id = {}
        self._value_for_joint = {}

        for sid, jn in zip(servo_ids, joint_names):
            topic = f'/boxygo_moteus/id_{sid}/cmd_position'
            self._pubs[sid] = self.create_publisher(PositionCommand, topic, 10)
            self._joint_for_id[sid] = jn
            self._value_for_joint[jn] = None

        # Subskrypcja joint_states
        self._joint_sub = self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)

        # Timer 50Hz ⇒ 0.02 s
        self.create_timer(1.0/50.0, self._tick)

        self.get_logger().info(
            f"Ready: ids={servo_ids}, joints={joint_names}, use_velocity={self.use_velocity} @50Hz"
        )

    def _joint_cb(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        for jn in self._value_for_joint.keys():
            idx = name_to_idx.get(jn, None)
            if idx is None:
                continue
            if self.use_velocity:
                if idx < len(msg.velocity):
                    self._value_for_joint[jn] = msg.velocity[idx]
            else:
                if idx < len(msg.position):
                    self._value_for_joint[jn] = msg.position[idx]

    def _tick(self):
        for sid, jn in self._joint_for_id.items():
            val = self._value_for_joint.get(jn)
            if val is None:
                continue
            cmd = PositionCommand()
            cmd.position = [math.nan]
            cmd.velocity = [val]
            self._pubs[sid].publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MultiWheelMoteus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
