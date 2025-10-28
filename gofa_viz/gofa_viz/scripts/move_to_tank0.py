#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class MoveToTank0Node(Node):
    def __init__(self):
        super().__init__('move_to_tank0_node')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info("MoveToTank0Node started, publishing joint angles in 1s...")
        self.timer = self.create_timer(1.0, self.publish_joint_angles)
        # Precomputed IK joint angles to reach Tank 0
        self.joint_angles = [0.0, -0.1161974, 0.53848606, 0.76509671, 1.524072, -0.63046708, -2.86662413]
        self.sent = False

    def publish_joint_angles(self):
        if self.sent:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7']
        msg.position = self.joint_angles
        self.publisher.publish(msg)
        self.get_logger().info(f"Published joint angles: {self.joint_angles}")
        self.sent = True

def main(args=None):
    rclpy.init(args=args)
    node = MoveToTank0Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF