#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

class MoveToTank0Node(Node):
    def __init__(self):
        super().__init__('move_to_tank0_node')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.get_logger().info("MoveToTank0Node started, publishing joint angles at 10 Hz...")

        # Precomputed IK joint angles to reach Tank 0
        self.target_angles = np.array([0.0, -0.1161974, 0.53848606, 0.76509671, 1.524072, -0.63046708, -2.86662413])
        
        # Start at all zeros (or you can set your current robot pose)
        self.current_angles = np.zeros(7)

        # Interpolation steps
        self.steps = 50
        self.step_count = 0

        # Timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_joint_angles)

    def publish_joint_angles(self):
        if self.step_count > self.steps:
            return

        alpha = self.step_count / self.steps
        interpolated_angles = (1 - alpha) * self.current_angles + alpha * self.target_angles

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1','joint_2','joint_3','joint_4','joint_5','joint_6','joint_7']
        msg.position = list(interpolated_angles)

        self.publisher.publish(msg)
        self.get_logger().info(f"Step {self.step_count}/{self.steps}, joint angles: {msg.position}")

        self.step_count += 1

def main(args=None):
    rclpy.init(args=args)
    node = MoveToTank0Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
