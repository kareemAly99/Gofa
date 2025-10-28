#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
import sys

class MoveCarrierNode(Node):
    def __init__(self):
        super().__init__('move_carrier_node')

        # Initialize MoveIt commander
        self.robot = RobotCommander()
        self.scene = PlanningSceneInterface()
        self.group = MoveGroupCommander("panda_arm")  # change if your MoveIt group has a different name

        # Tank 0 position (from add_tanks.py)
        x = 0.805
        y = -1.10
        z = 0.23  # middle of the tank

        # Orientation: pointing straight down (quaternion)
        # Adjust depending on your robot
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z

        # Orientation: straight down
        target_pose.orientation.x = 0.707
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.707

        self.get_logger().info(f"Target Pose: {target_pose}")

        # Set pose target
        self.group.set_pose_target(target_pose)

        # Plan and execute
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        if plan:
            self.get_logger().info("Successfully moved to tank 0!")
        else:
            self.get_logger().error("Failed to reach tank 0.")

def main(args=None):
    rclpy.init(args=args)
    node = MoveCarrierNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
