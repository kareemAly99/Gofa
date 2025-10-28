#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, RobotCommander, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped

def main():
    rclpy.init()
    node = Node("test_ik_node")

    # Initialize MoveIt interfaces
    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group = MoveGroupCommander("panda_arm")  # use your robot group name if different

    # Define a sample target pose (modify these values later for tanks)
    target_pose = PoseStamped()
    target_pose.header.frame_id = robot.get_planning_frame()
    target_pose.pose.position.x = 0.5
    target_pose.pose.position.y = 0.0
    target_pose.pose.position.z = 0.5
    target_pose.pose.orientation.x = 0.0
    target_pose.pose.orientation.y = 0.0
    target_pose.pose.orientation.z = 0.0
    target_pose.pose.orientation.w = 1.0

    group.set_pose_target(target_pose)

    # Compute IK solution
    plan = group.plan()
    if plan and len(plan.joint_trajectory.points) > 0:
        node.get_logger().info(f"IK solution found: {plan.joint_trajectory.points[-1].positions}")
    else:
        node.get_logger().warn("No IK solution found.")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
