#!/usr/bin/env python3

import rclpy
from moveit_commander.robot_commander import RobotCommander

def main():
    rclpy.init()
    robot = RobotCommander()
    print("MoveIt groups available:", robot.get_group_names())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
