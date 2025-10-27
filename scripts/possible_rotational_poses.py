#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math

class StableRotationDemo(Node):
    def __init__(self):
        super().__init__('stable_rotation_demo')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.current_joints = None
        
        # Wait for initial joint state
        print("Waiting for current robot state...")
        while self.current_joints is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        print("Got current robot state!")
        
    def joint_callback(self, msg):
        if len(msg.position) == 6:
            self.current_joints = list(msg.position)
    
    def move_to_position(self, target_joints, duration=5.0):
        """Move to target position from current position"""
        if self.current_joints is None:
            print("No current joint state available!")
            return
            
        print(f"Moving to target position...")
        steps = int(duration * 30)
        step_time = duration / steps
        
        for i in range(steps + 1):
            t = i / steps
            smooth_t = 0.5 - 0.5 * math.cos(t * math.pi)
            
            current_joints = []
            for j in range(6):
                current_joint = self.current_joints[j] + smooth_t * (target_joints[j] - self.current_joints[j])
                current_joints.append(current_joint)
            
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = self.joint_names
            joint_state.position = current_joints
            
            self.joint_pub.publish(joint_state)
            self.current_joints = current_joints
            time.sleep(step_time)
        
        # Final position
        self.current_joints = target_joints.copy()
        print("✓ Motion complete")

def main():
    rclpy.init()
    node = StableRotationDemo()
    
    print("STABLE ROTATION DEMONSTRATION")
    print("="*70)
    print("Using actual robot state - No shaking!")
    print("="*70)
    
    time.sleep(2.0)
    
    # Tank positions
    tank_positions = {
        'TANK 0': [0.2799, -0.5265, 0.6941, 0.0170, 0.6962, 0.3817],
        'TANK 1': [-0.2799, -0.5265, 0.6941, 0.0170, 0.5944, -0.5353],
        'TANK 2': [-0.6880, -0.1527, 0.4891, -0.0170, 0.5605, -0.9934],
        'TANK 3': [-1.0443, 0.1527, 0.1087, 0.6962, 0.4926, -0.9934]
    }
    
    # Rotation sequences for each tank
    rotation_sequences = {
        'TANK 0': [
            [0.2799, -0.5265, 0.6941, 0.0, 0.0, 0.0],
            [0.2799, -0.5265, 0.6941, 0.0, 0.785, 0.0],
            [0.2799, -0.5265, 0.6941, 0.0, -0.785, 0.0],
            [0.2799, -0.5265, 0.6941, 0.785, 0.0, 0.0],
            [0.2799, -0.5265, 0.6941, -0.785, 0.0, 0.0],
            [0.2799, -0.5265, 0.6941, 0.0, 0.0, 1.57],
            [0.2799, -0.5265, 0.6941, 0.0, 0.0, -1.57],
            [0.2799, -0.5265, 0.6941, 0.0, 0.0, 3.14],
            [0.2799, -0.5265, 0.6941, 0.0, 0.0, 0.0]
        ],
        'TANK 1': [
            [-0.2799, -0.5265, 0.6941, 0.0, 0.0, 0.0],
            [-0.2799, -0.5265, 0.6941, 0.0, 0.785, 0.0],
            [-0.2799, -0.5265, 0.6941, 0.0, -0.785, 0.0],
            [-0.2799, -0.5265, 0.6941, 0.785, 0.0, 0.0],
            [-0.2799, -0.5265, 0.6941, -0.785, 0.0, 0.0],
            [-0.2799, -0.5265, 0.6941, 0.0, 0.0, 1.57],
            [-0.2799, -0.5265, 0.6941, 0.0, 0.0, -1.57],
            [-0.2799, -0.5265, 0.6941, 0.0, 0.0, 3.14],
            [-0.2799, -0.5265, 0.6941, 0.0, 0.0, 0.0]
        ],
        'TANK 2': [
            [-0.6880, -0.1527, 0.4891, 0.0, 0.0, 0.0],
            [-0.6880, -0.1527, 0.4891, 0.0, 0.785, 0.0],
            [-0.6880, -0.1527, 0.4891, 0.0, -0.785, 0.0],
            [-0.6880, -0.1527, 0.4891, 0.785, 0.0, 0.0],
            [-0.6880, -0.1527, 0.4891, -0.785, 0.0, 0.0],
            [-0.6880, -0.1527, 0.4891, 0.0, 0.0, 1.57],
            [-0.6880, -0.1527, 0.4891, 0.0, 0.0, -1.57],
            [-0.6880, -0.1527, 0.4891, 0.0, 0.0, 3.14],
            [-0.6880, -0.1527, 0.4891, 0.0, 0.0, 0.0]
        ],
        'TANK 3': [
            [-1.0443, 0.1527, 0.1087, 0.0, 0.0, 0.0],
            [-1.0443, 0.1527, 0.1087, 0.0, 0.785, 0.0],
            [-1.0443, 0.1527, 0.1087, 0.0, -0.785, 0.0],
            [-1.0443, 0.1527, 0.1087, 0.785, 0.0, 0.0],
            [-1.0443, 0.1527, 0.1087, -0.785, 0.0, 0.0],
            [-1.0443, 0.1527, 0.1087, 0.0, 0.0, 1.57],
            [-1.0443, 0.1527, 0.1087, 0.0, 0.0, -1.57],
            [-1.0443, 0.1527, 0.1087, 0.0, 0.0, 3.14],
            [-1.0443, 0.1527, 0.1087, 0.0, 0.0, 0.0]
        ]
    }
    
    try:
        # Process each tank
        for tank_name, tank_pos in tank_positions.items():
            print(f"\n🎯 MOVING TO {tank_name}")
            node.move_to_position(tank_pos, 6.0)
            time.sleep(2.0)
            
            print(f"\n🔄 SHOWING ROTATIONS FOR {tank_name}")
            rotations = rotation_sequences[tank_name]
            
            for i, rotation_pos in enumerate(rotations):
                print(f"Rotation {i+1}/{len(rotations)}")
                node.move_to_position(rotation_pos, 3.0)
                time.sleep(2.0)
            
            print(f"✅ Finished {tank_name}")
            time.sleep(2.0)
        
        print(f"\n🎉 ALL ROTATIONS COMPLETED SMOOTHLY!")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()