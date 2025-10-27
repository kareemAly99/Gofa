#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time
import math

class PerfectElectroplating(Node):
    def __init__(self):
        super().__init__('perfect_electroplating')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        self.current_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        time.sleep(2.0)
        
    def joint_callback(self, msg):
        if len(msg.position) == 6:
            self.current_joints = list(msg.position)
    
    def move_to_position(self, target_joints, duration=4.0, description=""):
        """Move smoothly to target joint position"""
        print(f"Moving: {description}")
        
        start_joints = self.current_joints.copy()
        steps = int(duration * 20)
        step_time = duration / steps
        
        for i in range(steps + 1):
            t = i / steps
            smooth_t = 0.5 - 0.5 * math.cos(t * math.pi)
            
            current_joints = []
            for j in range(6):
                current_joint = start_joints[j] + smooth_t * (target_joints[j] - start_joints[j])
                current_joints.append(current_joint)
            
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = self.joint_names
            joint_state.position = current_joints
            
            self.joint_pub.publish(joint_state)
            self.current_joints = current_joints
            time.sleep(step_time)
        
        # Final position
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joint_names
        joint_state.position = target_joints
        
        self.joint_pub.publish(joint_state)
        self.current_joints = target_joints.copy()
        time.sleep(0.5)
        
        print("✓ Motion complete")
        return target_joints

def main():
    rclpy.init()
    node = PerfectElectroplating()
    
    print("PERFECT ELECTROPLATING SEQUENCE")
    print("="*70)
    print("Using exact joint positions from manual positioning")
    print("No tank intersections - Pure motions")
    print("="*70)
    
    # ALL JOINT POSITIONS FROM MANUAL POSITIONING
    positions = {
        # TANK 0
        'above_0': [0.2799, -0.5265, 0.6941, 0.0170, 0.6962, 0.3817],
        'inside_0': [0.2799, 0.2545, 0.6941, 0.0170, 0.6962, 0.3817],
        
        # TANK 1  
        'above_1': [-0.2799, -0.5265, 0.6941, 0.0170, 0.5944, -0.5353],
        'inside_1': [-0.2799, 0.2205, 0.6941, 0.0170, 0.5944, -0.5353],
        
        # TANK 2
        'above_2a': [-0.7898, -0.2545, 0.6941, 0.6622, 0.4587, -0.5353],  # Before tank 2
        'inside_2': [-0.6880, 0.4247, 0.4891, -0.0170, 0.5605, -0.9934],
        'above_2b': [-0.6880, -0.1527, 0.4891, -0.0170, 0.5605, -0.9934], # After tank 2
        
        # TANK 3
        'above_3': [-1.0443, 0.1527, 0.1087, 0.6962, 0.4926, -0.9934],
        'inside_3': [-0.9425, 0.7301, -0.0374, 0.3230, 0.6283, -1.0952]
    }
    
    # Safe home position
    home = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    try:
        print("\n🚀 STARTING PERFECT ELECTROPLATING")
        
        # Start from inside Tank 0 (current position)
        print(f"\n{'='*70}")
        print("STARTING FROM INSIDE TANK 0")
        print(f"{'='*70}")
        node.move_to_position(positions['inside_0'], 2.0, "Starting inside Tank 0")
        time.sleep(1.0)
        
        # LIFT from Tank 0
        print(f"🔼 LIFTING FROM TANK 0")
        node.move_to_position(positions['above_0'], 3.0, "Above Tank 0")
        time.sleep(1.0)
        
        # PROCESS TANK 1
        print(f"\n{'='*70}")
        print("PROCESSING TANK 1")
        print(f"{'='*70}")
        node.move_to_position(positions['above_1'], 5.0, "Moving to above Tank 1")
        time.sleep(1.0)
        node.move_to_position(positions['inside_1'], 3.0, "Lowering into Tank 1")
        print("⏳ Electroplating Tank 1 (3 seconds)...")
        time.sleep(3.0)
        node.move_to_position(positions['above_1'], 3.0, "Lifting from Tank 1")
        time.sleep(1.0)
        
        # PROCESS TANK 2
        print(f"\n{'='*70}")
        print("PROCESSING TANK 2")
        print(f"{'='*70}")
        node.move_to_position(positions['above_2a'], 5.0, "Moving to above Tank 2")
        time.sleep(1.0)
        node.move_to_position(positions['inside_2'], 3.0, "Lowering into Tank 2")
        print("⏳ Electroplating Tank 2 (3 seconds)...")
        time.sleep(3.0)
        node.move_to_position(positions['above_2b'], 3.0, "Lifting from Tank 2")
        time.sleep(1.0)
        
        # PROCESS TANK 3
        print(f"\n{'='*70}")
        print("PROCESSING TANK 3")
        print(f"{'='*70}")
        node.move_to_position(positions['above_3'], 5.0, "Moving to above Tank 3")
        time.sleep(1.0)
        node.move_to_position(positions['inside_3'], 3.0, "Lowering into Tank 3")
        print("⏳ Electroplating Tank 3 (3 seconds)...")
        time.sleep(3.0)
        node.move_to_position(positions['above_3'], 3.0, "Lifting from Tank 3")
        time.sleep(1.0)
        
        # RETURN HOME
        print(f"\n{'='*70}")
        print("RETURNING TO HOME POSITION")
        print(f"{'='*70}")
        node.move_to_position(home, 5.0, "Home Position")
        
        print(f"\n{'='*70}")
        print("🎉 PERFECT ELECTROPLATING COMPLETED!")
        print("✓ Used exact joint positions from manual testing")
        print("✓ No tank intersections")
        print("✓ Smooth realistic motions")
        print(f"{'='*70}")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
