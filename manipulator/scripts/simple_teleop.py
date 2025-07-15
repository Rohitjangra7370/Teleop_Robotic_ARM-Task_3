#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import threading
import sys

class SimpleTeleopNode(Node):
    def __init__(self):
        super().__init__('simple_teleop')
        
        # Publisher for joint trajectory
        self.joint_traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
            
        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
            
        # Current joint positions
        self.current_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.joint_names = [
            'base_base_1', 'base_1_shoulder', 'shoulder_upper_arm',
            'upper_arm_forearm', 'forearm_wrist_1', 'wrist_1_wrist_2'
        ]
        
        self.print_instructions()
        
        # Start input thread
        self.input_thread = threading.Thread(target=self.input_loop)
        self.input_thread.daemon = True
        self.input_thread.start()
        
    def joint_state_callback(self, msg):
        if len(msg.position) >= 6:
            self.current_joints = list(msg.position[:6])
            
    def print_instructions(self):
        print("\n=== Simple Robotic Arm Teleoperation ===")
        print("Commands:")
        print("  1: Move base joint +0.2 rad")
        print("  2: Move base joint -0.2 rad")
        print("  3: Move shoulder joint +0.2 rad")
        print("  4: Move shoulder joint -0.2 rad")
        print("  5: Move upper arm joint +0.2 rad")
        print("  6: Move upper arm joint -0.2 rad")
        print("  0: Reset all joints to zero")
        print("  q: Quit")
        print("========================================\n")
        
    def input_loop(self):
        while rclpy.ok():
            try:
                cmd = input("Enter command: ").strip()
                
                if cmd == 'q':
                    rclpy.shutdown()
                    break
                elif cmd == '0':
                    self.move_to_position([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
                elif cmd == '1':
                    new_joints = self.current_joints.copy()
                    new_joints[0] += 0.2
                    self.move_to_position(new_joints)
                elif cmd == '2':
                    new_joints = self.current_joints.copy()
                    new_joints[0] -= 0.2
                    self.move_to_position(new_joints)
                elif cmd == '3':
                    new_joints = self.current_joints.copy()
                    new_joints[1] += 0.2
                    self.move_to_position(new_joints)
                elif cmd == '4':
                    new_joints = self.current_joints.copy()
                    new_joints[1] -= 0.2
                    self.move_to_position(new_joints)
                elif cmd == '5':
                    new_joints = self.current_joints.copy()
                    new_joints[2] += 0.2
                    self.move_to_position(new_joints)
                elif cmd == '6':
                    new_joints = self.current_joints.copy()
                    new_joints[2] -= 0.2
                    self.move_to_position(new_joints)
                else:
                    print("Invalid command!")
                    
            except EOFError:
                break
                
    def move_to_position(self, target_joints):
        # Apply joint limits
        limits = [(-3.14, 3.14), (-1.57, 1.57), (-1.57, 1.57), 
                 (-1.57, 1.57), (-1.57, 1.57), (-1.57, 1.57)]
        
        safe_joints = []
        for i, joint in enumerate(target_joints):
            safe_joints.append(max(limits[i][0], min(limits[i][1], joint)))
            
        # Create trajectory message
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = safe_joints
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        traj.points = [point]
        self.joint_traj_pub.publish(traj)
        
        print(f"Moving to: {[f'{j:.2f}' for j in safe_joints]}")

def main():
    rclpy.init()
    node = SimpleTeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
