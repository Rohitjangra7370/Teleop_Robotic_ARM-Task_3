#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
import numpy as np
from builtin_interfaces.msg import Duration

class CartesianController(Node):
    def __init__(self):
        super().__init__('cartesian_controller')
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/arm_cartesian_velocity', self.velocity_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
            
        # Publisher for joint trajectory
        self.joint_traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
            
        # Current joint positions
        self.current_joints = [0.0] * 6
        self.joint_names = [
            'base_base_1', 'base_1_shoulder', 'shoulder_upper_arm',
            'upper_arm_forearm', 'forearm_wrist_1', 'wrist_1_wrist_2'
        ]
        
        # Control parameters
        self.dt = 0.1  # Control loop time step
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.current_velocity = Twist()
        
    def joint_state_callback(self, msg):
        """Update current joint positions"""
        if len(msg.position) >= 6:
            self.current_joints = list(msg.position[:6])
            
    def velocity_callback(self, msg):
        """Store the latest velocity command"""
        self.current_velocity = msg
        
    def control_loop(self):
        """Main control loop - convert Cartesian velocity to joint increments"""
        if (abs(self.current_velocity.linear.x) > 0.001 or 
            abs(self.current_velocity.linear.y) > 0.001 or 
            abs(self.current_velocity.linear.z) > 0.001 or
            abs(self.current_velocity.angular.x) > 0.001 or
            abs(self.current_velocity.angular.y) > 0.001 or
            abs(self.current_velocity.angular.z) > 0.001):
            
            # Simple joint-space mapping (you can implement proper Jacobian here)
            joint_increments = self.cartesian_to_joint_velocity(self.current_velocity)
            
            # Apply increments to current joint positions
            new_joints = [
                self.current_joints[i] + joint_increments[i] * self.dt 
                for i in range(6)
            ]
            
            # Apply joint limits
            new_joints = self.apply_joint_limits(new_joints)
            
            # Publish joint trajectory
            self.publish_joint_trajectory(new_joints)
            
    def cartesian_to_joint_velocity(self, twist):
        """Convert Cartesian velocity to joint velocities (simplified mapping)"""
        # This is a simplified mapping - you can implement proper inverse kinematics
        joint_vels = [0.0] * 6
        
        # Base rotation (yaw)
        joint_vels[0] = twist.angular.z
        
        # Shoulder (pitch for forward/backward)
        joint_vels[1] = -twist.linear.x * 2.0
        
        # Upper arm (pitch for up/down)
        joint_vels[2] = twist.linear.z * 2.0
        
        # Forearm (pitch)
        joint_vels[3] = twist.linear.x * 1.5
        
        # Wrist 1 (roll)
        joint_vels[4] = twist.angular.x
        
        # Wrist 2 (pitch)
        joint_vels[5] = twist.angular.y
        
        return joint_vels
        
    def apply_joint_limits(self, joints):
        """Apply joint limits to prevent damage"""
        limits = [
            (-3.14, 3.14),   # Base
            (-1.57, 1.57),   # Shoulder
            (-1.57, 1.57),   # Upper arm
            (-1.57, 1.57),   # Forearm
            (-1.57, 1.57),   # Wrist 1
            (-1.57, 1.57),   # Wrist 2
        ]
        
        return [max(limits[i][0], min(limits[i][1], joints[i])) for i in range(6)]
        
    def publish_joint_trajectory(self, target_joints):
        """Publish joint trajectory command"""
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9))
        
        traj.points = [point]
        self.joint_traj_pub.publish(traj)

def main():
    rclpy.init()
    node = CartesianController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
