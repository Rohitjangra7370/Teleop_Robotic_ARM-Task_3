#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # Publisher for Cartesian velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/arm_cartesian_velocity', 10)
        
        # Key mappings for 6DOF control
        self.key_mappings = {
            # Linear movements
            'w': (0.1, 0.0, 0.0, 0.0, 0.0, 0.0),    # +X
            's': (-0.1, 0.0, 0.0, 0.0, 0.0, 0.0),   # -X
            'a': (0.0, 0.1, 0.0, 0.0, 0.0, 0.0),    # +Y
            'd': (0.0, -0.1, 0.0, 0.0, 0.0, 0.0),   # -Y
            'q': (0.0, 0.0, 0.1, 0.0, 0.0, 0.0),    # +Z
            'e': (0.0, 0.0, -0.1, 0.0, 0.0, 0.0),   # -Z
            
            # Rotational movements
            'i': (0.0, 0.0, 0.0, 0.1, 0.0, 0.0),    # +Roll
            'k': (0.0, 0.0, 0.0, -0.1, 0.0, 0.0),   # -Roll
            'j': (0.0, 0.0, 0.0, 0.0, 0.1, 0.0),    # +Pitch
            'l': (0.0, 0.0, 0.0, 0.0, -0.1, 0.0),   # -Pitch
            'u': (0.0, 0.0, 0.0, 0.0, 0.0, 0.1),    # +Yaw
            'o': (0.0, 0.0, 0.0, 0.0, 0.0, -0.1),   # -Yaw
        }
        
        self.print_instructions()
        
    def print_instructions(self):
        print("\n=== 6DOF Robotic Arm Teleoperation ===")
        print("Linear Movement:")
        print("  W/S: Move forward/backward (X-axis)")
        print("  A/D: Move left/right (Y-axis)")
        print("  Q/E: Move up/down (Z-axis)")
        print("\nRotational Movement:")
        print("  I/K: Roll rotation")
        print("  J/L: Pitch rotation")
        print("  U/O: Yaw rotation")
        print("\nPress SPACE to stop, ESC to quit")
        print("=====================================\n")
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
        
    def run(self):
        self.settings = termios.tcgetattr(sys.stdin)
        
        try:
            while rclpy.ok():
                key = self.get_key()
                
                if key == '\x1b':  # ESC key
                    break
                elif key == ' ':  # SPACE key - stop
                    self.publish_velocity(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
                elif key.lower() in self.key_mappings:
                    vel = self.key_mappings[key.lower()]
                    self.publish_velocity(*vel)
                    
        except Exception as e:
            print(f"Error: {e}")
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            
    def publish_velocity(self, x, y, z, rx, ry, rz):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = z
        twist.angular.x = rx
        twist.angular.y = ry
        twist.angular.z = rz
        
        self.cmd_vel_pub.publish(twist)
        print(f"Published: Linear({x:.1f}, {y:.1f}, {z:.1f}) Angular({rx:.1f}, {ry:.1f}, {rz:.1f})")

def main():
    rclpy.init()
    node = KeyboardTeleopNode()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
