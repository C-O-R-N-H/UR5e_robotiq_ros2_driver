#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pyRobotiqGripper import RobotiqGripper

class RobotiqHandEDriver(Node):
    """
    ROS2 driver node for Robotiq Hand-E gripper.
    
    Subscribes to:
        - /gripper_command (String): Accepts "open" or "close" commands
    """
    
    def __init__(self):
        super().__init__('robotiq_hand_e_driver')
        
        # Log initialization
        self.get_logger().info('Initializing Robotiq Hand-E Driver')
        
        # Initialize the gripper
        try:
            self.gripper = RobotiqGripper()
            self.get_logger().info('Connected to Robotiq gripper')
            
            # Activate the gripper
            self.get_logger().info('Activating gripper...')
            self.gripper.activate()
            self.get_logger().info('Gripper activated successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize gripper: {str(e)}')
            rclpy.shutdown()
            return
            
        # Create a subscription to the gripper_command topic
        self.subscription = self.create_subscription(
            String,
            '/gripper_command',
            self.gripper_command_callback,
            10)
        self.get_logger().info('Subscribed to /gripper_command topic')
        
        self.get_logger().info('Robotiq Hand-E Driver is ready')
    
    def gripper_command_callback(self, msg):
        """
        Callback function for gripper command messages.
        
        Args:
            msg (String): Command message. Should be either "open" or "close"
        """
        command = msg.data.lower()
        
        try:
            if command == "open":
                self.get_logger().info('Opening gripper')
                self.gripper.open()
                self.get_logger().info('Gripper opened')
                
            elif command == "close":
                self.get_logger().info('Closing gripper')
                self.gripper.close()
                self.get_logger().info('Gripper closed')
                
            else:
                self.get_logger().warn(f'Unknown command: {command}. Expected "open" or "close"')
                
        except Exception as e:
            self.get_logger().error(f'Failed to execute gripper command: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    external_perception = RobotiqHandEDriver()
    rclpy.spin(external_perception)
    external_perception.destroy_node()
    rclpy.shutdown()
    rclpy.init(args=args)

if __name__ == '__main__':
    main()