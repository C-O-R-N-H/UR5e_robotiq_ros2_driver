from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ur_robotiq_driver', 
            executable='robotiq_hand_e_driver.py',  
            name='robotiq_hand_e_driver',
            output='screen'
        )
    ])
