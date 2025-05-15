from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Launch arguments
    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    launch_rviz = LaunchConfiguration('launch_rviz')

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('ur_type', default_value='ur5e'),
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.102'),
        DeclareLaunchArgument('launch_rviz', default_value='false'),

        # Hand-E Gripper driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ur_robotiq_driver'),
                    'launch',
                    'hand_e.launch.py'
                )
            )
        ),

        # UR5e control (hardware interface)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ur_robot_driver'),
                    'launch',
                    'ur_control.launch.py'
                )
            ),
            launch_arguments={
                'ur_type': ur_type,
                'robot_ip': robot_ip,
                'launch_rviz': launch_rviz
            }.items()
        ),

        # MoveIt configuration
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('ur_moveit_config'),
                    'launch',
                    'ur_moveit.launch.py'
                )
            ),
            launch_arguments={
                'ur_type': ur_type,
                'robot_ip': robot_ip,
                'launch_rviz': "true",
            }.items()
        )
    ])
