#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue  # Add this import
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package paths
    pkg_name = 'manipulator'
    pkg_gz = 'ros_gz_sim'
    pkg_path = get_package_share_directory(pkg_name)
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_core.urdf.xacro')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    using_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    # Generate robot description
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        xacro_file
    ])

    declare_robot_description = DeclareLaunchArgument(
        'robot_description',
        default_value=robot_description_content,
        description='URDF description of the robot'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                LaunchConfiguration('robot_description'),
                # value_type=str
            )
        }]
    )

    return LaunchDescription([
        using_sim_time,
        declare_robot_description,
        robot_state_publisher,
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        )
    ])
