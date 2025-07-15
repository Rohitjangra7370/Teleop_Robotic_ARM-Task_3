#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    # Package paths
    pkg_main = 'manipulator'
    # pkg_moveit = 'move_it_config_testing_june_25'
    pkg_gz = 'ros_gz_sim'
    pkg_path = get_package_share_directory(pkg_main)
    # launch_dir_moveit = os.path.join(get_package_share_directory(pkg_moveit), 'launch')
    launch_dir_main = os.path.join(get_package_share_directory(pkg_main), 'launch')
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot_core.urdf.xacro')
    # controllers_yaml = os.path.join(pkg_path, 'config', 'ros2_controllers.yaml')


    declare_gazebo = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        description='Whether to launch Gazebo Garden'
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory(pkg_gz),
                'launch', 'gz_sim.launch.py'
            )
        ]),
        condition=IfCondition(LaunchConfiguration('use_gazebo')),
        launch_arguments={'gz_args':'-r empty.sdf'}.items()
    )

    robot_spawner=Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'robotic_manipulator',
            '-x', '0', '-y', '0', '-z', '0.1',
            '-R', '0', '-P', '0', '-Y', '0'
        ]
    )


    bridge = Node(
        package = 'ros_gz_bridge',
        executable = 'parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        output='screen'
    )
    robot_controllers = os.path.join(
        pkg_path,
        "config",
        "controllers.yaml"
    )

    return LaunchDescription([
        declare_gazebo,
        gazebo_launch,
        robot_spawner,
        bridge


    ])
