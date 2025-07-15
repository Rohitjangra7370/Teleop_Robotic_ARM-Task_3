import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Robot description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("manipulator"), "urdf", "robot_core.urdf.xacro"])
    ])
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    
    # Controller configuration
    robot_controllers = PathJoinSubstitution([
        FindPackageShare("manipulator"),
        "config",
        "controllers.yaml"
    ])
    
    # Launch Ignition Gazebo manually
    gazebo_launch = ExecuteProcess(
        cmd=['ign', 'gazebo', 'empty.sdf'],
        output='screen'
    )
    
    # Core nodes
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    
    # RViz for visualization
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen"
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=['-topic', '/robot_description', '-name', 'arm_robot'],
        output='screen'
    )
    
    # Load controllers
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )
    
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        controller_manager_node,
        rviz_node,
        
        # Wait for Gazebo to start
        TimerAction(
            period=5.0,
            actions=[spawn_entity]
        ),
        
        # Wait for controller manager
        TimerAction(
            period=3.0,
            actions=[load_joint_state_broadcaster]
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_arm_controller],
            )
        ),
    ])
