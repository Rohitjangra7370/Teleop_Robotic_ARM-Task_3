import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get URDF via xacro
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
    
    # Declare Gazebo launch argument
    declare_use_gazebo = DeclareLaunchArgument(
        'use_gazebo',
        default_value='true',
        description='Whether to launch Gazebo'
    )
    
    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch', 'gz_sim.launch.py'
            )
        ),
        condition=IfCondition(LaunchConfiguration('use_gazebo')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    # Controller Manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_robot',
        arguments=[
            '-topic', '/robot_description', 
            '-name', 'arm_robot',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gazebo'))
    )
    
    # ROS-Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_gazebo'))
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
        declare_use_gazebo,
        gazebo_launch,
        robot_state_publisher_node,
        controller_manager_node,
        spawn_entity,
        bridge,
        
        # Wait for controller manager to be ready
        TimerAction(
            period=5.0,
            actions=[load_joint_state_broadcaster]
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_arm_controller],
            )
        ),
    ])
