from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
import yaml

import os

def generate_launch_description():
    
    # Paths to resources
    modular_robot_share = get_package_share_directory('modular_robot')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    xacro_path = os.path.join(modular_robot_share, 'urdf', 'modular_robot.urdf.xacro')
    world_path = os.path.join(modular_robot_share, 'worlds', 'bookshelf.sdf')

    agents = []

    # Gazebo Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        )
    )

    # Pre-Agent Setup
    agents.append(gazebo_launch)

    # Load and publish the robot state
    modular_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', xacro_path])}]
    )

    # Spawn the robot entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name=f'spawn_agent',
        output='screen',
        arguments=[ '-entity', 'modular_robot',
                    '-topic', f'/robot_description'
                ]
    )

    spawn_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    
    spawn_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['chain_joint_trajectory_controller'],
        output='screen',
    )
    
    ros_controllers_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[spawn_joint_state_broadcaster,
                        spawn_joint_trajectory_controller,
                    ],
        )
    )

    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui'
    )

    agents.extend([
        ros_controllers_event,
        modular_robot_state_publisher,
        # joint_state_publisher,
        spawn_entity,
    ])
    
    # Launch RViz2 for visualization
    rviz_display = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    agents.append(rviz_display)
    return LaunchDescription(agents)