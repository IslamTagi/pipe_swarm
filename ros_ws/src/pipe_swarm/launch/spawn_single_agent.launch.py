from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to resources
    pipe_swarm_share = get_package_share_directory('pipe_swarm')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    xacro_path = os.path.join(pipe_swarm_share, 'urdf', 'pipe_robot.urdf.xacro')

    namespace = f'agent_n'

    # Include Gazebo launch file
    gazebo_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
            )
        )
    
    # Load and publish the robot state
    pipe_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        namespace=namespace,
        parameters=[{'robot_description': Command(['xacro ', xacro_path, ' agent_namespace:=', namespace])}]
    )

    # Spawn the robot entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        namespace=namespace,
        arguments=['-topic', 'robot_description',
                    '-entity', 'pipe_robot'],
        output='screen'
    )

    # Launch RViz2 for visualization
    rviz_display = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    # load_joint_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', 
    #          '--set-state', 'active',
    #          '--controller-manager', '/agent_n/controller_manager',
    #          'joint_state_broadcaster'],
    #     output='screen'
    # )

    # load_position_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller',
    #          '--set-state', 'active',
    #          '--controller-manager', '/agent_n/controller_manager',
    #          'position_controller'],
    #     output='screen'
    # )

    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    load_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        arguments=['position_controller'],
        output='screen',
    )
    
    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_position_controller],
            )
        ),
        gazebo_include,
        pipe_robot_state_publisher,
        spawn_entity,
        rviz_display
    ])
