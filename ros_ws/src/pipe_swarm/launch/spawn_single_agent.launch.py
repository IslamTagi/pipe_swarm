from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to resources
    pipe_swarm_share = get_package_share_directory('pipe_swarm')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    xacro_path = os.path.join(pipe_swarm_share, 'urdf', 'pipe_robot.urdf.xacro')

    return LaunchDescription([
        # Include Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
            )
        ),

        # Load and publish the robot state
        Node
        (
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace='agent_n',
            parameters=[{'robot_description': Command(['xacro ', xacro_path, ' agent_namespace:=', 'agent_n'])}]
        ),

        # Spawn the robot entity in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_agent',
            namespace='agent_n',
            output='screen',
            arguments=['-topic', 'robot_description', '-entity', 'pipe_robot']
        ),

        # Launch RViz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])
