from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import Command

def generate_launch_description():
    xacro_path = os.path.join(
        get_package_share_directory('pipe_swarm'),
        'urdf',
        'pipe_robot.urdf.xacro'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', xacro_path, ' agent_namespace:=', 'agent_n'])}]            # name='robot_state_publisher',
            # namespace='robot_ns',
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])