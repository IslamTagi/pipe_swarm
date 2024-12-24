from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
import yaml

import os

def recursive_controllers_updater(controllers, key_tag, key_update):
    updated_controllers = {}
    for key, value in controllers.items():
        updated_key = key.replace(key_tag, key_update)  # Replace key namespace
        if isinstance(value, dict):
            updated_value = recursive_controllers_updater(value, key_tag, key_update)
            updated_controllers[updated_key] = updated_value
        else:
            updated_controllers[updated_key] = value
    return updated_controllers


def update_controllers_namespace(namespace):
    print(f'Namespace is {namespace}')
    pipe_swarm_share = get_package_share_directory('pipe_swarm')
    controllers_yaml_path = os.path.join(pipe_swarm_share, 'config', 'robot_controllers_sim.yaml')
    controllers_temp_path = os.path.join(pipe_swarm_share, 'config', f'{namespace}_controllers_sim.yaml')

    updated_controllers = {}

    # Load YAML file and replace `{namespace}` dynamically
    with open(controllers_yaml_path, 'r') as file:
        controllers = yaml.safe_load(file)
    
    updated_controllers = recursive_controllers_updater(controllers, 'namespace_tag', f'{namespace}')
    
    # Save updated YAML
    with open(controllers_temp_path, 'w') as file:
        yaml.dump(updated_controllers, file)

def generate_launch_description():
    
    # Paths to resources
    pipe_swarm_share = get_package_share_directory('pipe_swarm')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    xacro_path = os.path.join(pipe_swarm_share, 'urdf', 'pipe_robot.urdf.xacro')

    agents = []

    # Gazebo Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        )
    )

    # Pre-Agent Setup
    agents.append(gazebo_launch)

    # for i in range(2):  # Launch 3 robots
    namespace_1 = f'agent_1'
    namespace_2 = f'agent_2'

    # update_controllers_namespace(namespace)

    # Load and publish the robot state
    pipe_robot_state_publisher_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=namespace_1,
        parameters=[{'robot_description': Command(['xacro ', xacro_path, ' agent_namespace:=', namespace_1])}]
    )
    pipe_robot_state_publisher_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        namespace=namespace_2,
        parameters=[{'robot_description': Command(['xacro ', xacro_path, ' agent_namespace:=', namespace_2])}]
    )

    # Spawn the robot entity in Gazebo
    spawn_entity_1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[ '-entity', namespace_1,
                    '-x', f'{0*2}',
                    '-y', '0',
                    '-z', '0', 
                    '-robot_namespace', namespace_1,
                    '-topic', f'/{namespace_1}/robot_description'
                ]
    )
    spawn_entity_2 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[ '-entity', namespace_2,
                    '-x', f'{1*2}',
                    '-y', '0',
                    '-z', '0', 
                    '-robot_namespace', namespace_2,
                    '-topic', f'/{namespace_2}/robot_description'
                ]
    )

    # spawn_joint_state_broadcaster = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     namespace=namespace,
    #     arguments=[f'{namespace}_joint_state_broadcaster'],
    #     output='screen',
    # )
    
    # spawn_position_controller = Node(
    #     package='controller_manager',
    #     executable='spawner',
    #     namespace=namespace,
    #     arguments=[f'{namespace}_position_controller'],
    #     output='screen',
    # )

    spawn_joint_state_broadcaster_1 = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace_1,
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    spawn_joint_state_broadcaster_2 = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace_2,
        arguments=['joint_state_broadcaster'],
        output='screen',
    )
    
    spawn_position_controller_1 = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace_1,
        arguments=['position_controller'],
        output='screen',
    )
    spawn_position_controller_2 = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace_2,
        arguments=['position_controller'],
        output='screen',
    )

    ros_controllers_event_1 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_1,
            on_exit=[spawn_joint_state_broadcaster_1,
                        spawn_position_controller_1],
        )
    )
    ros_controllers_event_2 = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity_2,
            on_exit=[spawn_joint_state_broadcaster_2,
                        spawn_position_controller_2],
        )
    )

    # agents.extend([
    #     pipe_robot_state_publisher_1,
    #     spawn_entity_1,
    #     spawn_entity_2,
    #     # ros_controllers_event
    # ])
    
    # # Launch RViz2 for visualization
    # rviz_display = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     output='screen'
    # )

    # agents.append(rviz_display)

    return LaunchDescription([
        ros_controllers_event_1,
        ros_controllers_event_2,
        gazebo_launch,
        pipe_robot_state_publisher_1,
        pipe_robot_state_publisher_2,
        spawn_entity_1,
        spawn_entity_2
    ])
