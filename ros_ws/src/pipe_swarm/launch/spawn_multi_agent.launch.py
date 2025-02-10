from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch.event_handlers import OnProcessExit
import yaml

import os

def recursive_controllers_updater(controllers, namespace_tag, updated_namespace):
    updated_controllers = {}
    for key, value in controllers.items():
        updated_key = key.replace(namespace_tag, updated_namespace)  # Replace key namespace
        if isinstance(value, dict):
            updated_value = recursive_controllers_updater(value, namespace_tag, updated_namespace)
            updated_controllers[updated_key] = updated_value
        else:
            updated_value = value
            if isinstance(value, list):
                i = 0
                for string in updated_value:
                    updated_value[i] = string.replace(namespace_tag, updated_namespace)
                    i+=1
            updated_controllers[updated_key] = updated_value
    return updated_controllers


def update_controllers_namespace(namespace):
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
    world_path = os.path.join(pipe_swarm_share, 'worlds', 'hollow_pipe.sdf')

    agents = []

    # Gazebo Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # Pre-Agent Setup
    agents.append(gazebo_launch)

    for i in range(1):  # Launch 3 robots
        namespace = f'agent_{i}'

        # update_controllers_namespace(namespace)

        # Load and publish the robot state
        pipe_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            namespace=namespace,
            parameters=[{'robot_description': Command(['xacro ', xacro_path, ' agent_namespace:=', namespace])}]
        )

        # Spawn the robot entity in Gazebo
        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_agent_{i}',
            output='screen',
            arguments=[ '-entity', namespace,
                        '-x', f'{i*2}',
                        '-y', '0',
                        '-z', '0',
                        '-robot_namespace', namespace,
                        '-topic', f'/{namespace}/robot_description'
                    ]
        )

        spawn_joint_state_broadcaster = Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['joint_state_broadcaster'],
            output='screen',
        )
        
        spawn_position_controller = Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['position_controller'],
            output='screen',
        )
        
        spawn_skid_steer_controller = Node(
            package='controller_manager',
            executable='spawner',
            namespace=namespace,
            arguments=['skid_steer_controller'],
            output='screen',
        )

        ros_controllers_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[spawn_joint_state_broadcaster,
                         spawn_position_controller,
                         # spawn_skid_steer_controller,
                        ],
            )
        )

        agents.extend([
            ros_controllers_event,
            pipe_robot_state_publisher,
            spawn_entity,
        ])
    
    # Launch RViz2 for visualization
    rviz_display = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    # agents.append(rviz_display)
    return LaunchDescription(agents)