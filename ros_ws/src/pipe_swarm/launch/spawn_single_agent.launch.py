from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, OpaqueFunction
from launch.event_handlers import OnProcessExit

from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def update_controllers_namespace(namespace):
    print(f'namespace is {namespace}')
    pipe_swarm_share = get_package_share_directory('pipe_swarm')
    controllers_yaml_path = os.path.join(pipe_swarm_share, 'config', 'robot_controllers_sim.yaml')
    controllers_temp_path = os.path.join(pipe_swarm_share, 'config', f'{namespace}_controllers_sim.yaml')

    updated_controllers = {}

    # Load YAML file and replace `{namespace}` dynamically
    with open(controllers_yaml_path, 'r') as file:
        controllers = yaml.safe_load(file)
    
    for key, value in controllers.items():
        updated_key = key.replace('/namespace_tag', f'/{namespace}')
        updated_controllers[updated_key] = value

    # Save updated YAML
    with open(controllers_temp_path, 'w') as file:
        yaml.dump(updated_controllers, file)

def run_update_controllers_namespace(context): 
    namespace = LaunchConfiguration('namespace').perform(context)
    update_controllers_namespace(namespace)
    return []

# -----------------

def generate_launch_description():
    # Paths to resources
    pipe_swarm_share = get_package_share_directory('pipe_swarm')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    xacro_path = os.path.join(pipe_swarm_share, 'urdf', 'pipe_robot.urdf.xacro')

    namespace_arg = DeclareLaunchArgument('namespace',default_value='agent_n',description="Agent's Namespace")
    update_namespace_action = OpaqueFunction(function=run_update_controllers_namespace) 
    namespace = LaunchConfiguration('namespace')
    
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
        namespace_arg,
        update_namespace_action,
        gazebo_include,
        pipe_robot_state_publisher,
        spawn_entity,
        rviz_display
    ])