from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import TimerEvent
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    ld = LaunchDescription()

    modular_robot_share = get_package_share_directory('modular_robot')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    moveit_config_share = get_package_share_directory('modular_robot_moveit_config')
    xacro_path = os.path.join(modular_robot_share, 'urdf', 'modular_robot.urdf.xacro')
    rviz_path = os.path.join(moveit_config_share, 'config', 'moveit.rviz')

    ros2_controllers_path = os.path.join(modular_robot_share, "config", "modular_controller.yaml")

    gazebo_launch_path = os.path.join(gazebo_ros_share, "launch", "gazebo.launch.py")

    moveit_config = (
        MoveItConfigsBuilder("modular_robot", package_name="modular_robot_moveit_config")
        .robot_description(file_path="config/modular_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/modular_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_scene_monitor(
            publish_robot_description= True, publish_robot_description_semantic= True
        ).planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # Gazebo Launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments = {
            "use_sim_time": "true",
            "debug": "false",
            "gui": "true",
            "paused": "true",
        }.items()
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics
        ]
    )

    # Spawn the robot entity in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[ '-entity', 'modular_robot',
                    '-topic', 'robot_description'
                ],
        output='screen'
    )

    # Controller manager
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
        # remappings=[
        #     ("~/robot_description", "/robot_description")
        # ]
    )

    # Load and publish the robot state
    modular_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description]
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

    use_sim_time = {"use_sim_time": True}
    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict.update(use_sim_time)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict],
        arguments=["--ros-args", "--log-level", "info"]
    )

    ros_controllers_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[spawn_joint_state_broadcaster,
                        spawn_joint_trajectory_controller,
                    ]
        )
    )
    
    rviz_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=modular_robot_state_publisher,
            on_start=[rviz_node]
        )
    )

    return LaunchDescription(
        [
            gazebo_launch,
            controller_manager_node,
            spawn_entity,
            modular_robot_state_publisher,
            move_group_node,
            ros_controllers_event,
            rviz_event,
        ]
    )



    