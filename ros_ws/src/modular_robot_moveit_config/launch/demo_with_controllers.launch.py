from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
import os

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("modular_robot", package_name="modular_robot_moveit_config")
        .robot_description(file_path="config/modular_robot.urdf.xacro")
        .robot_description_semantic(file_path="config/modular_robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    ros2_controllers_path = os.path.join(
        get_package_share_directory("modular_robot_moveit_config"),
        "config",
        "ros2_controllers.yaml"
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )
    
    chain_joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        parameters=["chain_joint_trajectory_controller", "--controller-manager", "/controller_manager"],
        output="screen"
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"]
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("modular_robot_moveit_config"),
        "config",
        "moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description]
    )

    return LaunchDescription(
        [
            ros2_control_node,
            joint_state_broadcaster_spawner,
            chain_joint_trajectory_controller_spawner,
            robot_state_publisher_node,
            move_group_node,
            rviz_node
        ]
    )


