#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

void splitExecuteTrajectory(std::shared_ptr<rclcpp::Node> move_group_node,
                  rclcpp::Logger logger,
                  moveit::planning_interface::MoveGroupInterface::Plan plan)
{
  auto trajectory_points = plan.trajectory_.joint_trajectory.points;

  // create publishers for each joint
  auto agent_1_pub = move_group_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/agent_1/female_joint_trajectory_controller/joint_trajectory", 10);
  
  // set joint names
  trajectory_msgs::msg::JointTrajectory traj_agent_1;
  traj_agent_1.joint_names.push_back("base_female_joint");

  const auto& points = plan.trajectory_.joint_trajectory.points;

  for (const auto& point : points)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point_agent_1;
    point_agent_1.positions.push_back(point.positions[1]); // index 1: agent_n2_pivot_base_joint
    point_agent_1.time_from_start = point.time_from_start;
    traj_agent_1.points.push_back(point_agent_1);
  }

  RCLCPP_INFO(logger, "Sending Traj 1");
  agent_1_pub->publish(traj_agent_1);
  RCLCPP_INFO(logger, "Sent Traj 1");
  
}

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const move_group_node = std::make_shared<rclcpp::Node>(
    "modular_robot_moveit",
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true)
      .parameter_overrides({{"use_sim_time", rclcpp::ParameterValue(true)}})
  );

  static const std::string PLANNING_GROUP = "mobile_chain";
  // Create a ROS logger
  auto const logger = rclcpp::get_logger("modular_robot_moveit");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  // create moveit interface to allow access to moveit functionality
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(move_group_node, PLANNING_GROUP);
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10);
  
  // fetch joint states and modify them then update planning state
  const moveit::core::JointModelGroup* joint_model_group =
      move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  std::vector<double> target_joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, target_joint_group_positions);
  target_joint_group_positions[0] = 0.09;  // m
  target_joint_group_positions[1] = -0.78;  // radians
  target_joint_group_positions[2] = -0.78;  // radians
  move_group_interface.setJointValueTarget(target_joint_group_positions);

  // We lower the allowed maximum velocity and acceleration to 5% of their maximum.
  // The default values are 10% (0.1).
  // Set your preferred defaults in the joint_limits.yaml file of your robot's moveit_config
  // or set explicit factors in your code if you need your robot to move faster.
  move_group_interface.setMaxVelocityScalingFactor(0.05);
  move_group_interface.setMaxAccelerationScalingFactor(0.05);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = "agent_n_base_link";  // Use your planning frame
  collision_object.id = "obstacle_box";

  // Define the box shape
  shape_msgs::msg::SolidPrimitive box;
  box.type = box.BOX;
  box.dimensions = {0.1, 0.2, 0.2};  // size in meters (x, y, z)

  // Define the box pose
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.17;
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.2;

  // Attach shape and pose to the object
  collision_object.primitives.push_back(box);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // Apply the collision object
  planning_scene_interface.applyCollisionObjects({collision_object});

  // start path planning
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group_interface.setPlanningTime(10.0);  // Set 10-second timeout
  moveit::core::MoveItErrorCode plan_state = move_group_interface.plan(my_plan);
  bool success = (plan_state == moveit::core::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(logger, "Motion Planning Request: %s", moveit::core::error_code_to_string(plan_state).c_str());
  if (true == success)
  {
    splitExecuteTrajectory(move_group_node, logger, my_plan);
    moveit::core::MoveItErrorCode exec_status = move_group_interface.execute(my_plan);
    RCLCPP_INFO(logger, "Execution Request: %s", moveit::core::error_code_to_string(exec_status).c_str());
  }
  
  // RCLCPP_INFO(logger, "Visualizing plan 2 (joint space goal)");
  
  // visualize the plan in RViz:
  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "agent_n_base_link", "visualization_marker_array",
                                                      move_group_interface.getRobotModel());

  visual_tools.deleteAllMarkers();

  /* Remote control is an introspection tool that allows users to step through a high level script */
  /* via buttons and keyboard shortcuts in RViz */
  // visual_tools.loadRemoteControl();
  // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);

  visual_tools.trigger();
  // visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  
  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}