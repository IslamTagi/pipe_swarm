#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <geometry_msgs/msg/twist.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

void splitExecuteTrajectory(std::shared_ptr<rclcpp::Node> move_group_node,
                            // rclcpp::Logger logger,
                            moveit::planning_interface::MoveGroupInterface::Plan plan)
{
  auto trajectory_points = plan.trajectory_.joint_trajectory.points;

  // create publishers for each joint
  auto agent_1_pub = move_group_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/agent_1/female_joint_trajectory_controller/joint_trajectory", 10);
  
    auto agent_2_pub = move_group_node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/agent_2/female_joint_trajectory_controller/joint_trajectory", 10);
  
  // set joint names
  trajectory_msgs::msg::JointTrajectory traj_agent_1;
  traj_agent_1.joint_names.push_back("base_female_joint");
  trajectory_msgs::msg::JointTrajectory traj_agent_2;
  traj_agent_2.joint_names.push_back("base_female_joint");

  const auto& points = plan.trajectory_.joint_trajectory.points;

  for (const auto& point : points)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point_agent_1;
    point_agent_1.positions.push_back(-point.positions[1]); // invert position
    point_agent_1.time_from_start = point.time_from_start;
    traj_agent_1.points.push_back(point_agent_1);
    
    trajectory_msgs::msg::JointTrajectoryPoint point_agent_2;
    point_agent_2.positions.push_back(-point.positions[2]); // invert position
    point_agent_2.time_from_start = point.time_from_start;
    traj_agent_2.points.push_back(point_agent_2);
  }

  agent_1_pub->publish(traj_agent_1);
  agent_2_pub->publish(traj_agent_2);
  
}

void transformPrismaticCmd(std::shared_ptr<rclcpp::Node> move_group_node,
                        //   rclcpp::Logger logger,
                          moveit::planning_interface::MoveGroupInterface::Plan plan)
{
  // Create a publisher for cmd_vel
  auto cmd_vel_pub = move_group_node->create_publisher<geometry_msgs::msg::Twist>("/agent_0/cmd_vel", 10);

  // Previous values for calculation
  double prev_position = plan.trajectory_.joint_trajectory.points.front().positions[0];
  // Convert time_from_start (builtin_interfaces::msg::Duration) to rclcpp::Duration
  rclcpp::Duration prev_time(
    plan.trajectory_.joint_trajectory.points.front().time_from_start.sec,
    plan.trajectory_.joint_trajectory.points.front().time_from_start.nanosec
  );


  for (size_t i = 0; i <= plan.trajectory_.joint_trajectory.points.size(); i=i+10)
  {
    const auto& point = plan.trajectory_.joint_trajectory.points[i];
    // RCLCPP_INFO(logger, "Point %zu: time_from_start.sec = %d, nanosec = %u", 
    //   i, point.time_from_start.sec, point.time_from_start.nanosec);


    double current_position = point.positions[0]; // assuming index 0 is the prismatic joint

    // Convert time_from_start to rclcpp::Duration
    rclcpp::Duration current_time(
      point.time_from_start.sec,
      point.time_from_start.nanosec
    );

    // Calculate time delta
    double dt = (current_time - prev_time).seconds();
    if (dt <= 0)
    {
    //   RCLCPP_WARN(logger, "Delta time is non-positive at point %zu, skipping", i);
      continue;
    }

    // Calculate velocity
    double velocity = (current_position - prev_position) / dt;

    // Prepare Twist message
    geometry_msgs::msg::Twist cmd_vel_msg;
    cmd_vel_msg.linear.x = velocity;
    cmd_vel_msg.angular.z = 0.0;

    // Publish
    cmd_vel_pub->publish(cmd_vel_msg);

    // Wait for dt duration to mimic trajectory timing
    rclcpp::Rate rate(1.0 / dt);
    rate.sleep();

    // Update previous values
    prev_position = current_position;
    prev_time = current_time;
  }


  // Optionally, send zero velocity at end
  geometry_msgs::msg::Twist stop_msg;
  cmd_vel_pub->publish(stop_msg);
}
class ModularRobotMover : public rclcpp::Node
{
    public:

        typedef rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr BoolPublisher;
        typedef rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr Float64Subscriber;
        typedef rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr BoolSubscriber;

        // moveit2 parameters
        std::string PLANNING_GROUP = "mobile_chain";

        ModularRobotMover() : Node(
            "modular_robot_mover",
            rclcpp::NodeOptions()
                .automatically_declare_parameters_from_overrides(true)
                .parameter_overrides({{"use_sim_time", rclcpp::ParameterValue(true)}})
        )
        {
            // construct
            RCLCPP_INFO(this->get_logger(), "Modular Robot Mover Active");
            _plan_success_publisher = create_publisher<std_msgs::msg::Bool>(
                                    "/plan_successful", 
                                    10 );
            
                                    
            _goal_state_subscriber = create_subscription<std_msgs::msg::Float64MultiArray>(
                                    "/modular_goal_state",
                                    20,
                                    std::bind(&ModularRobotMover::_planToGoalState, this, std::placeholders::_1));
            _goal_execute_subscriber = create_subscription<std_msgs::msg::Bool>(
                                    "/execute_goal_state",
                                    20,
                                    std::bind(&ModularRobotMover::_moveToGoalState, this, std::placeholders::_1));
            _obstacle_position_subscriber = create_subscription<std_msgs::msg::Float64MultiArray>(
                                            "/define_pipe_positions",
                                            20,
                                            std::bind(&ModularRobotMover::_drawPipeShapes, this, std::placeholders::_1));
            _plan_successful = false;
        }

        void initialise(void)
        {
            _initialiseMoveGroupInterface(); // updating to current joint positions as target joint position
        }

        bool startPlan(void)
        {
            _move_group_interface_ptr->setPlanningTime(10.0);  // Set 10-second timeout
            moveit::core::MoveItErrorCode plan_state = _move_group_interface_ptr->plan(_plan);
            RCLCPP_INFO(this->get_logger(), "Motion Planning Request: %s", moveit::core::error_code_to_string(plan_state).c_str());
            _plan_successful = (plan_state == moveit::core::MoveItErrorCode::SUCCESS);
            
            std_msgs::msg::Bool success;
            success.data = _plan_successful;
            _plan_success_publisher->publish(success);
            return _plan_successful;
        }

        bool executePlan()
        {
            // splitExecuteTrajectory(this, /*logger,*/ _plan);
            // transformPrismaticCmd(this, /*logger*/, _plan);
            moveit::core::MoveItErrorCode execute_state = _move_group_interface_ptr->execute(_plan);
            RCLCPP_INFO(this->get_logger(), "Execution Request: %s", moveit::core::error_code_to_string(execute_state).c_str());
            bool success = (execute_state == moveit::core::MoveItErrorCode::SUCCESS);
            return success;
        }

        void setJointPosition(std::vector<double> positions)
        {
            for(uint8_t i=0; i<=_target_joint_positions.size(); i++)
            {
                _target_joint_positions[i] = positions[i];
            }
            _move_group_interface_ptr->setJointValueTarget(_target_joint_positions);
        }

    private:
        BoolPublisher _plan_success_publisher;

        Float64Subscriber _goal_state_subscriber;
        BoolSubscriber _goal_execute_subscriber;
        Float64Subscriber _obstacle_position_subscriber;

        bool _plan_successful;

        // moveit2 parameters
        moveit::core::RobotStatePtr _current_state_ptr;
        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> _move_group_interface_ptr;
        const moveit::core::JointModelGroup* _joint_model_group_ptr;
        std::vector<double> _target_joint_positions;

        // planning scene
        moveit::planning_interface::PlanningSceneInterface _planning_scene_interface;
        moveit_msgs::msg::CollisionObject _collision_object;
        moveit::planning_interface::MoveGroupInterface::Plan _plan;

        void _planToGoalState(const std_msgs::msg::Float64MultiArray::SharedPtr goal_state)
        {
            std::vector<double> new_pos;
            for (size_t i = 0; i <= goal_state->data.size(); i++) {
                new_pos.push_back(goal_state->data[i]);
                RCLCPP_INFO(this->get_logger(), "%f", goal_state->data[i]);
            }
            setJointPosition(new_pos);
            if(true == startPlan())
            {
                executePlan();
            }
        }
        
        void _moveToGoalState(const std_msgs::msg::Bool::SharedPtr goal_state)
        {
            if(true == _plan_successful && true == goal_state->data)
            {
                executePlan();
            }
        }
        
        void _drawPipeShapes(const std_msgs::msg::Float64MultiArray::SharedPtr pipe_array)
        {
            double length       = pipe_array->data[0] / 100;
            double radius       = pipe_array->data[1] / 100;
            double thickness    = pipe_array->data[2] / 100;
            double x1           = pipe_array->data[3] / 100;
            double y1           = pipe_array->data[4] / 100;
            double x2           = pipe_array->data[5] / 100;
            double y2           = pipe_array->data[6] / 100;

            std::vector<geometry_msgs::msg::Pose> pipe_positions(4);
            double robot_height = 0.062 + 0.015 + 0.02; // base_h + wheel_r + wheel clearance offset
            double robot_tail = 0.08;

            // pipe 1 bottom
            pipe_positions[0].orientation.w = 1.0;
            pipe_positions[0].position.x = x1 + (length / 2.0) - robot_tail;
            pipe_positions[0].position.y = 0.0;
            pipe_positions[0].position.z = (y1 - thickness / 2.0) - robot_height/2;
            
            // pipe 1 top
            pipe_positions[1].orientation.w = 1.0;
            pipe_positions[1].position.x = x1 + (length / 2.0) - robot_tail;
            pipe_positions[1].position.y = 0.0;
            pipe_positions[1].position.z = (y1 - thickness / 2.0) + radius*2 - robot_height/2 + thickness;

            // pipe 2 bottom
            pipe_positions[2].orientation.w = 1.0;
            pipe_positions[2].position.x = x2 + (length / 2.0) - robot_tail;
            pipe_positions[2].position.y = 0.0;
            pipe_positions[2].position.z = (y2 - thickness / 2.0) - robot_height/2;
            
            // pipe 2 top
            pipe_positions[3].orientation.w = 1.0;
            pipe_positions[3].position.x = x2 + (length / 2.0) - robot_tail;
            pipe_positions[3].position.y = 0.0;
            pipe_positions[3].position.z = (y2 - thickness / 2.0) + radius*2 - robot_height/2 + thickness;
            _definePlanningSceneInterface("agent_n_base_link", pipe_positions, length, thickness);

        }

        void _initialiseMoveGroupInterface(void)
        {
            _move_group_interface_ptr = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                shared_from_this(), PLANNING_GROUP);
            _current_state_ptr = _move_group_interface_ptr->getCurrentState(10);
            _joint_model_group_ptr = _move_group_interface_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
            _current_state_ptr->copyJointGroupPositions(_joint_model_group_ptr, _target_joint_positions);
            _move_group_interface_ptr->setJointValueTarget(_target_joint_positions);
            _move_group_interface_ptr->setMaxVelocityScalingFactor(0.05);
            _move_group_interface_ptr->setMaxAccelerationScalingFactor(0.05);
        }

        void _definePlanningSceneInterface(std::string frame_id, std::vector<geometry_msgs::msg::Pose> pipe_poses, 
                                            double length, double thickness)
        {
            _collision_object.header.frame_id = frame_id;  // Use your planning frame
            _collision_object.id = "pipe_block";
            
            _collision_object.primitives.clear();
            _collision_object.primitive_poses.clear();

            // Define the box shape
            shape_msgs::msg::SolidPrimitive pipe_shape;
            pipe_shape.type = pipe_shape.BOX;
            pipe_shape.dimensions = {length, 0.2, thickness};  // size in meters (x, y, z)

            // Define the box pose

            // Attach shape and pose to the object
            for (const auto& pose : pipe_poses)
            {
                _collision_object.primitives.push_back(pipe_shape);
                _collision_object.primitive_poses.push_back(pose);
                _collision_object.operation = _collision_object.ADD;
            }

            // Apply the collision object
            _planning_scene_interface.applyCollisionObjects({_collision_object});
        }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto mover_node = std::make_shared<ModularRobotMover>();

    // Start executor in background
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(mover_node);
    std::thread exec_thread([&executor]() { executor.spin(); });

    // Initialise safely, now subscriptions and services are live
    mover_node->initialise();

    // Join executor thread
    exec_thread.join();

    rclcpp::shutdown();
    return 0;
}
