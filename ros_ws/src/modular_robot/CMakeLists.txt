cmake_minimum_required(VERSION 3.8)
project(modular_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

install(
  DIRECTORY
    urdf
    launch
    config
    meshes
    worlds
  DESTINATION share/${PROJECT_NAME}/
)

# modular_robot
add_executable(modular_robot src/modular_robot.cpp)
target_include_directories(
  modular_robot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(modular_robot 
  PUBLIC c_std_99 cxx_std_17 # Require C99 and C++17
) 
ament_target_dependencies(
  modular_robot
  "rclcpp"
  "std_msgs"
  "gazebo_msgs"
)

# modular_robot_moveit
add_executable(modular_robot_moveit src/modular_robot_moveit.cpp)
target_include_directories(
  modular_robot_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(modular_robot_moveit 
  PUBLIC c_std_99 cxx_std_17 # Require C99 and C++17
)
ament_target_dependencies(
  modular_robot_moveit
  "moveit_ros_planning_interface"
  "moveit_visual_tools"
  "rclcpp"
)

install(
  TARGETS
    modular_robot
    modular_robot_moveit
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
