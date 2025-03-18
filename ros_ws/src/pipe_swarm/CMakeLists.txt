cmake_minimum_required(VERSION 3.8)
project(pipe_swarm)

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
find_package(nav_msgs REQUIRED)
find_package(pipe_swarm_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

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
    rviz
    models
    config
    worlds
  DESTINATION share/${PROJECT_NAME}/
)

#python packages
ament_python_install_package(${PROJECT_NAME})

#install python nodes
install(
  PROGRAMS
    py_src/python_talker.py
    py_src/link_service.py
    py_src/centre_robot.py
    py_src/move_robot.py
    py_src/move_using_lidar.py
    py_src/move_robot_with_imu.py
    py_src/joint_control.py
    py_src/alignment_topic.py
    py_src/connect_robots.py
  DESTINATION lib/${PROJECT_NAME}/  
)

ament_package()
