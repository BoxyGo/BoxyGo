cmake_minimum_required(VERSION 3.5)
project(boxygo)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# wykryj dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(gazebo_ros2_control REQUIRED)
find_package(ros2_control REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(moteus_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(
  PROGRAMS
    boxygo/wheel_moteus.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME wheel_moteus
)

install(
  PROGRAMS
    boxygo/multi_wheel_moteus.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME multi_wheel_moteus
)

install(
  PROGRAMS
    boxygo/multi_wheel_from_cmdvel_out.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME multi_wheel_from_cmdvel_out
)

install(
  FILES
    package.xml
  DESTINATION share/${PROJECT_NAME}
)
install(
  FILES
    resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

install(
  DIRECTORY
    urdf
    worlds
    launch
    config
    meshes
    maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
