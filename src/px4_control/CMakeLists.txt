cmake_minimum_required(VERSION 3.8)
project(px4_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(px4_control_node src/px4_control_node.cpp)
ament_target_dependencies(px4_control_node rclcpp px4_msgs)

add_executable(px4_control_node_optimized src/px4_control_node_optimized.cpp)
ament_target_dependencies(px4_control_node_optimized rclcpp px4_msgs)

add_executable(px4_control_node_services src/px4_control_node_services.cpp)
ament_target_dependencies(px4_control_node_services rclcpp px4_msgs)

add_executable(px4_control_node_joystick src/px4_control_node_joystick.cpp)
ament_target_dependencies(px4_control_node_joystick rclcpp px4_msgs sensor_msgs)

add_executable(px4_control_node_manual src/px4_control_node_manual.cpp)
ament_target_dependencies(px4_control_node_manual rclcpp px4_msgs sensor_msgs)

add_executable(px4_control_node_manual_v2 src/px4_control_node_manual_v2.cpp)
ament_target_dependencies(px4_control_node_manual_v2 rclcpp px4_msgs sensor_msgs)

add_executable(px4_control_node_manual_v3 src/px4_control_node_manual_v3.cpp)
ament_target_dependencies(px4_control_node_manual_v3 rclcpp px4_msgs sensor_msgs)

add_executable(px4_control_node_cmd_vel src/px4_control_node_cmd_vel.cpp)
ament_target_dependencies(px4_control_node_cmd_vel rclcpp px4_msgs geometry_msgs)

add_executable(px4_control_node_gui src/px4_control_node_gui.cpp)
ament_target_dependencies(px4_control_node_gui rclcpp px4_msgs)
target_link_libraries(px4_control_node_gui Qt5::Widgets)

install(TARGETS
  px4_control_node
  px4_control_node_optimized
  px4_control_node_services
  px4_control_node_joystick
  px4_control_node_manual
  px4_control_node_manual_v2
  px4_control_node_gui
  px4_control_node_manual_v3
  px4_control_node_cmd_vel
  DESTINATION lib/${PROJECT_NAME})

ament_package()
