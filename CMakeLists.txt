cmake_minimum_required(VERSION 3.8)
project(my_moving_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(my_moving_robot_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)

add_executable(moving_robot_client src/moving_robot_client.cpp)
ament_target_dependencies(moving_robot_client rclcpp rclcpp_action my_moving_robot_interfaces)

add_executable(moving_robot_server src/moving_robot_server.cpp)
ament_target_dependencies(moving_robot_server rclcpp rclcpp_action my_moving_robot_interfaces)

install(TARGETS
  moving_robot_client
  moving_robot_server
  
  DESTINATION lib/${PROJECT_NAME}/
)


ament_package()
