cmake_minimum_required(VERSION 3.8)
project(sushi_bot_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sushi_bot_interfaces REQUIRED)

add_library(sushi_bot_gazebo_plugin_pick_and_drop SHARED src/pick_and_drop_plugin.cpp)
ament_target_dependencies(sushi_bot_gazebo_plugin_pick_and_drop rclcpp gazebo_ros gazebo_msgs gazebo_dev std_srvs sushi_bot_interfaces)

install(TARGETS sushi_bot_gazebo_plugin_pick_and_drop
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_dependencies(rclcpp)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(gazebo_dev)
ament_export_dependencies(gazebo_ros)
ament_export_dependencies(gazebo_msgs)
ament_export_dependencies(std_srvs)

ament_export_libraries(sushi_bot_gazebo_plugin_pick_and_drop)

ament_package()
