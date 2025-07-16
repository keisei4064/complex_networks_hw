cmake_minimum_required(VERSION 3.8)
project(controlko_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  # 警告を抑制
  add_compile_options(-Wno-deprecated-declarations)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_srvs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

generate_parameter_library(
  displacement_controller_parameters
  src/displacement_controller.yaml
  include/controlko_controllers/varidate_displacement_controller_parameters.hpp
)
add_library(displacement_controller SHARED src/displacement_controller.cpp)
target_include_directories(displacement_controller PRIVATE include)
target_link_libraries(displacement_controller displacement_controller_parameters)
ament_target_dependencies(displacement_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_compile_definitions(displacement_controller PRIVATE "DISPLACEMENT_CONTROLLER_BUILDING_DLL")

# target_compile_definitions(displacement_controller PRIVATE " CONTROLKO_CONTROLLERS_BUILDING_LIBRARY ")
pluginlib_export_plugin_description_file(controller_interface controlko_controllers.xml)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS displacement_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)

  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(
  include
)
ament_export_dependencies(
)
ament_export_libraries(
  displacement_controller
)

ament_package()
