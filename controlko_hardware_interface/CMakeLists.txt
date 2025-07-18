cmake_minimum_required(VERSION 3.8)
project(controlko_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(
  controlko_hardware_interface
  SHARED
  src/controlko_hardware_interface.cpp
)
target_include_directories(
  controlko_hardware_interface
  PUBLIC
  include
)
ament_target_dependencies(
  controlko_hardware_interface
  hardware_interface
  rclcpp
  rclcpp_lifecycle
)
# pluginlibがブーストを使用しないようにします
target_compile_definitions(controlko_hardware_interface PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# pluginをエクスポート
pluginlib_export_plugin_description_file(
  hardware_interface
  controlko_hardware_interface.xml
)

install(
  TARGETS controlko_hardware_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
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

# （オプション）ハードウェアインターフェイスのライブラリをament_export_librariesに追加します
ament_export_libraries(controlko_hardware_interface)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
