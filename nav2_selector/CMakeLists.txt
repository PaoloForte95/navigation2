cmake_minimum_required(VERSION 3.8)
project(nav2_selector)
add_compile_options(-std=c++17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav2_core REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_collision_monitor REQUIRED)

include_directories(
  include
)

set(executable_name selector_server)
set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  nav2_core
  std_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  pluginlib
  nav2_costmap_2d
  nav2_util
  nav2_collision_monitor
)

# Selector server
add_library(${library_name} SHARED
  src/selector_server.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(${executable_name}
  src/main.cpp
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${executable_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "nav2_selector::SelectorServer")

# Costmap selector
add_library(costmap_selector SHARED
  src/selectors/costmap_selector.cpp
)

ament_target_dependencies(costmap_selector
  ${dependencies}
)

# Install
install(TARGETS ${library_name} costmap_selector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(costmap_selector ${library_name})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

ament_package()