cmake_minimum_required(VERSION 3.8)
project(global_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(pluginlib REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(trailblazer_map_interfaces REQUIRED)
find_package(field_map_builder REQUIRED)

find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------
# Common utility library
# ---------------------------------------------------------------------------
add_library(path_utils SHARED
  src/utility/path_utils.cpp
)

# 使本包内部也能使用 global_planner::path_utils 名称
add_library(global_planner::path_utils ALIAS path_utils)

target_include_directories(path_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(path_utils Eigen3::Eigen)

# ---------------------------------------------------------------------------
# Plugin library: global_planner
# ---------------------------------------------------------------------------
add_library(global_planner SHARED
  src/astar_esdf_global_planner.cpp
)

target_include_directories(global_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(global_planner
  rclcpp
  rclcpp_lifecycle
  pluginlib
  nav_msgs
  geometry_msgs
  trailblazer_map_interfaces
  field_map_builder
)

target_link_libraries(global_planner 
  Eigen3::Eigen 
  path_utils
)

# ---------------------------------------------------------------------------
# Component node library: GlobalPlannerServer
# ---------------------------------------------------------------------------
add_library(global_planner_server_component SHARED
  src/global_planner_server.cpp
)

target_include_directories(global_planner_server_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(global_planner_server_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  pluginlib
)

# Register component and create a normal executable for easy launch/debug
rclcpp_components_register_node(global_planner_server_component
  PLUGIN "global_planner::GlobalPlannerServer"
  EXECUTABLE global_planner_server
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS
  path_utils
  global_planner
  global_planner_server_component
  EXPORT export_global_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES global_planner.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin description for pluginlib
pluginlib_export_plugin_description_file(
  global_planner
  global_planner.xml
)

# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------
ament_export_targets(export_global_planner HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  pluginlib
  nav_msgs
  geometry_msgs
  trailblazer_map_interfaces
  field_map_builder
)

ament_package()