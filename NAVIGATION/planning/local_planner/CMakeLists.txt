cmake_minimum_required(VERSION 3.8)
project(local_planner)

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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(trailblazer_map_interfaces REQUIRED)
find_package(field_map_builder REQUIRED)

find_package(xtensor REQUIRED)

# ---------------------------------------------------------------------------
# MPPI source files
# ---------------------------------------------------------------------------
set(MPPI_SOURCES
  src/MPPI/mppi_planner.cpp

  src/MPPI/core/noise_generator.cpp
  src/MPPI/core/optimizer.cpp

  src/MPPI/tools/path_manager.cpp

  src/MPPI/critics/constraint_critic.cpp
  src/MPPI/critics/path_follow_critic.cpp
  src/MPPI/critics/path_align_critic.cpp
  src/MPPI/critics/esdf_footprint_critic.cpp
  src/MPPI/critics/path_angle_critic.cpp
  src/MPPI/critics/path_tracking_critic.cpp
)

# ---------------------------------------------------------------------------
# Plugin library: local_planner
# ---------------------------------------------------------------------------
add_library(local_planner SHARED
  ${MPPI_SOURCES}
)

target_include_directories(local_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(local_planner
  rclcpp
  pluginlib
  rclcpp_lifecycle

  nav_msgs
  geometry_msgs

  tf2
  tf2_ros
  tf2_geometry_msgs

  trailblazer_map_interfaces
  field_map_builder
)

# xtensor 不是 ament 包，使用 target_link_libraries。
#
# 不同发行版安装的 xtensor 导出 target 名可能不同，
# 这里同时兼容常见的两种名称。
if(TARGET xtensor)
  target_link_libraries(local_planner
    xtensor
  )
elseif(TARGET xtensor::xtensor)
  target_link_libraries(local_planner
    xtensor::xtensor
  )
else()
  message(FATAL_ERROR
    "xtensor was found, but no imported target named "
    "'xtensor' or 'xtensor::xtensor' is available."
  )
endif()

# MPPI 的循环计算量较大。
# Debug 构建不强制优化；Release 构建使用 O3。
target_compile_options(local_planner
  PRIVATE
    $<$<CONFIG:Release>:-O3>
)

# ---------------------------------------------------------------------------
# Component node library: LocalPlannerServer
# ---------------------------------------------------------------------------
add_library(local_planner_server_component SHARED
  src/local_planner_server.cpp
)

target_include_directories(local_planner_server_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(local_planner_server_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  pluginlib
)

# Register component and create a normal executable for easy launch/debug
rclcpp_components_register_node(local_planner_server_component
  PLUGIN "local_planner::LocalPlannerServer"
  EXECUTABLE local_planner_server
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS
  local_planner
  local_planner_server_component
  EXPORT export_local_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES local_planner.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin description for pluginlib
pluginlib_export_plugin_description_file(
  local_planner
  local_planner.xml
)

# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------
ament_export_targets(export_local_planner HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  pluginlib
  rclcpp_lifecycle

  nav_msgs
  geometry_msgs

  tf2
  tf2_ros
  tf2_geometry_msgs

  trailblazer_map_interfaces
  field_map_builder
)

ament_package()
