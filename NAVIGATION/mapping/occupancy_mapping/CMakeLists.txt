cmake_minimum_required(VERSION 3.8)
project(occupancy_mapping)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 使用了 std::filesystem
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ============================================================
# Dependencies
# ============================================================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(pluginlib REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(trailblazer_map_interfaces REQUIRED)


# ============================================================
# OrdinaryOccupancyMapping plugin
#
# 这个动态库包含：
#   OrdinaryOccupancyMapping
#   GridGeometry
#   StaticMapIO
#
# pluginlib 会在运行时加载它
# ============================================================

add_library(ordinary_occupancy_mapping SHARED
  src/ordinary_occupancy_mapping.cpp
  src/grid_geometry.cpp
  src/static_map_io.cpp
)

target_include_directories(ordinary_occupancy_mapping PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ordinary_occupancy_mapping
  rclcpp
  rclcpp_lifecycle
  pluginlib

  sensor_msgs
  nav_msgs
  std_srvs

  trailblazer_map_interfaces
)


# ============================================================
# OccupancyMappingServer component
#
# Server 自己不包含建图算法，
# 只负责 lifecycle + pluginlib 插件加载
# ============================================================

add_library(occupancy_mapping_server_component SHARED
  src/occupancy_mapping_server.cpp
)

target_include_directories(occupancy_mapping_server_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(occupancy_mapping_server_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  pluginlib
)


# ============================================================
# Register ROS2 Component
#
# 同时生成 standalone executable:
#
#   ros2 run occupancy_mapping occupancy_mapping_server
#
# 所以 bringup.launch.py 仍然可以直接使用 Node(...)
# ============================================================

rclcpp_components_register_node(
  occupancy_mapping_server_component
  PLUGIN "occupancy_mapping::OccupancyMappingServer"
  EXECUTABLE occupancy_mapping_server
)


# ============================================================
# Export pluginlib plugin description
# ============================================================

pluginlib_export_plugin_description_file(
  occupancy_mapping
  occupancy_mapping.xml
)


# ============================================================
# Install
# ============================================================

install(
  TARGETS
    ordinary_occupancy_mapping
    occupancy_mapping_server_component

  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)


# ============================================================
# Testing
# ============================================================

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()


# ============================================================
# Package
# ============================================================

ament_package()