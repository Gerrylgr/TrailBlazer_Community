cmake_minimum_required(VERSION 3.8)
project(field_map_builder)

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

find_package(trailblazer_map_interfaces REQUIRED)

# ---------------------------------------------------------------------------
# Common ESDF query library
# ---------------------------------------------------------------------------
add_library(esdf_query SHARED
  src/utility/esdf_query.cpp
)

# 使本包内部也能使用 field_map_builder::esdf_query 名称
add_library(field_map_builder::esdf_query ALIAS esdf_query)

target_include_directories(esdf_query PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(esdf_query
  trailblazer_map_interfaces
)

# ---------------------------------------------------------------------------
# Plugin library: field_map_builder
# ---------------------------------------------------------------------------
add_library(field_map_builder SHARED
  src/esdf_map_builder.cpp
)

target_include_directories(field_map_builder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(field_map_builder
  rclcpp
  rclcpp_lifecycle
  pluginlib
  nav_msgs
  trailblazer_map_interfaces
)

# ---------------------------------------------------------------------------
# Component node library: FieldMapBuilderServer
# ---------------------------------------------------------------------------
add_library(field_map_builder_server_component SHARED
  src/field_map_builder_server.cpp
)

target_include_directories(field_map_builder_server_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(field_map_builder_server_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  pluginlib
)

# Register component and create a normal executable for easy launch/debug
rclcpp_components_register_node(field_map_builder_server_component
  PLUGIN "field_map_builder::FieldMapBuilderServer"
  EXECUTABLE field_map_builder_server
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS
  esdf_query
  field_map_builder
  field_map_builder_server_component
  EXPORT export_field_map_builder
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES field_map_builder.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin description for pluginlib
pluginlib_export_plugin_description_file(
  field_map_builder
  field_map_builder.xml
)

# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------
ament_export_targets(export_field_map_builder HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  pluginlib
  nav_msgs
  trailblazer_map_interfaces
)

ament_package()