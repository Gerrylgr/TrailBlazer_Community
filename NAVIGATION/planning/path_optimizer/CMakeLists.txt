cmake_minimum_required(VERSION 3.8)
project(path_optimizer)

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
find_package(field_map_builder REQUIRED)

find_package(Eigen3 REQUIRED)

# ---------------------------------------------------------------------------
# Common utility library
# ---------------------------------------------------------------------------
add_library(optimization_utils SHARED
  src/utility/uniform_bspline.cpp
)

# 使本包内部也能使用 path_optimizer::optimization_utils 名称
add_library(path_optimizer::optimization_utils ALIAS optimization_utils)

target_include_directories(optimization_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(optimization_utils Eigen3::Eigen)

# ---------------------------------------------------------------------------
# Plugin library: path_optimizer
# ---------------------------------------------------------------------------
add_library(path_optimizer SHARED
  src/lbfgs_bspline_optimizer.cpp
  src/lbfgs_backend.cpp
)

target_include_directories(path_optimizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/LBFGS-Lite/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(path_optimizer
  rclcpp
  rclcpp_lifecycle
  pluginlib
  nav_msgs
  trailblazer_map_interfaces
  field_map_builder
)

target_link_libraries(path_optimizer 
  Eigen3::Eigen 
  optimization_utils
)

# ---------------------------------------------------------------------------
# Component node library: PathOptimizerServer
# ---------------------------------------------------------------------------
add_library(path_optimizer_server_component SHARED
  src/path_optimizer_server.cpp
)

target_include_directories(path_optimizer_server_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(path_optimizer_server_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  pluginlib
)

# Register component and create a normal executable for easy launch/debug
rclcpp_components_register_node(path_optimizer_server_component
  PLUGIN "path_optimizer::PathOptimizerServer"
  EXECUTABLE path_optimizer_server
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS
  optimization_utils
  path_optimizer
  path_optimizer_server_component
  EXPORT export_path_optimizer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES path_optimizer.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin description for pluginlib
pluginlib_export_plugin_description_file(
  path_optimizer
  path_optimizer.xml
)

# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------
ament_export_targets(export_path_optimizer HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  pluginlib
  nav_msgs
  trailblazer_map_interfaces
  field_map_builder
)

ament_package()
