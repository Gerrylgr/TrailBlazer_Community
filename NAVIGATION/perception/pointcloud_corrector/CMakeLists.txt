cmake_minimum_required(VERSION 3.8)
project(pointcloud_corrector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(pluginlib REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

find_package(PCL REQUIRED COMPONENTS common filters)
find_package(pcl_conversions REQUIRED)

# ---------------------------------------------------------------------------
# Plugin library: pointcloud_corrector
# ---------------------------------------------------------------------------
add_library(pointcloud_corrector SHARED
  src/pointcloud_aligning.cpp
)

target_include_directories(pointcloud_corrector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(pointcloud_corrector
  rclcpp
  rclcpp_lifecycle
  pluginlib
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)

target_link_libraries(pointcloud_corrector
  ${PCL_LIBRARIES}
)

# ---------------------------------------------------------------------------
# Component node library: PointcloudCorrectorServer
# ---------------------------------------------------------------------------
add_library(pointcloud_corrector_server_component SHARED
  src/pointcloud_corrector_server.cpp
)

target_include_directories(pointcloud_corrector_server_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pointcloud_corrector_server_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  pluginlib
)

# Register component and create a normal executable for easy launch/debug
rclcpp_components_register_node(pointcloud_corrector_server_component
  PLUGIN "pointcloud_corrector::PointcloudCorrectorServer"
  EXECUTABLE pointcloud_corrector_server
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS
  pointcloud_corrector
  pointcloud_corrector_server_component
  EXPORT export_pointcloud_corrector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES pointcloud_corrector.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin description for pluginlib
pluginlib_export_plugin_description_file(
  pointcloud_corrector
  pointcloud_corrector.xml
)

# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------
ament_export_targets(export_pointcloud_corrector HAS_LIBRARY_TARGET)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  pluginlib
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  pcl_conversions
)

ament_package()