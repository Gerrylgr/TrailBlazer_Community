cmake_minimum_required(VERSION 3.8)
project(ground_segmentor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED) 

# ---------------------------------------------------------------------------
# Plugin library: ground_segmentor
# ---------------------------------------------------------------------------
add_library(ground_segmentor SHARED
  src/ground_segmentor.cpp
)

target_include_directories(ground_segmentor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(ground_segmentor
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

target_link_libraries(ground_segmentor
  ${PCL_LIBRARIES}
)

# ---------------------------------------------------------------------------
# Component node library: GroundSegmentorServer
# ---------------------------------------------------------------------------
add_library(ground_segmentor_server_component SHARED
  src/ground_segmentor_server.cpp
)

target_include_directories(ground_segmentor_server_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(ground_segmentor_server_component
  rclcpp
  rclcpp_lifecycle
  rclcpp_components
  pluginlib
)

# Register component and create a normal executable for easy launch/debug
rclcpp_components_register_node(ground_segmentor_server_component
  PLUGIN "ground_segmentor::GroundSegmentorServer"
  EXECUTABLE ground_segmentor_server
)

# ---------------------------------------------------------------------------
# Install
# ---------------------------------------------------------------------------
install(TARGETS
  ground_segmentor
  ground_segmentor_server_component
  EXPORT export_ground_segmentor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES ground_segmentor.xml
  DESTINATION share/${PROJECT_NAME}
)

# Export plugin description for pluginlib
pluginlib_export_plugin_description_file(
  ground_segmentor
  ground_segmentor.xml
)

# ---------------------------------------------------------------------------
# Export
# ---------------------------------------------------------------------------
ament_export_targets(export_ground_segmentor HAS_LIBRARY_TARGET)

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