cmake_minimum_required(VERSION 3.5)
project(multi_cloud_fusion)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters registration)
find_package(OpenMP REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# ===================== cloud_fusion_node =====================

add_library(cloud_fusion_node_lib
  src/cloud_fusion_node.cpp
)

ament_target_dependencies(cloud_fusion_node_lib
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  message_filters
  pcl_conversions
)

target_link_libraries(cloud_fusion_node_lib
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

add_executable(cloud_fusion_node
  src/main.cpp
)

target_link_libraries(cloud_fusion_node
  cloud_fusion_node_lib
)

ament_target_dependencies(cloud_fusion_node
  rclcpp
)

# ===================== top_view_projector_node =====================

add_library(top_view_projector_node_lib
  src/top_view_projector_node.cpp
)

ament_target_dependencies(top_view_projector_node_lib
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(top_view_projector_node_lib
  ${PCL_LIBRARIES}
  OpenMP::OpenMP_CXX
)

add_executable(top_view_projector_node
  src/top_view_projector_main.cpp
)

target_link_libraries(top_view_projector_node
  top_view_projector_node_lib
)

ament_target_dependencies(top_view_projector_node
  rclcpp
)

# ===================== icp_tf_node (ICP-based TF estimation) =====================

add_executable(icp_tf_node
  src/icp_tf_node.cpp
)

ament_target_dependencies(icp_tf_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  message_filters
  pcl_conversions
  geometry_msgs
)

target_link_libraries(icp_tf_node
  ${PCL_LIBRARIES}
)

# ===================== frame_tf_saver_node (TF -> YAML) =====================

add_executable(frame_tf_saver_node
  src/frame_tf_saver_node.cpp
)

ament_target_dependencies(frame_tf_saver_node
  rclcpp
  tf2
  tf2_ros
  geometry_msgs
)

# ===================== Installation =====================

install(TARGETS
  cloud_fusion_node
  cloud_fusion_node_lib
  top_view_projector_node
  top_view_projector_node_lib
  icp_tf_node
  frame_tf_saver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
