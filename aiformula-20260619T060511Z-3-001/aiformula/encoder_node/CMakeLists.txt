cmake_minimum_required(VERSION 3.8)
project(encoder_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  $<BUILD_INTERFACE:${serial_INCLUDE_DIRS}>
)

# --- 1つ目: encoder_driver ノード ---
add_executable(encoder_driver
  src/encoder_driver.cpp
  src/udp_sender.cpp        # encoder_driver が使うならここへ
)
ament_target_dependencies(encoder_driver
  rclcpp nav_msgs geometry_msgs serial sensor_msgs tf2 tf2_geometry_msgs
)
target_link_libraries(encoder_driver ${serial_LIBRARIES})

# --- 2つ目: fusion_node ノード ---
add_executable(fusion_node
  src/fusion_node.cpp
)
ament_target_dependencies(fusion_node
  rclcpp nav_msgs geometry_msgs serial sensor_msgs tf2 tf2_geometry_msgs
)
target_link_libraries(fusion_node ${serial_LIBRARIES})

install(TARGETS
  encoder_driver
  fusion_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
