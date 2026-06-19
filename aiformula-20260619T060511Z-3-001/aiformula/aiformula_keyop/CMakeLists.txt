cmake_minimum_required(VERSION 3.8)
project(aiformula_keyop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(oit_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

# include
include_directories(include)

# library
add_library(aiformula_keyop SHARED src/aiformula_keyop.cpp)
ament_target_dependencies(aiformula_keyop
  geometry_msgs
  oit_interfaces
  rclcpp
  rclcpp_components
)

rclcpp_components_register_node(
  aiformula_keyop
  PLUGIN "aiformula_keyop::KeyOp"
  EXECUTABLE aiformula_keyop_node
)

# install library
install(TARGETS aiformula_keyop
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

# install config if exists
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
