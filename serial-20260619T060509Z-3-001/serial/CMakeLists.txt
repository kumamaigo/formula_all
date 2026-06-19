cmake_minimum_required(VERSION 3.5)
project(serial)

find_package(ament_cmake REQUIRED)

# ソースファイル
set(serial_SRCS
  src/serial.cc
  src/impl/unix.cc
  src/impl/list_ports/list_ports_linux.cc
)

# 共有ライブラリを作成
add_library(${PROJECT_NAME} SHARED ${serial_SRCS})

# ヘッダパス
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ament export
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# install
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)

ament_package()
