cmake_minimum_required(VERSION 3.8)
project(hikrobot_camera)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 查找海康机器人 MVS SDK（模块位于 cmake/FindMVS.cmake）
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
find_package(MVS REQUIRED)

add_executable(camera_node
  src/main.cpp
  src/camera_node.cpp
  src/mvs_camera.cpp
)
target_compile_features(camera_node PRIVATE cxx_std_17)
target_include_directories(camera_node PRIVATE include)
ament_target_dependencies(camera_node
  rclcpp
  sensor_msgs
)

# 链接 MVS SDK（导入目标会同时携带头文件目录）。
# 注意：ament_target_dependencies 内部使用 target_link_libraries 的
# 旧式（plain）签名，因此这里不能混用 PRIVATE/PUBLIC 关键字。
target_link_libraries(camera_node MVS::MVS)

# 将 SDK 库目录写入安装后的 rpath，运行时可自动找到 libMvCameraControl.so。
# 传输层 .cti 等文件仍需依赖 LD_LIBRARY_PATH，见 README / 运行时说明。
set_target_properties(camera_node PROPERTIES
  INSTALL_RPATH_USE_LINK_PATH TRUE
)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(camera_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS camera_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
