cmake_minimum_required(VERSION 3.0.2)
project(d2p)

## 使用 C++11 或更新版本
add_compile_options(-std=c++11)

## 查找 catkin 组件和库
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  vision_msgs
  cv_bridge
  image_transport
  message_filters
  tf2
  tf2_ros
  eigen_conversions
)

## 查找 OpenCV
find_package(OpenCV REQUIRED)

## 查找 Eigen
find_package(Eigen3 REQUIRED)

## 生成 catkin 包
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs nav_msgs 
                 visualization_msgs vision_msgs cv_bridge image_transport 
                 message_filters tf2 tf2_ros eigen_conversions
  DEPENDS EIGEN3
)

## 包含头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

## 添加可执行文件
add_executable(target_detector
  src/main.cpp
  src/target_detector.cpp
  src/Tracker.cpp
  src/TrackerState.cpp
)

## 指定可执行文件需要链接的库
target_link_libraries(target_detector
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

install(TARGETS target_detector
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
