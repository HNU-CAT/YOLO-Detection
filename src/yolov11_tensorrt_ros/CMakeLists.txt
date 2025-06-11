cmake_minimum_required(VERSION 3.0.2)
project(yolo_detector_node)

add_definitions(-std=c++11)
add_definitions(-DAPI_EXPORTS)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE release)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_filters
  roscpp
  rospy
  std_msgs
  vision_msgs
  message_generation
)

find_package(PCL 1.7 REQUIRED)
# ============= cuda ============
find_package(CUDA REQUIRED)
include_directories(/usr/local/cuda/include)
link_directories(/usr/local/cuda/lib64)
# ============= tensorrt ============
if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("Embed_platform on")
  include_directories(/usr/include/aarch64-linux-gnu)
  link_directories(/usr/lib/aarch64-linux-gnu)
else()
  message("Embed_platform off")
  include_directories(/usr/include/x86_64-linux-gnu)
  link_directories(/usr/lib/x86_64-linux-gnu)
endif()
# ============ opencv ============
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES libyolo_detector
   CATKIN_DEPENDS roscpp rospy std_msgs message_generation geometry_msgs
  #  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# ============ yolo infer shared lib ============
include_directories(${PROJECT_SOURCE_DIR}/include)
file(GLOB_RECURSE SRCS ${PROJECT_SOURCE_DIR}/src/*.cpp ${PROJECT_SOURCE_DIR}/src/*.cu)
cuda_add_library(libyolo_detector SHARED ${SRCS})
target_link_libraries(libyolo_detector cudart nvinfer nvonnxparser ${OpenCV_LIBS} ${catkin_LIBRARIES})

# ======== main execute file ========
add_executable(yolo_detector src/main.cpp)
target_link_libraries(yolo_detector libyolo_detector ${catkin_LIBRARIES})

