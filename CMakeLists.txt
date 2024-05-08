cmake_minimum_required(VERSION 3.0.2)
project(slam2d)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  rosbag
  tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3)
find_package(Ceres)
find_package(PCL)
find_package(TBB COMPONENTS tbbmalloc tbbmalloc_proxy tbb_preview)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES slam2d
 CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs std_msg tf
 DEPENDS EIGEN3 PCL
)

###########
## Build ##
###########

# Not all machines have <cpuid.h> available
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
file(WRITE ${CMAKE_BINARY_DIR}/test_cpuid.cpp "#include <cpuid.h>")
try_compile(HAS_CPUID ${CMAKE_BINARY_DIR} ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
file(REMOVE ${CMAKE_BINARY_DIR}/test_cpuid.cpp)
if(HAS_CPUID)
  add_compile_definitions(HAS_CPUID)
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  src
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(map2d STATIC src/map2d.cpp)
target_link_libraries(map2d ${PCL_LIBRARIES})

add_library(slam2d STATIC src/slam2d.cpp)
target_link_libraries(slam2d ${CERES_LIBRARIES} ${PCL_LIBRARIES} tbb map2d)

add_executable(slam2d_node  src/slam2d_node.cpp)
target_link_libraries(slam2d_node slam2d ${catkin_LIBRARIES} ${PCL_LIBRARIES})

