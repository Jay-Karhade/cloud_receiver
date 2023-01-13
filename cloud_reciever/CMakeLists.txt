cmake_minimum_required(VERSION 2.8.3)
project(cloud_reciever)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
set(CMAKE_CXX_STANDARD 17)

# Uncomment line above if using Ubuntu 20.04
set(CMAKE_CXX_FLAGS -DFMT_HEADER_ONLY)
set(CMAKE_BUILD_TYPE "RelWithDebInfo")

add_definitions(-w)

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  image_transport

  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs

  map_database
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(Ceres REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}

)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)

add_executable(receive_clouds src/receive_clouds.cpp)
target_link_libraries(receive_clouds
  ${catkin_LIBRARIES}
  ${base_LIBRARIES})