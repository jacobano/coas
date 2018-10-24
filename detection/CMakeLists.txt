cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)
project(detection)
#add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  nodelet
  sensor_msgs
  pcl_ros
  pcl_conversions
  geometry_msgs
  nav_msgs
  std_msgs
  message_generation
  jsk_recognition_msgs
)
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  VectorPointCloud.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES detection
  CATKIN_DEPENDS roscpp rospy message_runtime jsk_recognition_msgs
  DEPENDS EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#find_package(octomap REQUIRED)
#include_directories(${OCTOMAP_INCLUDE_DIRS})
#link_directories(${OCTOMAP_LIBRARY_DIRS})
#target_link_libraries(${OCTOMAP_LIBRARIES})

include_directories(src ${EIGEN3_INCLUDE_DIR})

add_library(euclidean_clusterer src/euclidean_clusterer.cpp)
add_dependencies(euclidean_clusterer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(euclidean_clusterer_node src/euclidean_clusterer_node.cpp)
target_link_libraries(euclidean_clusterer_node euclidean_clusterer bounding_boxes ${catkin_LIBRARIES})
add_dependencies(euclidean_clusterer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(bounding_boxes src/bounding_boxes.cpp)
add_dependencies(bounding_boxes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(bounding_boxes_node src/bounding_boxes_node.cpp)
target_link_libraries(bounding_boxes_node bounding_boxes ${catkin_LIBRARIES})
add_dependencies(bounding_boxes_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(matching src/matching.cpp)
add_dependencies(matching ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(matching_node src/matching_node.cpp)
target_link_libraries(matching_node matching ${catkin_LIBRARIES})
add_dependencies(matching_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
