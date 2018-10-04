cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)
project(detection)
add_compile_options(-std=c++11)

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
  vectorPointCloud.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES detection rootfinding
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

add_library(euclideanClusterer src/euclideanClusterer.cpp)
add_dependencies(euclideanClusterer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(euclideanClusterer_node src/euclideanClusterer_node.cpp)
target_link_libraries(euclideanClusterer_node euclideanClusterer boundingBoxes ${catkin_LIBRARIES})
add_dependencies(euclideanClusterer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(boundingBoxes src/boundingBoxes.cpp)
add_dependencies(boundingBoxes ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(boundingBoxes_node src/boundingBoxes_node.cpp)
target_link_libraries(boundingBoxes_node boundingBoxes ${catkin_LIBRARIES})
add_dependencies(boundingBoxes_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_library(volexFilter src/volexFilter.cpp)
add_dependencies(volexFilter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(volexFilter_node src/volexFilter_node.cpp)
target_link_libraries(volexFilter_node volexFilter ${catkin_LIBRARIES})
add_dependencies(volexFilter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(matching src/matching.cpp)
add_dependencies(matching ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(matching_node src/matching_node.cpp)
target_link_libraries(matching_node matching ${catkin_LIBRARIES})
add_dependencies(matching_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})