cmake_minimum_required(VERSION 2.8.3)
project(tbtop_square)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
  FILES
  Projected.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs jsk_recognition_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
add_library(${PROJECT_NAME} src/projector.cpp)
add_executable(projector src/projector.cpp)
target_link_libraries(projector ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_definitions(-std=c++11)
