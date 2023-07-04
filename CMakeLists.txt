cmake_minimum_required(VERSION 3.0.2)
project(second_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
  move_base
  move_base_msgs
  tf
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES second_project
 CATKIN_DEPENDS roscpp rospy std_msgs actionlib move_base_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(tf_publisher src/tf_publisher.cpp)
add_executable(navigation src/navigation.cpp)
# add_dependencies(tf_publisher second_project_generate_messages_cpp)
target_link_libraries(tf_publisher ${catkin_LIBRARIES})
target_link_libraries(navigation ${catkin_LIBRARIES})
add_dependencies(tf_publisher ${catkin_EXPORTED_TARGETS})
add_dependencies(navigation ${catkin_EXPORTED_TARGETS})