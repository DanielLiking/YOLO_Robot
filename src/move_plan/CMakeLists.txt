cmake_minimum_required(VERSION 3.0.2)
project(move_plan)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  actionlib_msgs
  geometry_msgs
  move_base_msgs
)
find_package(OpenCV REQUIRED)
find_package(actionlib_msgs REQUIRED)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(plan src/test.cpp)
target_link_libraries(plan
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   )
