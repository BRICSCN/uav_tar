cmake_minimum_required(VERSION 3.0.2)
project(yolo_tracking_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  mavros_msgs
  cv_bridge
  sensor_msgs
  image_transport
  yolov5_ros_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs mavros_msgs cv_bridge sensor_msgs yolov5_ros_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(tracking_control_node src/tracking_control_node.cpp)
target_link_libraries(tracking_control_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(DIRECTORY launch urdf rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 只安装run_yolo.sh和run_simulation.sh
catkin_install_python(PROGRAMS
  scripts/run_yolo.sh
  scripts/run_simulation.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 