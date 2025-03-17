#!/bin/bash

# 激活conda环境
source ~/anaconda3/etc/profile.d/conda.sh
conda activate yolov5

# 设置ROS环境
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash

# 设置PYTHONPATH以找到YOLOv5
export PYTHONPATH=$PYTHONPATH:$(env HOME)/catkin_ws/src/Yolov5_ros-master/yolov5_ros/yolov5

# 运行launch文件
exec roslaunch yolo_tracking_control tracking_control_simulation.launch 