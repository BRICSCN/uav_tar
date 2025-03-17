#!/bin/bash
source ~/anaconda3/etc/profile.d/conda.sh
conda activate yolov5
exec python3 ~/catkin_ws/src/Yolov5_ros-master/yolov5_ros/yolov5_ros/scripts/yolo_v5.py "$@" 