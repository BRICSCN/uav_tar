#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.msg import ModelStates
import time

def wait_for_model(model_name, timeout=20):
    """Wait for model to spawn"""
    rospy.loginfo("Waiting for model %s...", model_name)
    
    # 等待/gazebo/model_states话题可用
    try:
        rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5)
    except rospy.ROSException:
        rospy.logerr("Timeout waiting for /gazebo/model_states topic")
        return False

    start_time = time.time()
    rate = rospy.Rate(10)  # 10Hz
    
    while not rospy.is_shutdown():
        try:
            model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=1)
            rospy.loginfo("Available models: %s", ', '.join(model_states.name))  # 打印所有可用模型
            if model_name in model_states.name:
                rospy.loginfo("Model %s found", model_name)
                return True
            for name in model_states.name:
                if model_name in name:  # 部分匹配
                    rospy.loginfo("Found similar model: %s", name)
                    return name
        except rospy.ROSException:
            pass
            
        if time.time() - start_time > timeout:
            rospy.logerr("Timeout waiting for model %s", model_name)
            return False
            
        rate.sleep()
    return False

def spawn_camera():
    rospy.init_node('spawn_camera')
    
    # Wait for Gazebo services
    rospy.loginfo("Waiting for gazebo services...")
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    
    # Wait for iris model to spawn
    rospy.sleep(5)  # 给Gazebo一些时间来完全加载
    iris_name = wait_for_model('iris')
    if not iris_name:
        rospy.logerr("Failed to find iris model")
        return
    
    # Load camera model file
    model_path = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'models/camera/camera.sdf'
    )
    
    try:
        with open(model_path, 'r') as f:
            model_xml = f.read()
    except IOError as e:
        rospy.logerr("Failed to read model file: %s", str(e))
        return
    
    # Set camera pose
    initial_pose = Pose()
    initial_pose.position = Point(0.2, 0, -0.05)
    initial_pose.orientation = Quaternion(0, 0.2588, 0, 0.9659)
    
    try:
        # 使用找到的实际模型名称构建参考框架
        reference_frame = "{}::base_link".format(iris_name)
        rospy.loginfo("Using reference frame: %s", reference_frame)
        
        resp = spawn_model(
            model_name='camera_sensor',
            model_xml=model_xml,
            robot_namespace='/camera_sensor/iris',
            initial_pose=initial_pose,
            reference_frame=reference_frame
        )
        
        if resp.success:
            rospy.loginfo("Camera spawned successfully")
        else:
            rospy.logerr("Failed to spawn camera: %s", resp.status_message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    try:
        spawn_camera()
    except rospy.ROSInterruptException:
        pass 