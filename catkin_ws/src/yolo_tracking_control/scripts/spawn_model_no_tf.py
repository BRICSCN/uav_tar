#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
import math

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to quaternion without using tf"""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()
    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q

def spawn_model():
    rospy.init_node('spawn_model_no_tf')
    
    # Get parameters
    model_path = rospy.get_param('~model_path')
    model_name = rospy.get_param('~model_name')
    x = float(rospy.get_param('~x', '0'))
    y = float(rospy.get_param('~y', '0'))
    z = float(rospy.get_param('~z', '0'))
    R = float(rospy.get_param('~R', '0'))
    P = float(rospy.get_param('~P', '0'))
    Y = float(rospy.get_param('~Y', '0'))
    reference_frame = rospy.get_param('~reference_frame', 'world')

    rospy.loginfo('Waiting for gazebo services...')
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    # Read model file
    with open(model_path, 'r') as f:
        model_xml = f.read()

    # Create initial pose
    initial_pose = Pose()
    initial_pose.position = Point(x, y, z)
    initial_pose.orientation = euler_to_quaternion(R, P, Y)

    # Try to spawn model
    try:
        resp = spawn_model_srv(
            model_name=model_name,
            model_xml=model_xml,
            robot_namespace='',
            initial_pose=initial_pose,
            reference_frame=reference_frame
        )
        if resp.success:
            rospy.loginfo("Successfully spawned model %s", model_name)
        else:
            rospy.logerr("Failed to spawn model %s: %s", model_name, resp.status_message)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

if __name__ == '__main__':
    try:
        spawn_model()
    except rospy.ROSInterruptException:
        pass 