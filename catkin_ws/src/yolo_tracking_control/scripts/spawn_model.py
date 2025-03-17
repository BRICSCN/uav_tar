#!/usr/bin/env python2

import rospy
import rospkg
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

def spawn_model():
    rospy.init_node('spawn_model')
    
    # 等待Gazebo服务
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    
    # 获取URDF内容
    with open(rospy.get_param('robot_description'), 'r') as f:
        model_xml = f.read()
    
    # 设置初始位姿
    initial_pose = Pose()
    initial_pose.position = Point(0, 0, 0.5)
    initial_pose.orientation = Quaternion(0, 0, 0, 1)
    
    # 生成模型
    spawn_urdf(
        model_name='iris',
        model_xml=model_xml,
        robot_namespace='/iris',
        initial_pose=initial_pose,
        reference_frame='world'
    )

if __name__ == '__main__':
    try:
        spawn_model()
    except rospy.ROSInterruptException:
        pass 