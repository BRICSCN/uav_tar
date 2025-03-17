#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import shutil
import subprocess

def check_model():
    """检查iris_vision模型是否存在，如果不存在则创建符号链接"""
    
    # 获取HOME目录
    home_dir = os.environ.get('HOME')
    if not home_dir:
        print("无法获取HOME目录")
        return False
    
    # 检查PX4固件目录
    px4_firmware_dir = os.path.join(home_dir, 'PX4_Firmware')
    if not os.path.exists(px4_firmware_dir):
        print("PX4_Firmware目录不存在: {}".format(px4_firmware_dir))
        return False
    
    # 检查模型目录
    models_dir = os.path.join(px4_firmware_dir, 'Tools', 'sitl_gazebo', 'models')
    if not os.path.exists(models_dir):
        print("模型目录不存在: {}".format(models_dir))
        return False
    
    # 检查iris_vision模型
    iris_vision_dir = os.path.join(models_dir, 'iris_vision')
    if os.path.exists(iris_vision_dir):
        print("iris_vision模型已存在: {}".format(iris_vision_dir))
        
        # 检查模型配置文件
        model_config = os.path.join(iris_vision_dir, 'model.config')
        if not os.path.exists(model_config):
            print("警告: model.config文件不存在")
        
        # 检查SDF文件
        sdf_file = os.path.join(iris_vision_dir, 'iris_vision.sdf')
        if not os.path.exists(sdf_file):
            print("警告: iris_vision.sdf文件不存在")
        
        # 检查模型文件夹内容
        print("iris_vision模型文件夹内容:")
        for item in os.listdir(iris_vision_dir):
            print("  - {}".format(item))
        
        return True
    
    # 如果iris_vision不存在，检查其他可能的相机模型
    print("iris_vision模型不存在，检查其他相机模型...")
    
    # 列出所有可用的iris模型
    iris_models = [d for d in os.listdir(models_dir) if d.startswith('iris') and os.path.isdir(os.path.join(models_dir, d))]
    print("可用的iris模型: {}".format(iris_models))
    
    # 优先选择带相机的模型
    camera_models = ['iris_vision', 'iris_fpv_cam', 'iris_downward_camera', 'iris_stereo_camera', 'iris_realsense_camera']
    
    selected_model = None
    for model in camera_models:
        if model in iris_models:
            selected_model = model
            print("选择模型: {}".format(selected_model))
            break
    
    if not selected_model:
        print("未找到合适的相机模型")
        return False
    
    # 创建符号链接
    selected_model_dir = os.path.join(models_dir, selected_model)
    print("创建从 {} 到 {} 的符号链接".format(selected_model_dir, iris_vision_dir))
    
    try:
        # 在Linux上创建符号链接
        os.symlink(selected_model_dir, iris_vision_dir)
        print("符号链接创建成功")
        return True
    except Exception as e:
        print("创建符号链接失败: {}".format(e))
        return False

if __name__ == '__main__':
    check_model() 