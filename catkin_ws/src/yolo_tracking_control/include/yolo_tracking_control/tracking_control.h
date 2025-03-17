#ifndef TRACKING_CONTROL_H
#define TRACKING_CONTROL_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <yolov5_ros_msgs/BoundingBoxes.h>
#include <opencv2/opencv.hpp>

class TrackingControl {
public:
    TrackingControl(ros::NodeHandle& nh);
    ~TrackingControl();

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Subscriber yolo_detection_sub_;
    ros::Subscriber image_sub_;
    ros::Publisher local_pos_pub_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient set_mode_client_;
    
    // 状态变量
    mavros_msgs::State current_state_;
    bool tracking_enabled_;
    int tracked_object_id_;
    cv::Rect2d tracked_bbox_;
    
    // 回调函数
    void stateCallback(const mavros_msgs::State::ConstPtr& msg);
    void detectionCallback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr& msg);
    void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    void mouseCallback(int event, int x, int y, int flags, void* userdata);
    
    // 控制函数
    void updateDronePosition();
    void initializeTracking();
    
    // 辅助函数
    double calculateDistance(const cv::Rect2d& bbox);
    geometry_msgs::PoseStamped calculateTargetPose(const cv::Rect2d& bbox);
};

#endif // TRACKING_CONTROL_H 