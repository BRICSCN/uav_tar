#include "yolo_tracking_control/tracking_control.h"

TrackingControl::TrackingControl(ros::NodeHandle& nh) : nh_(nh) {
    // 初始化订阅者
    state_sub_ = nh_.subscribe<mavros_msgs::State>
        ("mavros/state", 10, &TrackingControl::stateCallback, this);
    yolo_detection_sub_ = nh_.subscribe<yolov5_ros_msgs::BoundingBoxes>
        ("/yolov5/BoundingBoxes", 1, &TrackingControl::detectionCallback, this);
    image_sub_ = nh_.subscribe<sensor_msgs::Image>
        ("/camera/color/image_raw", 1, &TrackingControl::imageCallback, this);

    // 初始化发布者
    local_pos_pub_ = nh_.advertise<geometry_msgs::PoseStamped>
        ("mavros/setpoint_position/local", 10);

    // 初始化服务客户端
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");
    set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");

    // 初始化变量
    tracking_enabled_ = false;
    tracked_object_id_ = -1;
}

// 析构函数定义
TrackingControl::~TrackingControl() {
    // 如果有需要释放的资源，可以在这里添加相应的代码
    // 例如，关闭图像显示窗口
    cv::destroyAllWindows();
}

void TrackingControl::stateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_state_ = *msg;
}

void TrackingControl::detectionCallback(const yolov5_ros_msgs::BoundingBoxes::ConstPtr& msg) {
    if (!tracking_enabled_) return;
    
    // 更新追踪目标的边界框
    for (const auto& box : msg->bounding_boxes) {
        if (box.id == tracked_object_id_) {
            tracked_bbox_ = cv::Rect2d(box.xmin, box.ymin, 
                                     box.xmax - box.xmin, 
                                     box.ymax - box.ymin);
            updateDronePosition();
            break;
        }
    }
}

void TrackingControl::imageCallback(const sensor_msgs::Image::ConstPtr& msg) {
    try {
        cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        
        // 显示追踪框
        if (tracking_enabled_ && tracked_bbox_.width > 0) {
            cv::rectangle(frame, tracked_bbox_, cv::Scalar(0, 255, 0), 2);
        }
        
        cv::imshow("Tracking View", frame);
        cv::waitKey(1);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void TrackingControl::updateDronePosition() {
    if (!tracking_enabled_) return;
    
    geometry_msgs::PoseStamped target_pose = calculateTargetPose(tracked_bbox_);
    local_pos_pub_.publish(target_pose);
}

geometry_msgs::PoseStamped TrackingControl::calculateTargetPose(const cv::Rect2d& bbox) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    
    // 计算目标位置，保持1米距离
    double distance = calculateDistance(bbox);
    double scale_factor = 1.0 / distance;  // 用于调整到1米距离
    
    pose.pose.position.x = bbox.x + bbox.width/2;  // 中心x坐标
    pose.pose.position.y = bbox.y + bbox.height/2;  // 中心y坐标
    pose.pose.position.z = 1.0;  // 保持固定高度
    
    return pose;
}

double TrackingControl::calculateDistance(const cv::Rect2d& bbox) {
    // 基于边界框大小估算距离
    // 这里需要根据实际相机参数和目标物体大小进行调整
    const double KNOWN_WIDTH = 0.5;  // 假设目标物体宽度为0.5米
    const double FOCAL_LENGTH = 1000.0;  // 相机焦距，需要标定
    
    return (FOCAL_LENGTH * KNOWN_WIDTH) / bbox.width;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "tracking_control_node");
    ros::NodeHandle nh;
    
    TrackingControl tracking_control(nh);
    
    ros::spin();
    
    return 0;
}
