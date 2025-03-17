/**
 * @file manual_offboard_control.cpp
 * @brief Offboard control with manual mode switching
 * 手动模式切换的Offboard控制节点
 */

#include <ros/ros.h>  // ROS核心库
#include <geometry_msgs/PoseStamped.h>  // 位置消息类型
#include <mavros_msgs/CommandBool.h>  // 解锁命令服务
#include <mavros_msgs/SetMode.h>  // 模式设置服务
#include <mavros_msgs/State.h>  // 无人机状态消息
#include <cmath>  // 数学计算库
#include <vector>  // 向量容器
#include <iostream>  // 标准输入输出

mavros_msgs::State current_state;  // 当前无人机状态
geometry_msgs::PoseStamped current_pose;  // 当前无人机位置

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;  // 更新无人机状态
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;  // 更新无人机当前位置
}

double calculate_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
    double dx = p1.pose.position.x - p2.pose.position.x;  // 计算x方向距离
    double dy = p1.pose.position.y - p2.pose.position.y;  // 计算y方向距离
    return std::sqrt(dx*dx + dy*dy);  // 返回两点间欧几里得距离
}

std::vector<std::pair<double, double>> generate_square_waypoints(double side_length) {
    return {
        {0, 0},  // 起点
        {side_length, 0},  // 右移
        {side_length, side_length},  // 上移
        {0, side_length}  // 左移
    };  // 返回正方形路径点
}

std::vector<std::pair<double, double>> generate_circle_waypoints(double radius, double center_x, double center_y, int num_points) {
    std::vector<std::pair<double, double>> waypoints;  // 存储路径点
    double angle_increment = 2 * M_PI / num_points;  // 计算角度增量
    for (int i = 0; i < num_points; ++i) {
        double angle = i * angle_increment;  // 当前角度
        double x = center_x + radius * cos(angle);  // 计算x坐标
        double y = center_y + radius * sin(angle);  // 计算y坐标
        waypoints.push_back({x, y});  // 添加路径点
    }
    return waypoints;  // 返回圆形路径点
}

bool confirm_action(const std::string& action) {
    std::cout << "确认要执行 " << action << " 吗? (y/n): ";
    char input;
    std::cin >> input;
    return (input == 'y' || input == 'Y');
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manual_offb_node");  // 初始化ROS节点
    ros::NodeHandle nh;  // 创建节点句柄

    // 从参数服务器读取飞行模式（默认正方形）
    std::string flight_mode;
    nh.param<std::string>("flight_mode", flight_mode, "square"); // 参数可通过launch文件传入
    // 飞行模式：square-正方形，circle-圆形

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);  // 订阅无人机状态
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);  // 订阅无人机位置
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);  // 发布目标位置
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");  // 解锁服务客户端
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");  // 模式设置服务客户端

    ros::Rate rate(20.0);  // 控制循环频率20Hz

    // 等待FCU连接
    while(ros::ok() && !current_state.connected){
        ROS_INFO_THROTTLE(3, "\033[37m  Waiting for FCU connection...   \033[0m");  // 每3秒打印一次连接状态
        ros::spinOnce();  // 处理回调
        rate.sleep();  // 保持循环频率
    }
    ROS_INFO_STREAM("\033[32m FCU connected \033[0m");  // 连接成功提示

    // 初始位置设置
    geometry_msgs::PoseStamped pose;
    pose.pose.position.z = 1.5;  // 设置初始高度为2米

    // 发送一些预设点
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);  // 发布目标位置
        ros::spinOnce();  // 处理回调
        rate.sleep();  // 保持循环频率
    }  // 发送100个预设点以初始化

    // 定义路径点：根据飞行模式选择正方形或圆形
    int waypoint_index = 0;  // 当前路径点索引
    std::vector<std::pair<double, double>> waypoints;  // 存储路径点

    if (flight_mode == "square") {
        // 生成正方形路径
        waypoints = generate_square_waypoints(2.0); // 正方形边长为2米
    } else if (flight_mode == "circle") {
        // 生成圆形路径
        waypoints = generate_circle_waypoints(1.0, 0, 0, 36); // 半径2米，圆心在(0,0)，分36个点
    } else {
        ROS_ERROR("Invalid flight mode. Using square by default.");  // 无效模式提示
        waypoints = generate_square_waypoints(2.0);  // 默认使用2米边长的正方形
    }

    // 手动解锁和模式切换
    bool armed = false;
    bool offboard_enabled = false;

    while(ros::ok()){
        // 手动解锁
        if(!armed && confirm_action("解锁无人机")) {
            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;
            
            if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                ROS_INFO("Vehicle armed");  // 成功解锁
                armed = true;
            } else {
                ROS_WARN("Failed to arm vehicle");
            }
        }

        // 手动进入OFFBOARD模式
        if(armed && !offboard_enabled && confirm_action("进入OFFBOARD模式")) {
            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "OFFBOARD";
            
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");  // 成功进入offboard模式
                offboard_enabled = true;
            } else {
                ROS_WARN("Failed to enable Offboard mode");
            }
        }

        // 如果已解锁，开始导航
        if(armed) {
            // 发布目标点
            pose.pose.position.x = waypoints[waypoint_index].first;  // 设置目标x坐标
            pose.pose.position.y = waypoints[waypoint_index].second;  // 设置目标y坐标
            local_pos_pub.publish(pose);  // 发布目标位置
            
            // 计算距离并切换到下一个目标点
            double distance = calculate_distance(current_pose, pose);  // 计算当前位置与目标点距离
            if(distance < 0.3){  // 如果距离小于0.3米
                waypoint_index = (waypoint_index + 1) % waypoints.size();  // 切换到下一个路径点
                ROS_INFO("Reached waypoint %d, moving to next", waypoint_index);  // 打印到达信息
            }
        }

        ros::spinOnce();  // 处理回调
        rate.sleep();  // 保持循环频率
    }

    return 0;
}
