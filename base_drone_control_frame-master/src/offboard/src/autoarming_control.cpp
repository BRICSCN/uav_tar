/**
 * @file autoarming_control.cpp
 * @brief Offboard control example node, flying a square or circular trajectory
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include <vector>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

double calculate_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    return std::sqrt(dx*dx + dy*dy);
}

std::vector<std::pair<double, double>> generate_square_waypoints(double side_length) {
    return {
        {0, 0}, {side_length, 0}, {side_length, side_length}, {0, side_length}
    };
}

std::vector<std::pair<double, double>> generate_circle_waypoints(double radius, double center_x, double center_y, int num_points) {
    std::vector<std::pair<double, double>> waypoints;
    double angle_increment = 2 * M_PI / num_points;
    for (int i = 0; i < num_points; ++i) {
        double angle = i * angle_increment;
        double x = center_x + radius * cos(angle);
        double y = center_y + radius * sin(angle);
        waypoints.push_back({x, y});
    }
    return waypoints;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    // 从参数服务器读取飞行模式（默认正方形）
    std::string flight_mode;
    nh.param<std::string>("flight_mode", flight_mode, "square"); // 参数可通过launch文件传入

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);

    // 等待FCU连接
    while(ros::ok() && !current_state.connected){
        ROS_INFO_THROTTLE(3, "\033[37m  Waiting for FCU connection...   \033[0m");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO_STREAM("\033[32m FCU connected \033[0m");

    // 初始位置设置
    geometry_msgs::PoseStamped pose;
    pose.pose.position.z = 2;

    // 发送一些预设点
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // 设置OFFBOARD模式和解锁命令
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    // 定义路径点：根据飞行模式选择正方形或圆形
    int waypoint_index = 0;
    std::vector<std::pair<double, double>> waypoints;

    if (flight_mode == "square") {
        // 生成正方形路径
        waypoints = generate_square_waypoints(8.0); // 正方形边长为4米
    } else if (flight_mode == "circle") {
        // 生成圆形路径
        waypoints = generate_circle_waypoints(2.0, 0, 0, 36); // 半径2米，圆心在(0,0)，分36个点
    } else {
        ROS_ERROR("Invalid flight mode. Using square by default.");
        waypoints = generate_square_waypoints(4.0);
    }

    while(ros::ok()){
        // 请求OFFBOARD模式
        if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        // 解锁无人机
        else if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
            if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }

        // 发布目标点
        pose.pose.position.x = waypoints[waypoint_index].first;
        pose.pose.position.y = waypoints[waypoint_index].second;
        local_pos_pub.publish(pose);
        
        // 计算距离并切换到下一个目标点
        double distance = calculate_distance(current_pose, pose);
        if(distance < 0.3){
            waypoint_index = (waypoint_index + 1) % waypoints.size();
            ROS_INFO("Reached waypoint %d, moving to next", waypoint_index);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
