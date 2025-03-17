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
#include <tf2/LinearMath/Quaternion.h>  // 四元数计算
#include <locale>  // 本地化库

mavros_msgs::State current_state;  // 当前无人机状态
geometry_msgs::PoseStamped current_pose;  // 当前无人机位置
geometry_msgs::PoseStamped home_pose;  // 起飞点位置
int lap_count = 0;  // 圈数计数器

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;  // 更新无人机状态
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;  // 更新无人机当前位置
    static bool first_pose = true;
    if (first_pose) {
        home_pose = current_pose;  // 记录起飞点位置
        first_pose = false;
    }
    ROS_INFO_THROTTLE(1, "当前高度: %.2f m", current_pose.pose.position.z); // 每秒输出一次高度
}

double calculate_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
    double dx = p1.pose.position.x - p2.pose.position.x;  // 计算x方向距离
    double dy = p1.pose.position.y - p2.pose.position.y;  // 计算y方向距离
    return std::sqrt(dx*dx + dy*dy);  // 返回两点间欧几里得距离
}

void set_yaw_orientation(geometry_msgs::PoseStamped& pose, double yaw) {
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);  // 设置偏航角
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
}

double calculate_yaw(const geometry_msgs::PoseStamped& current, const geometry_msgs::PoseStamped& target) {
    double dx = target.pose.position.x - current.pose.position.x;
    double dy = target.pose.position.y - current.pose.position.y;
    return atan2(dy, dx);  // 计算目标方向偏航角
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
    // 设置本地化
    setlocale(LC_ALL, "zh_CN.UTF-8");
    
    ros::init(argc, argv, "manual_offb_node");
    ros::NodeHandle nh;

    // 从参数服务器读取飞行模式（默认正方形）
    std::string flight_mode;
    nh.param<std::string>("flight_mode", flight_mode, "square");

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(50.0);

    // 等待FCU连接
    while(ros::ok() && !current_state.connected){
        ROS_INFO_THROTTLE(3, "\033[37m  等待飞控连接...   \033[0m");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO_STREAM("\033[32m 飞控已连接 \033[0m");

    // 检查传感器数据有效性
    if(fabs(current_pose.pose.position.z) > 10.0) {
        ROS_ERROR("无效的高度数据: %.2f m. 请检查传感器校准.", current_pose.pose.position.z);
        return -1;
    }

    // 初始化变量
    bool armed = false;
    bool offboard_enabled = false;
    geometry_msgs::PoseStamped pose;

    // 修改菜单显示方式
    std::cout << "\n============ 无人机控制菜单 ============" << std::endl;
    std::cout << "1 - 解锁无人机" << std::endl;
    std::cout << "2 - 进入OFFBOARD模式" << std::endl;
    std::cout << "3 - 开始执行任务" << std::endl;
    std::cout << "s - 显示当前状态" << std::endl;
    std::cout << "q - 退出程序" << std::endl;
    std::cout << "=====================================" << std::endl;

    while(ros::ok()){
        std::cout << "\n请输入命令: ";
        char cmd;
        std::cin >> cmd;

        // 将变量声明移到switch外部
        geometry_msgs::PoseStamped setpoint;
        mavros_msgs::SetMode offb_set_mode;

        switch(cmd) {
            case '1':  // 解锁
                {   // 使用代码块限制作用域
                    if(!current_state.armed) {
                        mavros_msgs::CommandBool arm_cmd;
                        arm_cmd.request.value = true;
                        
                        std::cout << "正在尝试解锁..." << std::endl;
                        if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                            // 等待确认解锁状态
                            ros::Time last_request = ros::Time::now();
                            while(ros::ok() && (ros::Time::now() - last_request < ros::Duration(2.0))) {
                                if(current_state.armed) {
                                    std::cout << "无人机解锁成功确认" << std::endl;
                                    armed = true;
                                    break;
                                }
                                ros::spinOnce();
                                rate.sleep();
                            }
                            
                            if(!current_state.armed) {
                                std::cout << "解锁命令已发送但未确认，请检查：" << std::endl;
                                std::cout << "1. 遥控器是否在手动模式" << std::endl;
                                std::cout << "2. 飞控是否允许解锁" << std::endl;
                                std::cout << "3. 安全开关状态" << std::endl;
                            }
                        } else {
                            std::cout << "无人机解锁命令发送失败" << std::endl;
                        }
                    } else {
                        std::cout << "无人机已经处于解锁状态" << std::endl;
                    }
                }
                break;

            case '2':  // 进入OFFBOARD模式
                {   
                    if(!current_state.armed) {
                        std::cout << "请先解锁无人机" << std::endl;
                        break;
                    }

                    std::cout << "当前模式: " << current_state.mode << std::endl;
                    
                    // 初始化setpoint为当前高度
                    geometry_msgs::PoseStamped setpoint;
                    setpoint.pose.position.x = 0;
                    setpoint.pose.position.y = 0;
                    setpoint.pose.position.z = 1.5;  // 设置初始高度
                    set_yaw_orientation(setpoint, 0);

                    // 先发送100个setpoint
                    std::cout << "发送初始setpoint..." << std::endl;
                    for(int i = 100; ros::ok() && i > 0; --i){
                        if(i % 25 == 0) {
                            std::cout << "还需发送 " << i << " 次..." << std::endl;
                        }
                        local_pos_pub.publish(setpoint);
                        ros::spinOnce();
                        rate.sleep();
                    }

                    // 尝试切换到OFFBOARD模式
                    mavros_msgs::SetMode offb_set_mode;
                    offb_set_mode.request.custom_mode = "OFFBOARD";
                    
                    if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                        ros::Time last_request = ros::Time::now();
                        while(ros::ok() && (ros::Time::now() - last_request < ros::Duration(5.0))) {
                            // 持续发送setpoint
                            local_pos_pub.publish(setpoint);
                            
                            if(current_state.mode == "OFFBOARD") {
                                std::cout << "成功切换至OFFBOARD模式" << std::endl;
                                offboard_enabled = true;
                                break;
                            }
                            
                            ros::spinOnce();
                            rate.sleep();
                        }
                        
                        if(!offboard_enabled) {
                            std::cout << "模式切换超时，请检查：" << std::endl;
                            std::cout << "1. 遥控器是否允许OFFBOARD模式" << std::endl;
                            std::cout << "2. 检查飞控参数配置" << std::endl;
                            std::cout << "当前模式: " << current_state.mode << std::endl;
                        }
                    } else {
                        std::cout << "OFFBOARD模式切换命令发送失败" << std::endl;
                    }
                }
                break;

            case '3':  // 开始执行任务
                {   // 使用代码块限制作用域
                    if(!current_state.armed || !offboard_enabled) {
                        std::cout << "请确保无人机已解锁并处于OFFBOARD模式" << std::endl;
                        break;
                    }
                    
                    pose.header.frame_id = "map";
                    pose.pose.position.x = 0;
                    pose.pose.position.y = 0;
                    pose.pose.position.z = 1.5;
                    set_yaw_orientation(pose, 0);

                    std::cout << "开始执行飞行任务..." << std::endl;
                    // TODO: 添加任务执行代码
                }
                break;

            case 's':  // 显示状态
                {   // 使用代码块限制作用域
                    std::cout << "\n====== 当前状态 ======" << std::endl;
                    std::cout << "解锁状态: " << (current_state.armed ? "已解锁" : "未解锁") << std::endl;
                    std::cout << "飞行模式: " << current_state.mode << std::endl;
                    std::cout << "当前高度: " << current_pose.pose.position.z << " m" << std::endl;
                    std::cout << "======================" << std::endl;
                }
                break;

            case 'q':  // 退出程序
                std::cout << "程序正常退出" << std::endl;
                return 0;

            default:
                std::cout << "无效命令，请重新输入" << std::endl;
                break;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
