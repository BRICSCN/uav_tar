# step
## 创建基础包
~~~
mkdir -p drone_control_ws/src
cd drone_control_ws
catkin_make

cd src
catkin_create_pkg offboard roscpp rospy std_msgs
~~~

## write function
we have two method to write a ros node, a easy which only haves one .cpp and .launch file, and a defficult which is used class of c++ and hpp file.
### easy 
autoarming_control.cpp
position_control.launch

### defficult
position_control.cpp
position_control_lib.cpp
position_control.launch
position_control.h
1. use a class, and put main function to its lib's constructor function

2. you must writ CmakeList.txt :

~~~
# for position_control_lib.cpp build .lib and position_control.cpp build .exe

#添加库
add_library(position_control_lib src/position_control_lib.cpp)
#添加依赖
add_dependencies(position_control_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(position_control src/position_control.cpp)
add_dependencies(position_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 添加自己的头文件库和其他默认库
target_link_libraries(position_control
position_control_lib
  ${catkin_LIBRARIES}
)
~~~
3. you must note the CallBack function's message type.
a pub and a sub, them have a difference point: const and not const
~~~
    position_sub = nh_.subscribe<const geometry_msgs::PoseStamped&>("/drone_control/goal_position", 10, &DroneControl::position_CallBack, this);
    local_pos_pub = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
~~~
CallBack function type:
~~~
void DroneControl::position_CallBack(const geometry_msgs::PoseStamped& msg)
{
  /***/
}
~~~


## 编译及运行
~~~
//编译
catkin_make
//更新环境变量，可以把刚才编译的功能包路径添加到环境中，那么本终端就可以运行本工作空间的功能包，也可以在~/.bashrc里面添加source语句，那么所有终端都可以使用本工作空间功能包
source ./devel/setup.bash
//运行
roslaunch offboard position_control.launch
~~~





# note
## 1.文件注释的格式
这种文件的注释格式是 `Doxygen` 风格的注释。`Doxygen` 是一种用于从源代码中生成文档的工具，支持多种编程语言，包括 `C/C++`、`Java`、`Python` 等。`Doxygen` 注释允许你为代码添加额外的描述，这些描述可以被 `Doxygen` 工具解析并用于生成 `HTML`、`LaTeX`、`PDF` 等格式的文档。

`Doxygen` 注释通常包含以下部分：

    @file：标识当前文件的描述。
    @brief：为接下来的元素（例如类、函数、变量等）提供简短的描述。
    @detail：用于提供更详细的描述。
    @param：描述函数或方法的参数。
    @return：描述函数或方法的返回值。
    @see：引用其他相关的元素或页面。

## 2.CmakeList语法
这是最基础的，添加一个可执行文件，并将它链接到一个库
~~~
add_executable(可执行文件名字（launch文件启动的名字）
  src/源文件名.cpp
)
target_link_libraries(可执行文件名字
  ${catkin_LIBRARIES}
)
~~~

如果有自定义消息的话：
~~~
add_executable(可执行文件名字（launch文件启动的名字） src/源文件名.cpp)

add_dependencies(可执行文件名字 ${PROJECT_NAME}_generate_messages_cpp)

target_link_libraries(可执行文件名字
  ${catkin_LIBRARIES}
)

~~~

如果有库要生成：
~~~
//假设JetsonGPIO是一个CMake包，您可以使用find_package来查找它 
find_package(JetsonGPIO REQUIRED) 
  
//添加库（假设您有一个库需要构建） 
add_library(my_library_name src/nodes/library_source.cpp) 
target_link_libraries(my_library_name JetsonGPIO) # 如果库需要链接到JetsonGPIO 
  
//添加可执行文件 
add_executable(main_control_li_main_ego src/nodes/main_control_li_main_ego.cpp) 
//链接可执行文件到库和其他依赖项 
target_link_libraries( 
    main_control_li_main_ego 
    PUBLIC 
    my_library_name # 链接到您的库 
    JetsonGPIO # 链接到JetsonGPIO库 
    ${catkin_LIBRARIES} # 链接到Catkin库 
) 
  
//添加依赖项（如果您有其他生成的消息或目标） 
add_dependencies(main_control_li_main_ego ${PROJECT_NAME}_generate_messages_cpp) 
//如果my_library_name也依赖于其他catkin或系统生成的目标，您也需要添加相应的依赖项 
//例如：add_dependencies(my_library_name some_other_target)
~~~

## 3.ROS提示输出颜色控制
[原文](https://blog.csdn.net/lemonxiaoxiao/article/details/123529344)
~~~
#include <ros/ros.h>
#include <ros/console.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rosdebug");

  ROS_INFO_STREAM("\033[30m 黑色字 \033[0m");
  ROS_INFO_STREAM("\033[31m 红色字 \033[0m");
  ROS_INFO_STREAM("\033[32m 绿色字 \033[0m");
  ROS_INFO_STREAM("\033[33m 黄色字 \033[0m");
  ROS_INFO_STREAM("\033[34m 蓝色字 \033[0m");
  ROS_INFO_STREAM("\033[36m 天蓝字 \033[0m");
  ROS_INFO_STREAM("\033[37m 白色字 \033[0m");
  ROS_INFO_STREAM("\033[40;37m 黑底白字 \033[0m");
  ROS_INFO_STREAM("\033[41;37m 红底白字 \033[0m");
  ROS_INFO_STREAM("\033[42;37m 绿底白字 \033[0m");
  ROS_INFO_STREAM("\033[47;30m 白底黑字 \033[0m");
  ROS_INFO_STREAM("\033[43;37m 黄底白字 \033[0m");
  ROS_INFO_STREAM("\033[44;37m 蓝底白字 \033[0m");
  ROS_INFO_STREAM("\033[45;37m 紫底白字 \033[0m");
  ROS_INFO_STREAM("\033[46;37m 天蓝底白字 \033[0m");

  return 1;
}
~~~


