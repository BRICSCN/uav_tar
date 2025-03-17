# 快速开始

## 下载编译
~~~
下载
cd ~/ 
git clone https://gitee.com/lulese/base_drone_control_frame.git
cd base_drone_control_frame/

//编译
catkin_make
//更新环境变量，可以把刚才编译的功能包路径添加到环境中，那么本终端就可以运行本工作空间的功能包，也可以在~/.bashrc里面添加source语句，那么所有终端都可以使用本工作空间功能包
source ./devel/setup.bash
~~~
## 运行

### 启动环境（开一个终端）

~~~
roslaunch px4 mavros_posix_sitl.launch 
~~~

### 程控（两个launch二选一）（另开一个终端）

~~~
//运行点控程序(需要自己解锁、切到offboard)
cd ~/base_drone_control_frame/ && source ./devel/setup.bash
roslaunch offboard position_control.launch

//跑正方形或者圆形(不需要自己解锁、切到offboard，注意实物飞行慎用这个程序，防止意外发生，还是写成手动解锁好一点)
cd ~/base_drone_control_frame/ && source ./devel/setup.bash
roslaunch offboard autoarming_control.launch 
~~~

other note, please goto note.md