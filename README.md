## 运行环境
1. 系统：Ubuntu 22.04.2
1. ROS2版本：ROS2-Humble
1. Gazebo 11.10.2
1. 在 **$GAZEBO_MODEL_PATH** 路径下存在名为 quadrotor 的无人机模型文件。
    > 检验方式：在终端中输入命令  
    `ls $GAZEBO_MODEL_PATH | grep quadrotor`  
    如果没有返回值，可能未安装该模型，也可能是 Gazebo 默认的模型路径未指向安装位置。  
    模型下载参考 [https://bbs.csdn.net/topics/616849315](https://bbs.csdn.net/topics/616849315)
1. Python版本：3.10.12，已安装 numpy 和 quaternion 包。
    > `pip3 install numpy`  
    `pip3 install numpy-quaternion`

## 安装功能包
`cd ~`  
`mkdir -p quadrotor_ws/src/quadrotor_formation`  
`cd quadrotor_ws/src/quadrotor_formation`  
`git clone https://github.com/GreenZhu233/Robotics-Experiments_planar-2DOF-robot.git`

## 编译功能包
`cd ~/quadrotor_ws`  
`colcon build`

## 运行
`source install/setup.bash`  
`ros2 launch quadrotor_formation circle.launch.py`

## 其他
在 launch/circle.launch.py 中修改变量 *num_of_quadrotors* 的值，可以控制编队的无人机数量。