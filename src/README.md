# How to run 2019 code for simulation
## 创建工作空间文件夹
cd ~ && mkdir -p roborts_ws/src
## 切换至src目录
cd roborts_ws/src
## 下载RoboRTS源码
git clone https://github.com/RoboMaster/RoboRTS
## 编译源码
cd roborts_ws && catkin_make
## 加载环境变量
source devel/setup.bash

## 启动stage仿真
roslaunch roborts_bringup roborts_stage.launch

## 选择启动官方行为树demo或下载decision package启动还在开发中的decision node
rosrun roborts_decision behavior_test_node

rosrun decision decision_node.py
	
# How to run 2019 code in real environment
rosrun roborts_bringup roborts.launch
	
rosrun decision decision_node.py

