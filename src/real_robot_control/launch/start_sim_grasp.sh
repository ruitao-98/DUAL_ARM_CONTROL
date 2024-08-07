#!/bin/bash

# 启动第一个launch文件
rosrun robot_planning robot_sim_env.py &

# 休眠10秒钟
sleep 5

# 启动第二个launch文件
roslaunch robot_moveit_config_ver2 demo.launch 