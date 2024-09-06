#!/bin/bash

rosrun real_robot_control task_control_screw &

sleep 3
rosrun real_robot_control calcu_orientation.py &

sleep 1
rosrun real_robot_control right_robot_control