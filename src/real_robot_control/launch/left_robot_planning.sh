rosrun real_robot_control motion_planning &

sleep 5
roslaunch robot_moveit_config_ver2 demo.launch &

sleep 3
rosrun real_robot_control task_control_screw &

sleep 1
rosrun real_robot_control left_or_right_assembly &

sleep 1
rosrun real_robot_control gripper_open.py