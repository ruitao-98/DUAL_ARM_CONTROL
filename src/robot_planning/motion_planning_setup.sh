cd scripts
sleep 2
python3 robot_sim_env.py &
sleep 5
roslaunch robot_moveit_config_ver2 demo.launch
