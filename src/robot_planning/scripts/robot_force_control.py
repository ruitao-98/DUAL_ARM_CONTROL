#! /home/yanji/anaconda3/envs/mujo/bin/python3
# -*- coding: utf-8 -*-

from robot_sim_env import Dual_arm_env
import numpy as np
import time
import transforms3d.quaternions as trans_quat



env = Dual_arm_env()

state, left_eef_pos, left_eef_rotm, right_eef_pos, right_eef_rotm = env.reset()

left_eef_quat = trans_quat.mat2quat(left_eef_rotm)
right_eef_quat = trans_quat.mat2quat(right_eef_rotm)

# print(action)
# ros 的指令从这里传入，我们首先只考虑控制一个机械臂
# action[:6] = np.array([-0.2, 0, 0, 0, 0, 0])

# action[9] += -0.2
for _ in range(10000):
    start = time.time()
    action = np.concatenate([left_eef_pos, left_eef_quat, right_eef_pos, right_eef_quat])
    # action = np.ones(14)  #我们对这里进行分解，action=np.ones(12)意思是控制两台机械臂的末端运动，即两个机械臂的末端绝对位置
    action[7:9] += np.random.uniform(low=-0.01, high=0.01, size=2)
    # action[0:2] += np.random.uniform(low=-0.01, high=0.01, size=2)
    print(action)
    ##############

    ob, reward, done, _ = env.admittance_step(action)
    print('######################one epo########################')
    # print(ob[:3])
    # print(reward)
    # print(time.time() - start)
    time.sleep(0.1)