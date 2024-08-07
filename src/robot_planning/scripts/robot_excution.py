from robot_sim_env import Dual_arm_env
import numpy as np
import time
import transforms3d.quaternions as trans_quat
env = Dual_arm_env()
env.reset()

#首先促使机器人运动到初始位置
right_action = np.array([-np.pi/3, np.pi/3,2*np.pi / 3,np.pi/2 ,-np.pi / 2,np.pi / 2])
left_action = env.left_joint_pos
target_pos = np.concatenate([left_action, right_action])
env.joint_space_step(target_pos)
time.sleep(1)
dual_link6_pose = env.get_pose_()
left_eef_pos, left_eef_rotm, right_eef_pos, right_eef_rotm = env.get_eef_pose_(dual_link6_pose)
left_base_pos = env.left_base_pos
left_base_rotm = env.left_base_rotm
right_base_pos = env.right_base_pos
right_base_rotm = env.right_base_rotm

def get_right_in_left():
    global left_eef_pos, left_eef_rotm, right_eef_pos, right_eef_rotm
    global left_base_pos, left_base_rotm, right_base_pos, right_base_rotm

    right_pos_in_world = right_base_rotm @ right_eef_pos + right_base_pos
    right_rotm_in_world = right_base_rotm @ right_eef_rotm
    print(right_pos_in_world, right_rotm_in_world)

    right_pos_in_left = left_base_rotm.T @ (right_pos_in_world - left_base_pos)
    right_rotm_in_left = left_base_rotm.T @ right_rotm_in_world

    return right_pos_in_left, right_rotm_in_left

right_pos_in_left, right_rotm_in_left = get_right_in_left()
print(right_pos_in_left, right_rotm_in_left)
print(right_eef_rotm)
print(right_eef_pos)
# right_quat_in_left = trans_quat.mat2quat(right_rotm_in_left)
# right_eef_quat = trans_quat.mat2quat(right_eef_rotm)
# action = np.concatenate([right_pos_in_left, right_quat_in_left, right_eef_pos, right_eef_quat])
# for i in range(100):
#     env.step(action)




    # if env.moveit_joint_values:
    #     with env.lock:
    #         actions = np.array(env.moveit_joint_values)
    #     if env.which_arm == 'l_j1':
    #         right_action = env.right_joint_pos
    #         for i in range(actions.shape[0]):
    #             left_action = actions[i]
    #             action = np.concatenate([left_action, right_action])
    #             env.joint_space_step(action)
    #
    #     if env.which_arm == 'r_j1':
    #         left_action = env.left_joint_pos
    #         for i in range(actions.shape[0]):
    #             right_action = actions[i]
    #             action = np.concatenate([left_action, right_action])
    #             env.joint_space_step(action)
    #
    #     env.moveit_joint_values = []
