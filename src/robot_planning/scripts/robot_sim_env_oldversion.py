import mujoco
import mujoco.viewer
import numpy as np
import time
import os
import gym
from gym import spaces, register
import transforms3d.quaternions as trans_quat
import transforms3d.euler as trans_eul
import copy
from scipy.spatial.transform import Rotation as R
# from source.lrmate_kine_base import IK
import kinetic as ki
import func_ki_dy

# from ray.tune.registry import register_env

class Dual_arm_env(gym.Env):
    def __init__(self, render=True):
        super(Dual_arm_env, self).__init__()
        # self.m = mujoco.MjModel.from_xml_path('./source/LRMate_200iD.xml')
        self.m = mujoco.MjModel.from_xml_path('/home/yanji/mujoco/mujoco-3.1.1/my_project/ros_dual_robot_planing/src/robot_description/xml/dual_arm.xml')  #7号从这后面修改
        self.d = mujoco.MjData(self.m)
        self.kdl = ki.RobotKdl(self.m, self.d)
        #ids that are need in the calculation
        self.left_link_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, 'left_link6')
        self.right_link_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, 'right_link6')
        self.left_joint_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, "l_j6")
        self.right_joint_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, "r_j6")

        self.left_force_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SENSOR, "l_force_sensor")
        self.left_torque_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SENSOR, "l_torque_sensor")
        self.right_force_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SENSOR, "r_force_sensor")
        self.right_torque_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_SENSOR, "r_torque_sensor")

        self.left_actuator_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, "l_motor6")
        self.right_actuator_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, "r_motor6")

        self.screw_shaft_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, "r_p_screw")
        self.left_mouth_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, "r_p_mouth1")
        self.right_mouth_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, "r_p_mouth2")
        self.left_gripper_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, "l_p_gripper1")
        self.right_gripper_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_ACTUATOR, "l_p_gripper2")

        # joint control gains & variables
        self.joint_kp = np.array([100, 100, 100, 100, 100, 100])  #关节pd控制器
        self.joint_kd = 2 * np.sqrt(self.joint_kp)  #关节pd控制器

        self.left_joint_pos = np.zeros(6)
        self.left_joint_vel = np.zeros(6)
        self.right_joint_pos = np.zeros(6)
        self.right_joint_vel = np.zeros(6)
        self.left_joint_acc = np.zeros(6)
        self.right_joint_acc = np.zeros(6)

        self.ref_joint = np.zeros(12)
        self.ref_vel = np.zeros(12)
        self.ref_acc = np.zeros(12)

        # self.gripper_pose = 0.031

        # self.set_gripper(self.gripper_pose)  # 夹爪宽度

        # admittance control gains
        self.adm_kp = 10 * np.array([1, 1, 1, 1, 1, 1])  # [10 10 10 10 10 10]
        self.adm_m = 1 * np.array([1, 1, 1, 1, 1, 1])
        self.adm_kd = 4 * np.sqrt(np.multiply(self.adm_kp,
                                              self.adm_m))  # [12.64911064 12.64911064 12.64911064 12.64911064 12.64911064 12.64911064] 4 * 10 ^ (-0.5)

        self.left_adm_pose_ref = np.zeros(7)  # 机器人导纳控制目标位姿
        self.left_adm_vel_ref = np.zeros(6)
        self.right_adm_pose_ref = np.zeros(7)  # 机器人导纳控制目标位姿
        self.right_adm_vel_ref = np.zeros(6)
        self.left_eef_vel = np.zeros(6)
        self.right_eef_vel = np.zeros(6)
        self.HZ = 125
        self.HZ_action = 60   # 外层动作的假设频率，使用这个频率来计算机器人的导纳控制的期望速度

        # robot base pose
        left_base_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, 'left_base')
        right_base_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, 'right_link0')
        self.left_base_pos = self.d.xpos[left_base_id]
        self.right_base_pos = self.d.xpos[right_base_id]
        self.left_base_rotm = self.d.xmat[left_base_id].reshape([3,3])
        self.right_base_rotm = self.d.xmat[right_base_id].reshape([3,3])

        # place holder for robot state
        self.pose_ = np.zeros(14)  # pose of 2 link6, not the end-effector
        self.left_force_sensor_data = np.zeros(6)
        self.right_force_sensor_data = np.zeros(6)
        self.force_offset = np.zeros(6)
        # self.jacobian = np.zeros((6, 6))

        # eef and force sensor offset setting
        quat_gripper_body = trans_quat.quat2mat([0.923879, 0, 0, -0.382684])
        quat_grasping_frame = trans_quat.quat2mat([0.7071, 0, 0, 0.7071])
        self.left_eef_offset = np.array([0, 0, 0.146 + 0.0495])
        self.left_eef_offset_rotm = quat_gripper_body @ quat_grasping_frame

        self.right_eef_offset = np.array([-0.0785, 0, 0.0644 + 0.0525])
        self.right_eef_offset_rotm = trans_quat.quat2mat([1.34924e-11, -3.67321e-06, 1, -3.67321e-06])  # 注意四元数的顺序可能需要与库相对应

        self.left_force_frame_offset = np.array([0, 0, 0.0495])
        self.right_force_frame_offset = np.array([0.00044323, -0.00044323, 0.0395])

        ####################
        self.work_space_xy_limit = 4
        self.work_space_z_limit = 8
        self.work_space_rollpitch_limit = np.pi * 5 / 180.0  # x, y
        self.work_space_yaw_limit = np.pi * 10 / 180.0   # z
        self.work_space_origin = np.array([0.50, 0, 0.1])  # 工作目标位置的原点位置，即孔所在的位置
        self.work_space_origin_rotm = np.array([[0, 0, 1],
                                                [0, 1, 0],
                                                [-1, 0, 0]])  #这个没用了

        ####################
        self.goal = np.array([0, 0, 0])
        self.goal_ori = np.array([0, 0, 0])
        self.noise_level = 0.2
        self.ori_noise_level = 0.5
        self.use_noisy_state = False
        self.state_offset = np.zeros(18)  # 状态补偿
        self.force_noise = True
        self.force_noise_level = 0.2
        self.force_limit = 20
        self.evaluation = True  # self.Render
        self.moving_pos_threshold = 2.5
        self.moving_ori_threshold = 4

        # RL setting
        self.obs_high = [self.work_space_xy_limit, self.work_space_xy_limit, self.work_space_z_limit,
                         self.work_space_rollpitch_limit, self.work_space_rollpitch_limit, self.work_space_yaw_limit,
                         0.1, 0.1, 0.1, 0.1, 0.1, 0.1,
                         10, 10, 10, 10, 10, 10]
        self.obs_high = np.array(self.obs_high)  # 本代码没明显用途

        self.observation_space = spaces.Box(low=-1., high=1., shape=self.get_RL_obs().shape,
                                            dtype=np.float32)  # self.get_RL_obs()
        self.action_space = spaces.Box(low=-np.ones(12), high=np.ones(12), dtype=np.float32)  # np.ones(12)  # 用于和强化学习算法定义使用，状态空间和动作空间大小定义

        ######### 动作的限制 ##########
        self.action_vel_high = 0.1 * np.array([1, 1, 0.5, 1, 1, 1])
        self.action_vel_low = -0.1 * np.ones(6)

        self.action_pos_high = np.array([6.2832, 4.6251, 3.0543, 4.6251, 6.2832, 6.2832])
        self.action_pos_low =  -np.array([6.2832, 1.4835, 3.0543, 1.4835, 6.2832, 6.2832])

        self.action_kp_high = 200 * np.array([1, 1, 1, 1, 1, 1])
        self.action_kp_low = 1 * np.array([1, 1, 1, 1, 1, 1])
        #####################


        ############ init #############
        self.render = render
        if self.render:
            self.viewer = mujoco.viewer.launch_passive(self.m, self.d)  # mujoco 自带可视化工具，可视化
            # 假设 viewer 是通过之前代码段创建的
            self.viewer.cam.lookat[:] = [0.3, 0.8, 0]  # 设置新的焦点
            self.viewer.cam.distance = 4  # 设置相机到焦点的距离
            self.viewer.cam.azimuth = 0  # 设置水平旋转角度
            self.viewer.cam.elevation = -40  # 设置仰角
        self.reset()
        # mujoco.mj_step(self.m, self.d)  # 前进

        # get robot state
        self.get_pose_()  # 位置、速度
        self.get_force_sensor_data()  # 力
        # self.force_calibration()  # 力标定

    def get_pose_(self):
        # get the both robot link6 pose and velocity
        # geom_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_GEOM, 'link6')
        # left_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, 'left_link6')
        # right_id = mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_BODY, 'right_link6')


        # Get position and rotation matrix of the end-effector (Link 6)
        left_link6_pos = self.d.xpos[self.left_link_id]
        print(left_link6_pos)
        left_link6_rotm = self.d.xmat[self.left_link_id].reshape([3,3])
        right_link6_pos = self.d.xpos[self.right_link_id]
        right_link6_rotm = self.d.xmat[self.right_link_id].reshape([3, 3])

        trans_left = self.left_base_rotm.T
        trans_right = self.right_base_rotm.T
        left_link6_pos_base = trans_left @ (left_link6_pos - self.left_base_pos)
        right_link6_pos_base = trans_right @ (right_link6_pos - self.right_base_pos)
        left_link6_rotm_base = trans_left @  left_link6_rotm
        right_link6_rotm_base = trans_right @  right_link6_rotm


        # link6_pos = self.d.geom_xpos[geom_id]
        # link6_rot_mat = self.d.geom_xmat[geom_id].reshape([3, 3]) @ np.array([[1.0000, 0.0008, 0.0003],
        #                                                                       [-0.0008, 0.8373, 0.5467],
        #                                                                       [0.0002, -0.5467, 0.8373]]).T #？

        # Convert rotation matrix to quaternion
        # link6_rot_quat = np.zeros(4)
        # mujoco.mju_mat2Quat(link6_rot_quat,link6_rot_mat)
        left_link6_rot_quat = trans_quat.mat2quat(left_link6_rotm_base)
        right_link6_rot_quat = trans_quat.mat2quat(right_link6_rotm_base)
        # Calculate Jacobian
        # jacp = np.zeros([3, self.m.nv])
        # jacr = np.zeros([3, self.m.nv])
        # mujoco.mj_jacGeom(self.m, self.d, jacp, jacr, geom_id)
        # Full_jacobian = np.concatenate([jacp, jacr])
        #
        # # Calculate Cartesian velocity of Link 6
        # link6_vel = np.dot(Full_jacobian, self.d.qvel[:8])  # Assuming 8 DoFs

        # Combine position, orientation (quaternion), and velocity
        dual_pose = np.concatenate([left_link6_pos_base, left_link6_rot_quat, right_link6_pos_base, right_link6_rot_quat]) #(3 + 4) * 2 = 14

        # update robot state
        # self.jacobian = Full_jacobian[:6, :6]
        self.pose_ = dual_pose

        self.left_joint_pos = self.d.qpos[self.left_joint_id - 5 :self.left_joint_id  + 1].copy()
        self.right_joint_pos = self.d.qpos[self.right_joint_id - 5 :self.right_joint_id + 1].copy()
        self.left_joint_vel = self.d.qvel[self.left_joint_id  - 5 :self.left_joint_id  + 1].copy()
        self.right_joint_vel = self.d.qvel[self.right_joint_id - 5 :self.right_joint_id + 1].copy()

        return dual_pose
    def get_link6_pose_(self, left_eef_pos, left_eef_rotm, right_eef_pos, right_eef_rotm):
        # left_link6_pos = dual_link6_pose_[:3]
        # left_link6_rotm = trans_quat.quat2mat(dual_link6_pose_[3:7])
        # right_link6_pos = dual_link6_pose_[7:10]
        # right_link6_rotm = trans_quat.quat2mat(dual_link6_pose_[10:14])

        # link6_vel = link6_pose_vel[7:]
        left_link6_pos = left_eef_pos - left_eef_rotm @ (self.left_eef_offset_rotm.T @ self.left_eef_offset)
        left_link6_rotm = left_eef_rotm @ self.left_eef_offset_rotm.T
        right_link6_pos = right_eef_pos - right_eef_rotm @ (self.right_eef_offset_rotm.T @ self.right_eef_offset)
        # right_link6_pos = right_eef_pos + right_eef_rotm @ self.right_eef_offset #?
        right_link6_rotm = right_eef_rotm @ self.right_eef_offset_rotm.T

        # Let's forget kinematics and assume eef and link 6 have the same vel
        # OH we cannot do that
        #齐次坐标系下刚体运动的速度和角速度分析
        # https://gaoyichao.com/Xiaotu/?book=math_physics_for_robotics&title=%E5%A4%9A%E4%B8%AA%E5%9D%90%E6%A0%87%E7%B3%BB%E4%B8%8B%E7%9A%84%E9%80%9F%E5%BA%A6%E5%92%8C%E8%A7%92%E9%80%9F%E5%BA%A6%E5%88%86%E6%9E%90
        # eef_vel = copy.copy(link6_vel)
        # eef_vel[:3] = link6_vel[:3] + self.skew_symmetric(link6_vel[3:6]) @ link6_rotm @ self.eef_offset

        # return eef_pos, eef_rotm, eef_vel
        return left_link6_pos, left_link6_rotm, right_link6_pos, right_link6_rotm
    def get_eef_pose_(self, dual_link6_pose_):
        left_link6_pos = dual_link6_pose_[:3]
        left_link6_rotm = trans_quat.quat2mat(dual_link6_pose_[3:7])
        right_link6_pos = dual_link6_pose_[7:10]
        right_link6_rotm = trans_quat.quat2mat(dual_link6_pose_[10:14])

        # link6_vel = link6_pose_vel[7:]

        # left_eef_pos = left_link6_pos + left_link6_rotm @ self.left_eef_offset  #?
        left_eef_rotm = left_link6_rotm @ self.left_eef_offset_rotm
        left_eef_pos = left_link6_pos + left_eef_rotm @ (self.left_eef_offset_rotm.T @ self.left_eef_offset)

        # right_eef_pos = right_link6_pos + right_link6_rotm @ self.right_eef_offset
        right_eef_rotm = right_link6_rotm @ self.right_eef_offset_rotm
        right_eef_pos = right_link6_pos + right_eef_rotm @ (self.right_eef_offset_rotm.T @ self.right_eef_offset)

        # Let's forget kinematics and assume eef and link 6 have the same vel
        # OH we cannot do that
        #齐次坐标系下刚体运动的速度和角速度分析
        # https://gaoyichao.com/Xiaotu/?book=math_physics_for_robotics&title=%E5%A4%9A%E4%B8%AA%E5%9D%90%E6%A0%87%E7%B3%BB%E4%B8%8B%E7%9A%84%E9%80%9F%E5%BA%A6%E5%92%8C%E8%A7%92%E9%80%9F%E5%BA%A6%E5%88%86%E6%9E%90
        # eef_vel = copy.copy(link6_vel)
        # eef_vel[:3] = link6_vel[:3] + self.skew_symmetric(link6_vel[3:6]) @ link6_rotm @ self.eef_offset

        # return eef_pos, eef_rotm, eef_vel
        return left_eef_pos, left_eef_rotm, right_eef_pos, right_eef_rotm
    def get_RL_obs(self):
        global eef_pos, eef_rotm
        # 此函数原作者用于轴孔装配的强化学习，将机器人tcp姿态与self.work_space_origin进行了求差计算，即计算末端到孔的距离作为观测
        # 修改后直接将机器人末端的TCP位置作为观测，
        # eef_pos, eef_world_rotm, eef_vel = self.get_eef_pose_vel(self.pose_vel)  # eef_pos, eef_world_rotm, eef_vel 末端位置，位姿， 速度，将link6 位置和速度变换到end-effector上

        # eef_pos = eef_pos - self.work_space_origin  # work_space_origin， work_space_origin_rotm 孔的位姿
        # eef_rotm = eef_world_rotm @ self.work_space_origin_rotm.T
        # eef_eul = trans_eul.mat2euler(eef_rotm)  # 转欧拉角

        left_eef_pos, left_eef_rotm, right_eef_pos, right_eef_rotm = self.get_eef_pose_(self.pose_)
        left_eef_eul = trans_eul.mat2euler(left_eef_rotm)
        right_eef_eul = trans_eul.mat2euler(right_eef_rotm)


        # self.left_force_frame_offset = np.array([0, 0, 0.0495])
        # self.right_force_frame_offset = np.array([0.00044323, -0.00044323, 0.0395])
        right_world_force = np.zeros(6)
        left_world_force = np.zeros(6)
        # eef_force = self.left_force_sensor_data - self.force_offset  # force_offset 和 force_frame_offset
        left_eef_force = self.left_force_sensor_data
        left_world_force[:3] = left_eef_rotm @ left_eef_force[:3]
        left_world_force[3:6] = left_eef_rotm @ left_eef_force[3:6]
        right_eef_force = self.right_force_sensor_data
        right_world_force[:3] = right_eef_rotm @ right_eef_force[:3]
        right_world_force[3:6] = right_eef_rotm @ right_eef_force[3:6]

        if self.force_noise:
            left_world_force = left_world_force + np.random.normal(0, self.force_noise_level, 6)
            right_world_force = right_world_force + np.random.normal(0, self.force_noise_level, 6)
        left_world_force = np.clip(left_world_force, -10, 10)  # 力的大小进行限制
        right_world_force = np.clip(right_world_force, -10, 10)  # 力的大小进行限制
        # state = np.concatenate([100 * eef_pos, eef_eul, eef_vel, world_force])
        state = np.concatenate([left_eef_pos, left_eef_eul, left_world_force, right_eef_pos, right_eef_eul, right_world_force])
        # state = np.clip(state, -self.obs_high, self.obs_high)
        if self.use_noisy_state:
            return state + self.state_offset
        else:
            return state
    def get_force_sensor_data(self):
        # get force sensor data

        # Get address and dimension of the sensor
        left_adr = self.m.sensor_adr[self.left_force_id]
        left_dim = self.m.sensor_dim[self.left_force_id]
        left_force = np.copy(self.d.sensordata[left_adr:left_adr + left_dim])
        right_adr = self.m.sensor_adr[self.left_force_id]
        right_dim = self.m.sensor_dim[self.left_force_id]
        right_force = np.copy(self.d.sensordata[right_adr:right_adr + right_dim])

        # get torque sensor data
        left_adr = self.m.sensor_adr[self.left_torque_id]
        left_dim = self.m.sensor_dim[self.left_torque_id]
        left_torque = np.copy(self.d.sensordata[left_adr:left_adr + left_dim])
        right_adr = self.m.sensor_adr[self.right_torque_id]
        right_dim = self.m.sensor_dim[self.right_torque_id]
        right_torque = np.copy(self.d.sensordata[right_adr:right_adr + right_dim])
        left_force_torque = np.concatenate([left_force, left_torque])
        right_force_torque = np.concatenate([right_force, right_torque])
        # update robot state
        self.left_force_sensor_data = left_force_torque
        self.right_force_sensor_data = right_force_torque

        return left_force_torque, right_force_torque


    def step(self, action):
        # step function for RL
        # desired_vel, desired_kp = self.process_action(action)  # self.process_action(action)

        left_desired_pose = action[:7]
        right_desired_pose = action[7:]
        print(left_desired_pose)
        left_desired_pos = left_desired_pose[:3]
        right_desired_pos = right_desired_pose[:3]
        left_desired_rotm = trans_quat.quat2mat(left_desired_pose[3:])
        right_desired_rotm = trans_quat.quat2mat(right_desired_pose[3:])
        left_eef_pos, left_eef_rotm, right_eef_pos, right_eef_rotm = self.get_eef_pose_(self.pose_)

        left_relative_rotm = R.from_matrix(left_eef_rotm) * R.from_matrix(left_desired_rotm).inv()
        right_relative_rotm = R.from_matrix(right_eef_rotm) * R.from_matrix(right_desired_rotm).inv()
        left_rotvec = left_relative_rotm.as_rotvec()
        right_rotvec = right_relative_rotm.as_rotvec()

        left_angular_vel = left_rotvec * self.HZ_action
        right_angular_vel = right_rotvec * self.HZ_action
        left_pos_vel = (left_desired_pos - left_eef_pos) * self.HZ_action
        right_pos_vel = (right_desired_pos - right_eef_pos) * self.HZ_action
        left_desired_vel = np.concatenate([left_pos_vel, left_angular_vel])
        right_desired_vel = np.concatenate([right_pos_vel, right_angular_vel])

        # self.adm_kp, self.adm_m 在类里面直接定义
        self.adm_kd = 2 * np.sqrt(np.multiply(self.adm_kp, self.adm_m))
        init_ob = self.get_RL_obs()

        left_link6_pos, left_link6_rotm, right_link6_pos, right_link6_rotm = self.get_link6_pose_(left_desired_pos,
                                                                                                  left_desired_rotm,
                                                                                                  right_desired_pos,
                                                                                                  right_desired_rotm)
        #
        left_q_target = self.kdl.ik(self.d.qpos[self.left_joint_id - 5: self.left_joint_id + 1], left_link6_pos,
                                    left_link6_rotm)  # eft_desired_pos, left_desired_rotm 都是eef的期望位置，而ik是要根据link6计算，添加函数，根据eef计算link6
        right_q_target = self.kdl.ik(self.d.qpos[self.right_joint_id - 5: self.right_joint_id + 1], right_link6_pos,
                                     right_link6_rotm)
        target_pos = np.concatenate([left_q_target, right_q_target])
        # keep the same action for a short time
        for i in range(10):
            if i == 0:
                left_eef_pos_old, left_eef_rotm_old, right_eef_pos_old, right_eef_rotm_old = self.get_eef_pose_(self.pose_)

            ob = self.get_RL_obs()
            curr_force = ob[12:]
            off_work_space = False

            done = False
            #######################
            # 获取机器人EEF处的位置，不需要获取速度，通过ACTION获取EEF处的期望位置，直接输入导纳控制器
            # 在控制器内，直接在EEF处进行计算，然后将期望位置投影到linK6，最后进行IK计算，投影到关节空间
            #######################
            left_eef_pos, left_eef_rotm, right_eef_pos, right_eef_rotm = self.get_eef_pose_(self.pose_)
            # eef_pos, eef_rotm, eef_vel = self.get_eef_pose_(self.pose_)
            # link6_pos, link6_rotm, link6_vel = self.get_link6_pose_vel_from_eef(eef_pos, eef_rotm, desired_vel)
            self.left_adm_pose_ref[:3] = left_desired_pos
            self.left_adm_pose_ref[3:7] = trans_quat.mat2quat(left_desired_rotm)
            self.right_adm_pose_ref[:3] = right_desired_pos
            self.right_adm_pose_ref[3:7] = trans_quat.mat2quat(right_desired_rotm)
            self.left_adm_vel_ref = left_desired_vel
            self.right_adm_vel_ref = right_desired_vel
            ##差分获得机器人当前eef的速度

            left_relative_rotm = R.from_matrix(left_eef_rotm) * R.from_matrix(left_eef_rotm_old).inv()
            right_relative_rotm = R.from_matrix(right_eef_rotm) * R.from_matrix(right_eef_rotm_old).inv()
            left_rotvec = left_relative_rotm.as_rotvec()
            right_rotvec = right_relative_rotm.as_rotvec()

            left_angular_vel = left_rotvec * self.HZ
            right_angular_vel = right_rotvec * self.HZ
            left_pos_vel = (left_eef_pos_old - left_eef_pos) * self.HZ
            right_pos_vel = (right_eef_pos_old - right_eef_pos) * self.HZ
            self.left_eef_vel = np.concatenate([left_pos_vel, left_angular_vel])
            self.right_eef_vel = np.concatenate([right_pos_vel, right_angular_vel])
            ##
            ######################测试注释掉######################
            # left_q_target, right_q_target = self.admittance_control()
            # target_pos = np.concatenate([left_q_target, right_q_target])
            ####################################################

            # left_eef_pos_old, left_eef_rotm_old, right_eef_pos_old, right_eef_rotm_old = self.get_eef_pose_(self.pose_)
            #

            self.set_joint_pos(target_pos)  # target_pos发生了意想不到的变化

#####测试用，测试后删除#####
            # self.d.qpos[self.left_joint_id - 5:self.left_joint_id + 1] = left_q_target
            # self.d.qpos[self.right_joint_id - 5:self.right_joint_id + 1] = right_q_target
##########################
            self.sim_step()  #

        ob = self.get_RL_obs()
        # evalute reward
        dist = np.linalg.norm(ob[0:3] - self.goal)
        # if dist < 0.3:
        #     done = False
        #     reward = 10
        # else:
        #     done = False
        #     reward = np.power(10, 1 - dist)
        reward = -dist
        if self.evaluation and np.linalg.norm(ob[2] - self.goal[2]) < 0.3:  # z 方向达到目标位置
            done = True
        return ob, reward, done, dict(reward_dist=reward)

    def sim_step(self):
        # apply computed torque control
        self.computed_torque_control_robopal()
        mujoco.mj_step(self.m, self.d)
        if self.render:
            self.viewer.sync()
        # get robot state
        self.get_pose_()
        self.get_force_sensor_data()

    def reset(self):
        # set eef init pose 可以用这个测试kdl
        # left_init_c_pos = np.array([-0.3, -0, 0.4])
        # left_init_c_rotm = trans_eul.euler2mat((90 * np.pi) / 180, (180 * np.pi) / 180, (-90 * np.pi) / 180,  'sxyz')
        # right_init_c_pos = np.array([-0.5, 0.0, 0.40])
        # right_init_c_rotm = trans_eul.euler2mat(0.0, np.pi, np.pi, 'sxyz')
        # print(self.d.qpos[self.left_joint_id - 5: self.left_joint_id + 1])
        # print(np.array([self.left_joint_id - 5, self.left_joint_id + 1]))
        # left_init_j_pos = self.kdl.ik(self.d.qpos[self.left_joint_id - 5 :self.left_joint_id + 1], left_init_c_pos, left_init_c_rotm)
        # right_init_j_pos = self.kdl.ik(self.d.qpos[self.right_joint_id - 5:self.right_joint_id + 1], right_init_c_pos, right_init_c_rotm)
        print("#"*20)
        # 上述时逆运动学设置初始位置，下面是正运动学设置
        left_init_j_pos = np.array([0,np.pi/2,np.pi / 2,np.pi ,np.pi / 2,(135 * np.pi) / 180])
        print(left_init_j_pos)
        right_init_j_pos = np.array([0,np.pi/2,np.pi / 2,np.pi/2 ,-np.pi / 2,np.pi / 2])
        self.d.qpos[self.left_joint_id - 5 :self.left_joint_id + 1] = left_init_j_pos
        self.d.qpos[self.right_joint_id - 5:self.right_joint_id + 1] = right_init_j_pos
        mujoco.mj_step(self.m, self.d)
        self.viewer.sync()
        r1, p1 = self.kdl.fk(left_init_j_pos)
        r2, p2 = self.kdl.fk(right_init_j_pos)
        # print(r1, r2, p1, p2)
        r1, r2 = np.array(r1), np.array(r2)
        quat_p1 = trans_quat.mat2quat(np.array(p1))
        quat_p2 = trans_quat.mat2quat(np.array(p2))
        dual_link6_pose_ = np.concatenate([r1, quat_p1, r2, quat_p2])
        # print(dual_link6_pose_)
        left_eef_pos, left_eef_rotm, right_eef_pos, right_eef_rotm = self.get_eef_pose_(dual_link6_pose_)

        # print(r)
        # print(p)
        # self.force_calibration()
        # Domain-randomization
        # self.state_offset[:2] = np.random.normal(0, np.array([self.noise_level, self.noise_level]))  # 随即孔、轴之间的互相距离
        # angle = np.random.normal(0, self.ori_noise_level / 180 * np.pi)
        # self.state_offset[5] = angle
        return self.get_RL_obs(), left_eef_pos, left_eef_rotm, right_eef_pos, right_eef_rotm

    def set_reference_traj(self, ref_joint, ref_vel, ref_acc):
        assert (
                ref_joint.shape == (12,) and ref_vel.shape == (12,) and ref_acc.shape == (12,)
        )
        self.ref_joint = ref_joint
        self.ref_vel = ref_vel
        self.ref_acc = ref_acc
        # self.ref_joint = np.concatenate([ref_joint, np.zeros(2)])
        # self.ref_vel = np.concatenate([ref_vel, np.zeros(2)])
        # self.ref_acc = np.concatenate([ref_acc, np.zeros(2)])

    # def set_joint_velocity(self, target_vel):
    #     T = 1 / self.HZ
    #     target_pos = self.joint_pos + T * (self.joint_vel + target_vel) / 2  # x_t+1 = x_t + 1/2 * (v_t + v_t+1) * T
    #     target_acc = (target_vel - self.joint_vel) / T
    #     self.set_reference_traj(target_pos, target_vel, target_acc)

    def set_joint_pos(self, target_pos):
        T = 1 / self.HZ
        target_pos = target_pos.copy()
        # left_target_vel = ((target_pos[:6] - self.left_joint_pos) * 2 ) / T - self.left_joint_vel
        left_target_vel = (target_pos[:6] - self.left_joint_pos) / T
        left_target_acc = (left_target_vel - self.left_joint_vel) / T
        # right_target_vel = ((target_pos[6:] - self.right_joint_pos) * 2 ) / T - self.right_joint_vel
        right_target_vel = (target_pos[6:] - self.right_joint_pos) / T
        right_target_acc = (right_target_vel - self.right_joint_vel) / T
        target_vel = np.concatenate([left_target_vel, right_target_vel])
        target_acc = np.concatenate([left_target_acc, right_target_acc])

        self.set_reference_traj(target_pos, target_vel, target_acc)

    def computed_torque_control_robopal(self):
        M = np.zeros((self.m.nv, self.m.nv))
        mujoco.mj_fullM(self.m, M, self.d.qM)
        M_left = M[self.left_joint_id - 5: self.left_joint_id + 1, self.left_joint_id - 5: self.left_joint_id + 1]
        M_right = M[self.right_joint_id - 5: self.right_joint_id + 1, self.right_joint_id - 5: self.right_joint_id + 1]
        c_g_left = self.d.qfrc_bias[self.left_joint_id - 5: self.left_joint_id + 1]
        c_g_right = self.d.qfrc_bias[self.right_joint_id - 5 : self.right_joint_id + 1]

        # 也可以称之为是一种前馈+反馈控制器
        acc_desire_left = -self.joint_kp * (self.left_joint_pos - self.ref_joint[:6]) - self.joint_kd * (self.left_joint_vel)
        acc_desire_right = -self.joint_kp * (self.right_joint_pos - self.ref_joint[6:]) - self.joint_kd * (self.right_joint_vel)
        tau_left = np.dot(M_left, acc_desire_left) + c_g_left
        tau_right = np.dot(M_right, acc_desire_right) + c_g_right

        self.d.ctrl[self.left_actuator_id - 5: self.left_actuator_id + 1] = np.clip(tau_left, -20, 20)
        self.d.ctrl[self.right_actuator_id - 5: self.right_actuator_id + 1] = np.clip(tau_right, -20, 20)

        # Set screw tool
        self.d.ctrl[self.screw_shaft_id] = 0  # gripper_pose should be defined or passed as an argument
        self.d.ctrl[self.left_mouth_id] = 0
        self.d.ctrl[self.left_mouth_id] = 0
        # Set gripper
        self.d.ctrl[self.left_gripper_id] = 0
        self.d.ctrl[self.right_gripper_id] = 0

    def computed_torque_control(self):
        # Assuming self.m and self.d are equivalent to m and d in the C++ code
        # 前馈 - 反馈控制（feedforward and feedback control）
        # The low-level position/velocity controller is achieved via a Positional-Integral (PI) control law with feed-forward terms to cancel gravity and friction
        # Compute inverse dynamics forces
        mujoco.mj_rne(self.m, self.d, 0, self.d.qfrc_inverse)
        for i in range(self.m.nv):
            self.d.qfrc_inverse[i] += (self.m.dof_armature[i] * self.d.qacc[i] -
                                       self.d.qfrc_passive[i] -
                                       self.d.qfrc_constraint[i])

        # Error and error derivative
        # left_e = self.d.qpos[self.left_joint_id - 5 :self.left_joint_id + 1] - self.ref_joint[:6]  # ref_joint should be defined or passed as an argument
        left_e = self.left_joint_pos - self.ref_joint[:6]  # ref_joint should be defined or passed as an argument
        left_e_dot = self.left_joint_vel - self.ref_vel[:6]
        # left_e_dot = self.d.qvel[self.left_joint_id - 5 :self.left_joint_id + 1] - self.ref_vel[:6]  # ref_vel should be defined or passed as an argument
        right_e = self.right_joint_pos - self.ref_joint[6:]
        right_e_dot = self.right_joint_vel - self.ref_vel[6:]
        # right_e = self.d.qpos[self.right_joint_id - 5 : self.right_joint_id + 1] - self.ref_joint[6:]  # ref_joint should be defined or passed as an argument
        # right_e_dot = self.d.qvel[self.right_joint_id - 5 : self.right_joint_id + 1] - self.ref_vel[6:]  # ref_vel should be defined or passed as an argument

        # Control law components
        left_kve_dot = np.multiply(self.joint_kd, left_e_dot)  # kv should be defined or passed as an argument
        left_kpe = np.multiply(self.joint_kp, left_e)  # kp should be defined or passed as an argument
        # left_inertial_pd = self.ref_acc[:6] - left_kve_dot - left_kpe  # ref_acc should be defined or passed as an argument
        left_inertial_pd = - left_kve_dot - left_kpe
        right_kve_dot = np.multiply(self.joint_kd, right_e_dot)  # kv should be defined or passed as an argument
        right_kpe = np.multiply(self.joint_kp, right_e)  # kp should be defined or passed as an argument
        # right_inertial_pd = self.ref_acc[6:] - right_kve_dot - right_kpe  # ref_acc should be defined or passed as an argument
        right_inertial_pd = - right_kve_dot - right_kpe
        # #在上述约束力计算中，self.d.qfrc_constraint[i] 可能包含了来自环境的外部力 ( \tau_{\text{env}} ) —— 比如接触反作用力等。
        # 最终通过控制器施加到机器人关节上的力矩是将环境力和期望的控制力矩叠加的结果，其旨在驱动机器人执行期望的运动，同时考虑与环境的交互。

        # Compute full inertia matrix
        M = np.zeros((self.m.nv, self.m.nv))
        mujoco.mj_fullM(self.m, M, self.d.qM)
        M_left = M[self.left_joint_id - 5: self.left_joint_id + 1, self.left_joint_id - 5: self.left_joint_id + 1]
        M_right = M[self.right_joint_id - 5: self.right_joint_id + 1, self.right_joint_id - 5: self.right_joint_id + 1]
        # M_robot = M[:self.m.nu, :self.m.nu]  # Assuming self.m.nu is the number of actuators

        # Compute the control torques
        left_inertial_torque = np.dot(M_left, left_inertial_pd)
        right_inertial_torque = np.dot(M_right, right_inertial_pd)

        # Apply control and inverse dynamics torques
        self.d.ctrl[self.left_actuator_id - 5: self.left_actuator_id + 1] = np.clip(left_inertial_torque +
                                                                                    self.d.qfrc_inverse[self.left_joint_id - 5: self.left_joint_id + 1], -20, 20)
        self.d.ctrl[self.right_actuator_id - 5: self.right_actuator_id + 1] = np.clip(right_inertial_torque +
                                                                                    self.d.qfrc_inverse[self.right_joint_id - 5: self.right_joint_id + 1], -20, 20)

        # Set screw tool
        self.d.ctrl[self.screw_shaft_id] = 0  # gripper_pose should be defined or passed as an argument
        self.d.ctrl[self.left_mouth_id] = 0
        self.d.ctrl[self.left_mouth_id] = 0
        # Set gripper
        self.d.ctrl[self.left_gripper_id] = 0
        self.d.ctrl[self.right_gripper_id] = 0

    def admittance_control(self, ctl_ori=True):
        ## Get robot motion from desired dynamics
        # self.left_adm_pose_ref[:3] = left_desired_pos
        # self.left_adm_pose_ref[3:7] = trans_quat.mat2quat(left_desired_rotm)
        # self.right_adm_pose_ref[:3] = right_desired_pos
        # self.right_adm_pose_ref[3:7] = trans_quat.mat2quat(right_desired_rotm)
        # self.left_adm_vel_ref = left_desired_vel
        # self.right_adm_vel_ref = right_desired_vel

        left_eef_pos, left_eef_rotm, right_eef_pos, right_eef_rotm = self.get_eef_pose_(self.pose_)
        left_eef_pos_d, left_eef_rotm_d, right_eef_pos_d, right_eef_rotm_d = self.left_adm_pose_ref[:3], trans_quat.quat2mat(self.left_adm_pose_ref[3:7]), self.right_adm_pose_ref[:3], trans_quat.quat2mat(self.right_adm_pose_ref[3:7])
        left_eef_desired_vel, right_eef_desired_vel = self.left_adm_vel_ref, self.right_adm_vel_ref
        left_eef_vel, right_eef_vel = self.left_eef_vel, self.right_eef_vel

        # process force
        # world_force = np.zeros(6)
        # force_limit = np.array([10, 10, 10, 1, 1, 1])
        # eef_force = self.force_sensor_data[0:3] - self.force_offset[:3]
        # eef_torque = self.force_sensor_data[3:6] - self.force_offset[3:6]
        # world_force[:3] = self.force_frame_offset @ eef_rotm @ eef_force
        # world_force[3:] = self.force_frame_offset @ eef_rotm @ eef_torque
        # world_force = np.clip(world_force, -force_limit, force_limit)

        right_world_force = np.zeros(6)
        left_world_force = np.zeros(6)
        # eef_force = self.left_force_sensor_data - self.force_offset  # force_offset 和 force_frame_offset
        left_eef_force = self.left_force_sensor_data
        left_world_force[:3] = left_eef_rotm @ left_eef_force[:3]
        left_world_force[3:6] = left_eef_rotm @ left_eef_force[3:6]
        right_eef_force = self.right_force_sensor_data
        right_world_force[:3] = right_eef_rotm @ right_eef_force[:3]
        right_world_force[3:6] = right_eef_rotm @ right_eef_force[3:6]
        if self.force_noise:
            left_world_force = left_world_force + np.random.normal(0, self.force_noise_level, 6)
            right_world_force = right_world_force + np.random.normal(0, self.force_noise_level, 6)
        left_world_force = np.clip(left_world_force, -10, 10)  # 力的大小进行限制
        right_world_force = np.clip(right_world_force, -10, 10)  # 力的大小进行限制

        # dynamics
        left_e = np.zeros(6)
        right_e = np.zeros(6)
        adm_pos = np.zeros(3)
        adm_rotm = np.zeros((3,3))

        left_e[:3] = left_eef_pos - left_eef_pos_d  #所有的这些位置差距都被变化到世界坐标系下（机器人基坐标系）进行表示，包括这里的eef_pos也是在世界坐标系下的一种表示。
        print('left_eef_rotm', left_eef_rotm)
        print('left_eef_rotm_d',left_eef_rotm_d)
        if ctl_ori:
            eRd = left_eef_rotm @ left_eef_rotm_d.T
            dorn = trans_quat.mat2quat(eRd)
            do = dorn[1:]
            left_e[3:] = do

        e_dot = left_eef_vel - left_eef_desired_vel
        MA = 1 * left_world_force - np.multiply(self.adm_kp, left_e) - np.multiply(self.adm_kd, e_dot)
        adm_acc = np.divide(MA, self.adm_m)
        T = 1 / self.HZ
        adm_vel = left_eef_vel + adm_acc * T  # This vel is for eef not link6, which we can control
        left_axis_of_rotation = adm_vel[3:] / np.linalg.norm(adm_vel[3:])
        left_angle_of_rotation = np.linalg.norm(adm_vel[3:]) * T
        left_rotation_increment = R.from_rotvec(left_axis_of_rotation * left_angle_of_rotation)
        left_adm_pos = left_eef_pos + adm_vel[:3] * T
        left_adm_rotm = left_eef_rotm @ left_rotation_increment.as_matrix() #？

        right_e[:3] = right_eef_pos - right_eef_pos_d  #所有的这些位置差距都被变化到世界坐标系下（机器人基坐标系）进行表示，包括这里的eef_pos也是在世界坐标系下的一种表示。
        if ctl_ori:
            eRd = right_eef_rotm @ right_eef_rotm_d.T
            dorn = trans_quat.mat2quat(eRd)
            do = dorn[1:]
            right_e[3:] = do

        e_dot = right_eef_vel - right_eef_desired_vel
        MA = 1 * right_world_force - np.multiply(self.adm_kp, right_e) - np.multiply(self.adm_kd, e_dot)
        adm_acc = np.divide(MA, self.adm_m)

        adm_vel = right_eef_vel + adm_acc * T  # This vel is for eef not link6, which we can control
        right_axis_of_rotation = adm_vel[3:] / np.linalg.norm(adm_vel[3:])
        right_angle_of_rotation = np.linalg.norm(adm_vel[3:]) * T
        right_rotation_increment = R.from_rotvec(right_axis_of_rotation * right_angle_of_rotation)
        right_adm_pos = right_eef_pos + adm_vel[:3] * T
        right_adm_rotm = right_eef_rotm @ right_rotation_increment.as_matrix() #？

        # if not ctl_ori:
        #     adm_vel[3:] = 0 * adm_vel[3:]

        left_q_target = self.kdl.ik(self.d.qpos[self.left_joint_id - 5 : self.left_joint_id + 1], left_adm_pos, left_adm_rotm)
        right_q_target = self.kdl.ik(self.d.qpos[self.right_joint_id - 5: self.right_joint_id + 1], right_adm_pos, right_adm_rotm)
        # link6_pos, link6_rotm, link6_vel = self.get_link6_pose_vel_from_eef(eef_pos, eef_rotm, adm_vel)
        # target_joint_vel = np.linalg.pinv(self.jacobian) @ link6_vel
        # target_joint_vel = np.clip(target_joint_vel, -np.array([0.4, 0.4, 0.4, 0.5, 0.5, 0.5]) / 2,
        #                            np.array([0.4, 0.4, 0.4, 0.5, 0.5, 0.5]) / 2)
        # print("=="*50)
        # print("e",e)
        # print("e_dot",e_dot)
        # print("adm_vel",adm_vel)
        # print("link6_vel",link6_vel)
        # print("world_force",world_force)
        # print("control",self.d.ctrl[:8])
        return left_q_target, right_q_target

    # def admittance_control_link6(self):
    #     global eef_pos, eef_rotm
    #     link6_pos = self.pose_vel[:3]
    #     link6_rotm = trans_quat.quat2mat(self.pose_vel[3:7])
    #     link6_vel = self.pose_vel[7:]
    #     link6_pos_d = self.adm_pose_ref[:3]
    #     link6_rotm_d = trans_quat.quat2mat(self.adm_pose_ref[3:7])
    #     link6_vel_d = self.adm_vel_ref
    #
    #     # process force
    #     world_force = np.zeros(6)
    #     force_limit = np.array([10, 10, 10, 1, 1, 1])
    #     eef_force = self.force_sensor_data[0:3] - self.force_offset[:3]
    #     eef_torque = self.force_sensor_data[3:6] - self.force_offset[3:6]
    #     world_force[:3] = self.force_frame_offset @ eef_rotm @ eef_force
    #     world_force[3:] = self.force_frame_offset @ eef_rotm @ eef_torque
    #     world_force = np.clip(world_force, -force_limit, force_limit)
    #
    #     e = np.zeros(6)
    #     e[:3] = link6_pos - link6_pos_d
    #     eRd = link6_rotm @ link6_rotm_d.T
    #     dorn = trans_quat.mat2quat(eRd)
    #     do = dorn[1:]
    #     e[3:] = do
    #
    #     e_dot = link6_vel - link6_vel_d
    #     MA = 1 * world_force - np.multiply(self.adm_kp, e) - np.multiply(self.adm_kd, e_dot)
    #     adm_acc = np.divide(MA, self.adm_m)
    #     T = 1 / self.HZ
    #     adm_vel = link6_vel + adm_acc * T  # This vel is for eef not link6, which we can control
    #     target_joint_vel = np.linalg.pinv(self.jacobian) @ adm_vel
    #     return target_joint_vel



    def process_action(self, action):
        # 这段代码用于在某些范围内，通常是 -1 到 1 之间的标准化值）重新映射到一个特定的速度范围（即 self.action_vel_high 和 self.action_vel_low 之间），以便将其应用于机械臂的关节。
        # 这实质上是一个线性变换，它将智能体的输出（假设是一个在 [-1, 1] 之间的控制信号）转换为实际的关节速度命令。这个转换保证了智能体的输出能够映射到环境能接受的动作空间范围内。
        # Normalize actions
        desired_joint = np.clip(action[:6], -1, 1)
        # desired_kp = np.clip(action[6:12], -1, 1)
        desired_joint = (self.action_pos_high + self.action_pos_low) / 2 + np.multiply(desired_joint, (
                    self.action_pos_high - self.action_pos_low) / 2)
        # desired_kp = (self.action_kp_high + self.action_kp_low) / 2 + np.multiply(desired_kp, (
        #             self.action_kp_high - self.action_kp_low) / 2)
        return desired_joint

########################################################################


    # def Cartersian_vel_control(self, vel):
    #     target_joint_vel = np.linalg.pinv(self.jacobian) @ vel
    #
    #     # print("=="*50)
    #     # print("cartesian")
    #     return target_joint_vel


    # def get_link6_pose_vel_from_eef(self, eef_pos, eef_rotm, eef_vel):
    #     link6_rotm = copy.copy(eef_rotm)
    #     link6_pos = eef_pos - link6_rotm @ self.eef_offset
    #     link6_vel = copy.copy(eef_vel)
    #     link6_vel[:3] = eef_vel[:3] - self.skew_symmetric(eef_vel[3:6]) @ link6_rotm @ self.eef_offset
    #
    #     return link6_pos, link6_rotm, link6_vel

    # def skew_symmetric(self, vec):
    #     return np.array([[0, -vec[2], vec[1]],
    #                      [vec[2], 0, -vec[0]],
    #                      [-vec[1], vec[0], 0]])



    # def set_gripper(self, pose):
    #     pose = 0.42 if pose > 0.42 else pose
    #     self.gripper_pose = pose
    #
    # def force_calibration(self, H=100):
    #     """
    #     Calibrate force sensor reading
    #     H: force history horizon
    #     """
    #     force_history = np.zeros([H, 6])
    #     self.sim_step()  # sim_step
    #     self.set_reference_traj(
    #         self.joint_pos, 0 * self.joint_vel, 0 * self.joint_acc
    #     )
    #     for _ in range(H):
    #         self.sim_step()
    #         force_history[_, :] = self.force_sensor_data
    #     self.force_offset = np.mean(force_history[int(H / 2):], axis=0)


    # def sim(self):
    #     # set ref traj
    #     curr_pose_vel = self.get_pose_()
    #     eef_pos, eef_rotm, eef_vel = self.get_eef_pose_vel(self.pose_vel)
    #     eef_pos_d = eef_pos + np.array([0.0, 0.0, 0.1])
    #     eef_rotm_d = eef_rotm @ trans_eul.euler2mat(0, 0, 1 * np.pi / 6)
    #     eef_vel_d = np.zeros(6)
    #     link6_pos, link6_rotm, link6_vel = self.get_link6_pose_vel_from_eef(eef_pos_d, eef_rotm_d, eef_vel_d)
    #     self.adm_pose_ref = np.zeros(7)
    #     self.adm_pose_ref[:3] = link6_pos
    #     self.adm_pose_ref[3:7] = trans_quat.mat2quat(link6_rotm)
    #     # self.adm_pose_ref[:3] = curr_pose_vel[:3]+np.array([0.0,0.0,0.0])
    #     # delta_ori = trans_quat.mat2quat(trans_quat.quat2mat(curr_pose_vel[3:7])@trans_eul.euler2mat(0,0,1*np.pi/6))
    #     # self.adm_pose_ref[3:7] = delta_ori
    #     # self.adm_pose_ref = np.array([ 0.4899,  0.0352,  0.4076,  1, 0,  0,  0])
    #     # print(self.adm_pose_ref)
    #     start = time.time()
    #     while self.viewer.is_running() and time.time() - start < 3000:
    #         step_start = time.time()
    #
    #         # mj_step can be replaced with code that also evaluates
    #         # a policy and applies a control signal before stepping the physics.
    #         # eef_pos, eef_rotm, eef_vel = self.get_eef_pose_vel(self.pose_vel)
    #         # link6_pos, link6_rotm, link6_vel = self.get_link6_pose_vel_from_eef(eef_pos, eef_rotm,
    #         #                                                                     np.array([0.00,0,0,0.0,0.0,0.00]))
    #         target_joint_vel = self.admittance_control(ctl_ori=True)
    #         # target_joint_vel = self.Cartersian_vel_control(link6_vel)
    #         print(self.pose_vel[:3])
    #         print(trans_quat.quat2mat(self.pose_vel[3:7]))
    #         self.set_joint_velocity(target_joint_vel)
    #         # self.ref_joint = np.array([0, 0, 0, 0, -90, 0, 0.42, 0.42])/180*np.pi
    #         self.sim_step()
    #         # time.sleep(0.01)
    #         # print(self.d.time)
    #
    #         # Example modification of a viewer option: toggle contact points every two seconds.
    #         with self.viewer.lock():
    #             self.viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(self.d.time % 2)
    #
    #         # Pick up changes to the physics state, apply perturbations, update options from GUI.
    #         self.viewer.sync()
    #
    #         # Rudimentary time keeping, will drift relative to wall clock.
    #         time_until_next_step = self.m.opt.timestep - (time.time() - step_start)
    #         if time_until_next_step > 0:
    #             time.sleep(time_until_next_step)


register(
    id="mujoco_assembly-v0",
    entry_point=Dual_arm_env,
)
# register_env("mujoco_assembly-v0", Fanuc_mujoco_env)
if __name__ == '__main__':
    env = Dual_arm_env()
    # env.sim()
    for i in range(10):
        state, left_eef_pos, left_eef_rotm, right_eef_pos, right_eef_rotm = env.reset()
        print(left_eef_pos)
        print(left_eef_rotm)
        left_eef_quat = trans_quat.mat2quat(left_eef_rotm)
        right_eef_quat = trans_quat.mat2quat(right_eef_rotm)
        # time.sleep(100)
        for _ in range(100):
            start = time.time()
            # action = np.ones(14)  #我们对这里进行分解，action=np.ones(12)意思是控制两台机械臂的末端运动，即两个机械臂的末端绝对位置
            action = np.concatenate([left_eef_pos, left_eef_quat, right_eef_pos, right_eef_quat])
            print('**************action_before*****************')
            print(action)
            # ros 的指令从这里传入，我们首先只考虑控制一个机械臂
            # action[:6] = np.array([-0.2, 0, 0, 0, 0, 0])
            action[:2] += np.random.uniform(low=-0.01, high=0.01, size=2)
            print('**************action_after*****************')
            print(action)
            ##############

            ob, reward, done, _ = env.step(action)
            print('#####################################################################one epo##########################################################################')
            # print(ob[:3])
            # print(reward)
            # print(time.time() - start)
            time.sleep(0.1)