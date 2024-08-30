#! /home/yanji/anaconda3/envs/mujo/bin/python3
# -*- coding: utf-8 -*-
import os, sys, ctypes
jaka_lib_path = '/home/yanji/jaka_python'
os.environ['LD_LIBRARY_PATH'] = jaka_lib_path + ':' + os.environ.get('LD_LIBRARY_PATH', '')
ctypes.CDLL(os.path.join(jaka_lib_path, 'libjakaAPI.so'))
sys.path.append(jaka_lib_path)

import platform 
import rospy
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface
from moveit_msgs.msg import PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose, Point
import numpy as np
import transforms3d.quaternions as trans_quat
import transforms3d.euler as trans_eul
import time
from scipy.spatial.transform import Rotation as R
from pynput.keyboard import Key, Listener
import generate_grasping_pose as ge
from real_robot_control.msg import pose

import ransac_icp as rans
from utils import *
from real_robot_control.msg import gripper
from real_robot_control.srv import *



import jkrc
from pyRobotiqGripper import RobotiqGripper


##
# 集成所有规划的代码，全工作流程，执行这个Py即可
##

def multiarray_to_numpy(multiarray):
    rows = multiarray.layout.dim[0].size
    cols = multiarray.layout.dim[1].size
    return np.array(multiarray.data).reshape((rows, cols))

def callback(data):
    for i, matrix_msg in enumerate(data.matrices):  #enumerate 用于同时获取索引和实际值
        matrix = multiarray_to_numpy(matrix_msg)
        rospy.loginfo(f"Matrix {i+1}:\n{matrix}")


class Grasp_planning():
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化ROS节点
        rospy.init_node('grasp_planning')
        # 初始化场景对象
        self.scene = PlanningSceneInterface()

        ############添加物体############
        self.wall1 = 'wall1'
        self.wall2 = 'wall2'
        self.cabinet = 'cabinet'
        self.put = 'put'
        self.wall1_size = [0.35, 1.3, 2]
        self.wall2_size = [0.2, 0.7 + 0.15, 2]
        self.cabinet_size = [1.25, 0.2, 2]
        self.put_size = [0.02, 0.13, 0.054]

        ############添加物体############
        # 创建一个发布场景变化信息的发布者
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5)
        # 创建一个存储物体颜色的字典对象
        self.colors = dict()

        # 等待场景准备就绪
        rospy.sleep(1)
        # 初始化需要使用move group控制的机械臂中的arm group
        self.left_arm = MoveGroupCommander('left_arm')
        self.right_arm = MoveGroupCommander('right_arm')

        # 获取终端link的名称
        self.left_end_effector_link = self.left_arm.get_end_effector_link()
        self.right_end_effector_link = self.right_arm.get_end_effector_link()

        # 设置目标位置所使用的参考坐标系
        self.left_reference_frame = 'left_base'
        self.left_arm.set_pose_reference_frame(self.left_reference_frame)
        self.right_reference_frame = 'right_link0'
        self.right_arm.set_pose_reference_frame(self.right_reference_frame)
        self.world_reference_frame = 'base_link'

        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        self.left_arm.set_goal_position_tolerance(0.0005)
        self.left_arm.set_goal_orientation_tolerance(0.0005)
        self.right_arm.set_goal_position_tolerance(0.005)
        self.right_arm.set_goal_orientation_tolerance(0.03)

        # 当运动规划失败后，允许重新规划
        self.left_arm.allow_replanning(True)
        self.right_arm.allow_replanning(True)

        # 设置每次运动规划的时间限制：5s
        self.left_arm.set_planning_time(5)
        self.right_arm.set_planning_time(5)

        # 添加环境中的物体
        # 先移除残留的物体
        self.scene.remove_world_object(self.wall1)
        wall1_pose = PoseStamped()
        wall1_pose.header.frame_id = self.world_reference_frame
        wall1_pose.pose.position.x = self.wall1_size[0] / 2 + 0.9
        wall1_pose.pose.position.y = self.wall1_size[1] / 2 - 0.5
        wall1_pose.pose.position.z = self.wall1_size[2] / 2
        wall1_pose.pose.orientation.w = 1.0
        self.scene.add_box(self.wall1, wall1_pose, self.wall1_size)

        self.scene.remove_world_object(self.wall2)
        wall2_pose = PoseStamped()
        wall2_pose.header.frame_id = self.world_reference_frame
        wall2_pose.pose.position.x = self.wall2_size[0] / 2 + 0.9 
        wall2_pose.pose.position.y = self.wall2_size[1] / 2 + 0.8
        wall2_pose.pose.position.z = self.wall2_size[2] / 2
        wall2_pose.pose.orientation.w = 1.0
        self.scene.add_box(self.wall2, wall2_pose, self.wall2_size)

        self.scene.remove_world_object(self.cabinet)
        cabinet_pose = PoseStamped()
        cabinet_pose.header.frame_id = self.world_reference_frame
        cabinet_pose.pose.position.x = self.cabinet_size[0] / 2
        cabinet_pose.pose.position.y = self.cabinet_size[1] / 2 + 1.5 + 0.1
        cabinet_pose.pose.position.z = self.cabinet_size[2] / 2
        cabinet_pose.pose.orientation.w = 1.0
        self.scene.add_box(self.cabinet, cabinet_pose, self.cabinet_size)

        self.scene.remove_world_object(self.put)
        put_pose = PoseStamped()
        put_pose.header.frame_id = self.left_reference_frame
        put_pose.pose.position.x = -87.5/1000
        put_pose.pose.position.y = -300/1000
        put_pose.pose.position.z = -15/1000 + 0.054/2 +0.001
        put_pose.pose.orientation.w = 1.0
        self.scene.add_box(self.put, put_pose, self.put_size)

        # 将桌子设置成红色，两个box设置成橙色
        self.setColor(self.wall1, 0.8, 0.3, 0, 0.6)
        self.setColor(self.wall2, 0.8, 0.4, 0, 0.6)
        self.setColor(self.cabinet, 0.8, 0.4, 0, 0.6)
        self.sendColors()
    

    def wait_for_key_press(self):
        key_pressed = {'value': None}

        def on_press(key):
            if key == Key.backspace:
                print("Backspace pressed, returning 0.")
                key_pressed['value'] = 0
                return False  # Stop the listener
            elif key == Key.enter:
                print("Enter pressed, returning 1.")
                key_pressed['value'] = 1
                return False  # Stop the listener

        with Listener(on_press=on_press) as listener:
            print("please press 'enter' to ensure or press 'backspace'" )
            listener.join()
        
        return key_pressed['value']


    def setColor(self, name, r, g, b, a=0.9):
        # 初始化moveit颜色对象
        color = ObjectColor()

        # 设置颜色值
        color.id = name
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a

        # 更新颜色字典
        self.colors[name] = color

    def sendColors(self):
        # 初始化规划场景对象
        p = PlanningScene()

        # 需要设置规划场景是否有差异
        p.is_diff = True

        # 从颜色字典中取出颜色设置
        for color in self.colors.values():
            p.object_colors.append(color)

        # 发布场景物体颜色设置
        self.scene_pub.publish(p)

    def joint_position_planning(self, position):
        current_joint_values = self.left_arm.get_current_joint_values()
        target_joint_values = current_joint_values  # 复制当前的关节状态
        target_joint_values[0] = position[0] # 设置第一个关节的目标角度，单位：弧度
        target_joint_values[1] = position[1]  
        target_joint_values[2] = position[2]
        target_joint_values[3] = position[3]
        target_joint_values[4] = position[4]
        target_joint_values[5] = position[5]
        self.left_arm.set_joint_value_target(target_joint_values)
        self.left_arm.set_max_velocity_scaling_factor(0.1)  # 设置最大速度为10%
        self.left_arm.set_max_acceleration_scaling_factor(0.1)  # 设置最大加速度为10%


        # 规划出轨迹
        planning_result = self.left_arm.plan()
        decition_result = self.wait_for_key_press()
        for i in range(10):
            if decition_result == 0:
                planning_result = self.left_arm.plan()
                print(planning_result)
                decition_result = self.wait_for_key_press()
            else:
                break
        # 执行规划出的轨迹
        success = planning_result[0]
        left_arm_plan = planning_result[1] if success else None
        # 若规划成功且轨迹存在，则执行规划出的轨迹
        if left_arm_plan and success:
            suc = self.left_arm.execute(left_arm_plan, wait=True)
            if suc:
                print('succeed excuted')
    
    def target_pose_planning(self, position, quat):
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.left_reference_frame
        target_pose.pose.position.x = position[0]
        target_pose.pose.position.y = position[1]
        target_pose.pose.position.z = position[2]
        target_pose.pose.orientation.x = quat[1]
        target_pose.pose.orientation.y = quat[2]
        target_pose.pose.orientation.z = quat[3]
        target_pose.pose.orientation.w = quat[0]
        self.left_arm.set_pose_target(target_pose, self.left_end_effector_link)
        self.left_arm.set_max_velocity_scaling_factor(0.1)  # 设置最大速度为10%
        self.left_arm.set_max_acceleration_scaling_factor(0.1)  # 设置最大加速度为10%
        self.left_arm.set_planning_time(5)
        # 规划出轨迹
        planning_result = self.left_arm.plan()
        decition_result = self.wait_for_key_press()
        for i in range(20):
            if decition_result == 0:
                print('replanned!')
                planning_result = self.left_arm.plan()
                print(planning_result)
                decition_result = self.wait_for_key_press()
            else:
                break
       # 规划出轨迹
        success = planning_result[0]
        left_arm_plan = planning_result[1] if success else None
        # 若规划成功且轨迹存在，则执行规划出的轨迹
        if left_arm_plan and success:
            self.left_arm.execute(left_arm_plan, wait=True)
    
    def try_planning(self, position, quat):
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.left_reference_frame
        target_pose.pose.position.x = position[0]
        target_pose.pose.position.y = position[1]
        target_pose.pose.position.z = position[2]
        target_pose.pose.orientation.x = quat[1]
        target_pose.pose.orientation.y = quat[2]
        target_pose.pose.orientation.z = quat[3]
        target_pose.pose.orientation.w = quat[0]
        self.left_arm.set_pose_target(target_pose, self.left_end_effector_link)
        self.left_arm.set_max_velocity_scaling_factor(0.1)  # 设置最大速度为10%
        self.left_arm.set_max_acceleration_scaling_factor(0.1)  # 设置最大加速度为10%
        self.left_arm.set_planning_time(1)
        # 规划出轨迹
        planning_result = self.left_arm.plan()
        return planning_result[0]
        
        
    
    def exit(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


def wait_for_choice():
    key_pressed = {'value': None}

    def on_press(key):
        try:
            if key.char in ['0', '1', '2', '3', '4']:
                print(f"Key {key.char} pressed, returning {key.char}.")
                key_pressed['value'] = int(key.char)
                return False  # Stop the listener
        except AttributeError:
            pass  # Ignore non-character keys

    with Listener(on_press=on_press) as listener:
        print("please press '0-4' to select a choice" )
        listener.join()
    
    return key_pressed['value']

def planning_measurement_point():

    target_position = [187.755 * np.pi / 180, 100.911 * np.pi / 180, -88.554 * np.pi / 180, 
                       75.26 * np.pi / 180, 78.773 * np.pi / 180, 143.051 * np.pi / 180]
    left_arm.joint_position_planning(target_position)

    rz = -44.49 * np.pi / 180
    ry = 7.497 * np.pi / 180
    rx = -171.289 * np.pi / 180
    x = -371.060
    y = -146.762
    z = 511.249

    rospy.set_param("x",  float(x))
    rospy.set_param("y",  float(y))
    rospy.set_param("z",  float(z))
    rospy.set_param("rx",  float(rx))
    rospy.set_param("ry",  float(ry))
    rospy.set_param("rz",  float(rz))
    req.num = 7
    resp = client.call(req) #精确执行

def planning_grasping(pos, rotm):
    """
    pos 和 rotm 都是夹爪中心（eef) 的期望位姿,分别是numpy 3*1 向量和3*3矩阵
    """
    print(pos)
    print(rotm)
    pos = pos / 1000
    # 加偏执后， 直线运动
    delta_p = np.array([0, 0, -0.05])
    grasping_pos = pos + rotm @ delta_p
    grasping_pos = grasping_pos.astype(float)
    grasping_rotm = rotm.astype(float)
    grasping_quat = trans_quat.mat2quat(grasping_rotm)
    print(grasping_quat)
    left_arm.target_pose_planning(grasping_pos, grasping_quat)

    # 最后使用直线运动，保证精度，真实机器人使用，仿真器不用
    left_eef_offset = np.array([0, 0, 0.183 + 0.0495])
    quat_gripper_body = trans_quat.quat2mat([0.923879, 0, 0, -0.382684])
    quat_grasping_frame = trans_quat.quat2mat([0.7071, 0, 0, 0.7071])
    left_eef_offset_rotm = quat_gripper_body @ quat_grasping_frame


    left_link6_pos = pos - rotm @ (left_eef_offset_rotm.T @ left_eef_offset)
    left_link6_rotm = rotm @ left_eef_offset_rotm.T

    rpy = from_matrix_to_eular(left_link6_rotm)
    print('left_link6')
  
    left_link6_pos = left_link6_pos * 1000
    print(left_link6_pos)
    print(left_link6_rotm)
    print(rpy)
    ABS = 0
    INCR= 1
    tcp_pos=[left_link6_pos[0],left_link6_pos[1],left_link6_pos[2],rpy[0],rpy[1],rpy[2]]
    print(tcp_pos)
    rospy.set_param("x",  float(left_link6_pos[0]))
    rospy.set_param("y",  float(left_link6_pos[1]))
    rospy.set_param("z",  float(left_link6_pos[2]))
    rospy.set_param("rx",  float(rpy[0]))
    rospy.set_param("ry",  float(rpy[1]))
    rospy.set_param("rz",  float(rpy[2]))
    req.num = 7
    resp = client.call(req) #执行交接动作
    # robot.login() #登录
    # robot.linear_move(tcp_pos,ABS,True,5)
    # robot.logout() #每次运行完都登出

def try_planning_grasping(pos, rotm):
    """
    pos 和 rotm 都是夹爪中心（eef) 的期望位姿,分别是numpy 3*1 向量和3*3矩阵
    """
    print(pos)
    print(rotm)
    pos = pos / 1000
    # 加偏执后， 直线运动
    delta_p = np.array([0, 0, -0.05])
    grasping_pos = pos + rotm @ delta_p
    grasping_pos = grasping_pos.astype(float)
    grasping_rotm = rotm.astype(float)
    grasping_quat = trans_quat.mat2quat(grasping_rotm)
    print(grasping_quat)
    result = left_arm.try_planning(grasping_pos, grasping_quat)
    return result


def planning_grasping_move_up(pos, rotm):
    pos = pos / 1000
    delta_p = np.array([0, 0, -0.1])
    move_up_pos = pos + rotm @ delta_p
    move_up_pos = move_up_pos.astype(float)
    print(move_up_pos)
    move_up_rotm = rotm.astype(float)
    move_up_rotm = rotm
    move_up_quat = trans_quat.mat2quat(move_up_rotm)
    left_arm.target_pose_planning(move_up_pos, move_up_quat)


def planning_to_home():
    target_position = [np.pi / 3, np.pi / 2, np.pi * 3 / 4, np.pi * 3 /4, np.pi / 2, (135 * np.pi) / 180]

    left_arm.joint_position_planning(target_position)

def try_planning_handover(T_relatives):

    # 候补点在右侧的位置和姿态
    eef_pos_in_right = np.array([-0.07189, 0.3575, 0.2679])
    eef_mat_in_right = np.array([[  -0.5, -0.866,  0],
                                      [ 0.866,   -0.5,  0],
                                      [     0,      0,  1]])
    
    for i in range(len(T_relatives)):
    # 定义左侧更换点的变换姿态
        T_relative = T_relatives[i]
        eef_wait_mat_in_right = eef_mat_in_right @ T_relative[:3, :3]
        eef_wait_pos_in_right = eef_pos_in_right + eef_mat_in_right @ (T_relative[:3, 3]/1000)

        # 变换到左侧机器人坐标系下
        eef_wait_pos_in_right = eef_wait_pos_in_right.tolist()
        eef_pos_wait_in_left = [float(-25.0 /1000 + eef_wait_pos_in_right[0]), float(-1100.0 /1000 + eef_wait_pos_in_right[1]),
                                float(0/1000 + eef_wait_pos_in_right[2])]
        eef_mat_wait_in_left = eef_wait_mat_in_right
        eef_quat_wait_in_left = trans_quat.mat2quat(eef_mat_wait_in_left)

        
        res = left_arm.try_planning(position=eef_pos_wait_in_left, quat=eef_quat_wait_in_left)
        if res:
            return i

        if i == len(T_relatives) - 1 and res != 0:
            return -1


def planning_handover(T_relative):

    # 旋拧点在右侧的位置和姿态
    delta_p = np.array([0.10, 0, 0])
    eef_pos_in_right = np.array([-0.07189, 0.3575, 0.2679])
    eef_mat_in_right = np.array([[  -0.5, -0.866,  0],
                                [ 0.866,   -0.5,  0],
                                [     0,      0,  1]])
    # 等待点的位置和姿态
    eef_pos_wait_in_right = eef_pos_in_right + eef_mat_in_right @ delta_p
    eef_mat_wait_in_right = eef_mat_in_right

    # 抓取点在右侧的姿态
    gra_wait_mat_in_right = eef_mat_wait_in_right @ T_relative[:3, :3]
    gra_wait_pos_in_right = eef_pos_wait_in_right + eef_mat_wait_in_right @ (T_relative[:3, 3] / 1000)

    # 变换到左侧机器人坐标系下
    gra_wait_pos_in_right = gra_wait_pos_in_right.tolist()

    eef_pos_wait_in_left = [float(-30.0 /1000 + gra_wait_pos_in_right[0]), float(-1101.2 /1000 + gra_wait_pos_in_right[1]),
                            float(1.2/1000 + gra_wait_pos_in_right[2])]
    eef_mat_wait_in_left = gra_wait_mat_in_right
    eef_quat_wait_in_left = trans_quat.mat2quat(eef_mat_wait_in_left)

    res = left_arm.target_pose_planning(position=eef_pos_wait_in_left, quat=eef_quat_wait_in_left)

    # 最后使用直线运动，保证精度，真实机器人使用，仿真器不用

    ## 按照真正的旋拧点，再变换一次
    gra_mat_in_right = eef_mat_in_right @ T_relative[:3, :3]
    gra_pos_in_right = eef_pos_in_right + eef_mat_in_right @ (T_relative[:3, 3] / 1000)

    # 变换到左侧机器人坐标系下
    gra_pos_in_right = gra_pos_in_right.tolist()
    # eef_pos_in_left = [float(-28.0 /1000 + gra_pos_in_right[0]), float(-1101.2 /1000 + gra_pos_in_right[1]),
    #                         float(1.2/1000 + gra_pos_in_right[2])]
    eef_pos_in_left = [float(-28.0 /1000 + gra_pos_in_right[0]), float(-1101.2 /1000 + gra_pos_in_right[1]),
                        float(1.2/1000 + gra_pos_in_right[2])]
    print ("eef_pos_in_left= ", eef_pos_in_left)

    eef_mat_in_left = gra_mat_in_right

    left_eef_offset = np.array([0, 0, 0.183 + 0.0495])
    quat_gripper_body = trans_quat.quat2mat([0.923879, 0, 0, -0.382684])
    quat_grasping_frame = trans_quat.quat2mat([0.7071, 0, 0, 0.7071])
    left_eef_offset_rotm = quat_gripper_body @ quat_grasping_frame


    left_link6_pos = eef_pos_in_left - eef_mat_in_left @ (left_eef_offset_rotm.T @ left_eef_offset)
    left_link6_rotm = eef_mat_in_left @ left_eef_offset_rotm.T

    rpy = from_matrix_to_eular(left_link6_rotm)
    print('left_link6')
  
    left_link6_pos = left_link6_pos * 1000
    print(left_link6_pos)
    print(left_link6_rotm)
    print(rpy)
    ABS = 0
    INCR= 1
    tcp_pos=[left_link6_pos[0],left_link6_pos[1],left_link6_pos[2],rpy[0],rpy[1],rpy[2]]

    rospy.set_param("x",  float(left_link6_pos[0]))
    rospy.set_param("y",  float(left_link6_pos[1]))
    rospy.set_param("z",  float(left_link6_pos[2]))
    rospy.set_param("rx",  float(rpy[0]))
    rospy.set_param("ry",  float(rpy[1]))
    rospy.set_param("rz",  float(rpy[2]))
    req.num = 7
    resp = client.call(req) #执行交接动作
    # robot.login() #登录
    # robot.linear_move(tcp_pos,ABS,True,2)
    # robot.logout() #每次运行完都登出



def planning_to_right_side():
    eef_pos_in_right = np.array([-0.07189, 0.3575, 0.2679])
    eef_mat_in_right = np.array([[  -0.5, -0.866,  0],
                                 [ 0.866,   -0.5,  0],
                                 [     0,      0,  1]])
    # 候补点在右侧的位置和姿态
    delta_p = np.array([0.07, 0, 0])
    eef_wait_pos_in_right = eef_pos_in_right + eef_mat_in_right @ delta_p
    eef_wait_mat_in_right = np.array([[  -0.5, -0.866,  0],
                                      [ 0.866,   -0.5,  0],
                                      [     0,      0,  1]])
    


    # 定义右边更换点的变换姿态
    rotation_y = R.from_euler('y', -np.pi/2, degrees=False)
    R_right_side_temp = rotation_y
    R_right_side = R_right_side_temp.as_matrix()
    eef_wait_mat_in_right = eef_wait_mat_in_right @ R_right_side


    # 变换到左侧机器人坐标系下
    eef_wait_pos_in_right = eef_wait_pos_in_right.tolist()
    eef_pos_wait_in_left = [float(-28.0 /1000 + eef_wait_pos_in_right[0]), float(-1101.0 /1000 + eef_wait_pos_in_right[1]),
                             float(1.0/1000 + eef_wait_pos_in_right[2])]
    eef_mat_wait_in_left = eef_wait_mat_in_right
    eef_quat_wait_in_left = trans_quat.mat2quat(eef_mat_wait_in_left)

    left_arm.target_pose_planning(position=eef_pos_wait_in_left, quat=eef_quat_wait_in_left)

if __name__ == '__main__':
    # gripper_obj = RobotiqGripper()
    # gripper_obj.activate()
    # gripper_obj.goTo(0)

    gripper_pub = rospy.Publisher("gripper_siginal", gripper, queue_size=10)


    client = rospy.ServiceProxy("leftrobotservice",leftrobotsrv)
    client.wait_for_service()
    req = leftrobotsrvRequest() #机器人控制

    robot = jkrc.RC("192.168.3.200")#返回一个机器人对象 
    left_arm = Grasp_planning()
    gri = gripper() #夹爪控制
    for i in [0, 1]:
        gri.open = 2.0 #255
        gripper_pub.publish(gri)
    # 规划到测量点
    planning_measurement_point()

    time.sleep(1)
    # 从数据中获取点云
    # src, tar, src_down_o3, tar_down_o3, desk, gripper = rans.get_pointcloud_from_data()

    # 从相机中获取点云
    # file_name = "circle_bolt.pcd"
    # file_name = "three_3score.pcd"
    file_name = "bolt_3score.pcd"
    src, tar, src_down_o3, tar_down_o3, desk, gripper = rans.get_pointcloud_from_camera(file_name)

    aabb, vertices = rans.get_boundingbox(src)
    num_screw = 2 #随着物体变化，根据物体的对称形确定，圆形可设任意，六边形6个，对称2个
    s_poss, s_rpys, T_s = ge.screwing_poses(num_screw)

    result = rans.get_object_pose(src_down_o3, tar_down_o3)
    num_grasp = 26
    g_poss, g_rpys_trans, T_g, T_g_trans = rans.get_transformed_grasp_poses(num_grasp, vertices, result, gripper, desk)
    print(T_g_trans)
    keys_array = list(T_g_trans.keys())

    for item in range(20): #总体尝试次数
        #随机采样选择一个抓取点
        keys_array = list(T_g_trans.keys())
        index_trans = np.random.choice(keys_array)
        selected_T_g = T_g[index_trans]
        T_relatives = rans.calculate_relative_matrixs(T_g[index_trans], T_s) 
        print(T_g_trans[index_trans])

        index = try_planning_handover(T_relatives)
        print('index = ', index)
        gra_result = try_planning_grasping(T_g_trans[index_trans][:3, 3], T_g_trans[index_trans][:3, :3])
        if (index is not None) and gra_result:
            print("successfully planned!")
            # 规划抓取动作
            print('planning grasping')
            print(T_g_trans[index_trans])
            # try_planning_grasping(T_g_trans[index_trans][:3, 3], T_g_trans[index_trans][:3, :3])
            # decition_result = wait_for_key_press() #不满意，就重来
            # if decition_result == 0:
            #     continue

            planning_grasping(T_g_trans[index_trans][:3, 3], T_g_trans[index_trans][:3, :3])

            # 关闭夹爪
            print("please press 'enter' to grasp the object")
            decition_result = wait_for_key_press()
            if decition_result:
                for i in [0, 1]:
                    gri.open = 0.0

                    gripper_pub.publish(gri)
                    # gripper_obj.goTo(255)

            # 规划抬起动作
            print('planning move up')
            planning_grasping_move_up(T_g_trans[index_trans][:3, 3], T_g_trans[index_trans][:3, :3])

            print('please press 1-3 select the goal object')
            choice = wait_for_choice()
            print('choice = ', choice)

            if choice == 1:
                rospy.set_param("goal_width", -46762) #3分螺母
            
            elif choice == 2:
                rospy.set_param("goal_width", -46762) #3分3通
            
            elif choice == 3:
                rospy.set_param("goal_width", -46762) #m12螺母

            # 规划交接动作
            print('planning hand over')
            planning_handover(T_relatives[index])
            req.num = 6
            resp = client.call(req) #开启handover 任务
            break
        elif index == -1:
            continue

        # 检查是否成功

        # if 成功 -> 打开夹爪

        # if 失败 -> 规划回去





    








