#! /home/yanji/anaconda3/envs/mujo/bin/python3
# -*- coding: utf-8 -*-

import platform 
print("version", platform.python_version())
import rospy, sys
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
from real_robot_control.srv import *
from real_robot_control.msg import gripper





class Left_planning():
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        # 初始化ROS节点
        rospy.init_node('moveit_planning')
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
        self.left_arm.set_planning_time(30)
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
        wall2_pose.pose.position.x = self.wall2_size[0] / 2 + 0.9 + 0.3
        wall2_pose.pose.position.y = self.wall2_size[1] / 2 + 0.8
        wall2_pose.pose.position.z = self.wall2_size[2] / 2
        wall2_pose.pose.orientation.w = 1.0
        self.scene.add_box(self.wall2, wall2_pose, self.wall2_size)

        self.scene.remove_world_object(self.cabinet)
        cabinet_pose = PoseStamped()
        cabinet_pose.header.frame_id = self.world_reference_frame
        cabinet_pose.pose.position.x = self.cabinet_size[0] / 2
        cabinet_pose.pose.position.y = self.cabinet_size[1] / 2 + 1.5 + 0.15
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
            print("please press 'enter' to ensure the trajectory or press 'backspace' to replan" )
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
        # left_arm.set_max_velocity_scaling_factor(0.5)
        self.left_arm.set_max_velocity_scaling_factor(0.1)  # 设置最大速度为10%
        self.left_arm.set_max_acceleration_scaling_factor(0.1)  # 设置最大加速度为10%
        # self.left_arm.plan()
        # self.left_arm.go(wait=1)

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
       # 规划出轨迹
        success = planning_result[0]
        left_arm_plan = planning_result[1] if success else None

        # 若规划成功且轨迹存在，则执行规划出的轨迹
        if left_arm_plan and success:
            self.left_arm.execute(left_arm_plan, wait=True)
    
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
       # 规划出轨迹
        success = planning_result[0]
        left_arm_plan = planning_result[1] if success else None

        # 若规划成功且轨迹存在，则执行规划出的轨迹
        if left_arm_plan and success:
            self.left_arm.execute(left_arm_plan, wait=True)
        
    
    def exit(self):
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


def wait_for_choice():
    key_pressed = {'value': None}

    def on_press(key):
        
        try:
            # print(f"Key pressed: {key}, key.char: {getattr(key, 'char', None)}")
            if key.char in ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']:
                print(f"Key {key.char} pressed, returning {key.char}.")
                key_pressed['value'] = int(key.char)
                return False  # Stop the listener
            else:
                print('no in it')
            # if key.char in ['0', '1', '2', '3', '4', '5', '6', '7', '8', '9']:
            #     print(f"Key {key.char} pressed, returning {key.char}.")
            #     key_pressed['value'] = int(key.char)
            #     return False  # Stop the listener
            # else:
            #     print('no in it')
        except AttributeError:
            pass  # Ignore non-character keys

    with Listener(on_press=on_press) as listener:
        print("please press the keyboard to select a choice" )
        listener.join()
    
    return key_pressed['value']

def planning_to_put(choice):
    if choice == 0:
        pos_put_in_left = [-90/1000, (-300 + 28 *2)/1000, 76/1000]
        mat_put_in_left = np.array([[0, 0, 1],
                                    [ -1 ,0 , 0],
                                    [ 0 ,-1 , 0]])
        quat_put_in_left = trans_quat.mat2quat(mat_put_in_left)
    
    elif choice == 1:
        pos_put_in_left = [-90/1000, (-300 + 28 *1)/1000, 76/1000]
        mat_put_in_left = np.array([[0, 0, 1],
                                    [ -1 ,0 , 0],
                                    [ 0 ,-1 , 0]])
        quat_put_in_left = trans_quat.mat2quat(mat_put_in_left)
    
    elif choice == 2:
        pos_put_in_left = [-90/1000, (-300)/1000, 76/1000]
        mat_put_in_left = np.array([[0, 0, 1],
                                    [ -1 ,0 , 0],
                                    [ 0 ,-1 , 0]])
        quat_put_in_left = trans_quat.mat2quat(mat_put_in_left)
    
    elif choice == 3:
        pos_put_in_left = [-90/1000, (-300 - 28 *1)/1000, 76/1000]
        mat_put_in_left = np.array([[0, 0, 1],
                                    [ -1 ,0 , 0],
                                    [ 0 ,-1 , 0]])
        quat_put_in_left = trans_quat.mat2quat(mat_put_in_left)
    
    elif choice == 4:
        pos_put_in_left = [-90/1000, (-300 - 28 *2)/1000, 76/1000]
        mat_put_in_left = np.array([[0, 0, 1],
                                    [ -1 ,0 , 0],
                                    [ 0 ,-1 , 0]])
        quat_put_in_left = trans_quat.mat2quat(mat_put_in_left)

    
    left_arm.target_pose_planning(position=pos_put_in_left, quat=quat_put_in_left)


def planning_to_recycle(chioce):
    int_value =rospy.get_param("left_right_dis",4)
    if int_value == 3:
        if choice == 0:
            pos_put_in_left = [-90/1000, (-300 + 28 *2)/1000, 76/1000]
            mat_put_in_left = np.array([[0, 0, 1],
                                        [ -1 ,0 , 0],
                                        [ 0 ,-1 , 0]])
            quat_put_in_left = trans_quat.mat2quat(mat_put_in_left)
        
        elif choice == 1:
            pos_put_in_left = [-90/1000, (-300 + 28 *1)/1000, 76/1000]
            mat_put_in_left = np.array([[0, 0, 1],
                                        [ -1 ,0 , 0],
                                        [ 0 ,-1 , 0]])
            quat_put_in_left = trans_quat.mat2quat(mat_put_in_left)
        
        elif choice == 2:
            pos_put_in_left = [-90/1000, (-300)/1000, 76/1000]
            mat_put_in_left = np.array([[0, 0, 1],
                                        [ -1 ,0 , 0],
                                        [ 0 ,-1 , 0]])
            quat_put_in_left = trans_quat.mat2quat(mat_put_in_left)
        
        elif choice == 3:
            pos_put_in_left = [-90/1000, (-300 - 28 *1)/1000, 76/1000]
            mat_put_in_left = np.array([[0, 0, 1],
                                        [ -1 ,0 , 0],
                                        [ 0 ,-1 , 0]])
            quat_put_in_left = trans_quat.mat2quat(mat_put_in_left)
        
        elif choice == 4:
            pos_put_in_left = [-90/1000, (-300 - 28 *2)/1000, 76/1000]
            mat_put_in_left = np.array([[0, 0, 1],
                                        [ -1 ,0 , 0],
                                        [ 0 ,-1 , 0]])
            quat_put_in_left = trans_quat.mat2quat(mat_put_in_left)
    if int_value == 4:
        if choice == 0:
            pos_put_in_left = [-90/1000, (-300 + 28 *2)/1000, 76/1000]
            mat_put_in_left = np.array([[0, 0, 1],
                                        [ -1 ,0 , 0],
                                        [ 0 ,-1 , 0]])
            quat_put_in_left = trans_quat.mat2quat(mat_put_in_left)
        
        elif choice == 1:
            pos_put_in_left = [-90/1000, (-300 + 28 *1)/1000, 76/1000]
            mat_put_in_left = np.array([[0, 0, 1],
                                        [ -1 ,0 , 0],
                                        [ 0 ,-1 , 0]])
            quat_put_in_left = trans_quat.mat2quat(mat_put_in_left)
        
        elif choice == 2:
            pos_put_in_left = [-90/1000, (-300)/1000, 76/1000]
            mat_put_in_left = np.array([[0, 0, 1],
                                        [ -1 ,0 , 0],
                                        [ 0 ,-1 , 0]])
            quat_put_in_left = trans_quat.mat2quat(mat_put_in_left)
        
        elif choice == 3:
            pos_put_in_left = [-90/1000, (-300 - 28 *1)/1000, 76/1000]
            mat_put_in_left = np.array([[0, 0, 1],
                                        [ -1 ,0 , 0],
                                        [ 0 ,-1 , 0]])
            quat_put_in_left = trans_quat.mat2quat(mat_put_in_left)
        
        elif choice == 4:
            pos_put_in_left = [-90/1000, (-300 - 28 *2)/1000, 76/1000]
            mat_put_in_left = np.array([[0, 0, 1],
                                        [ -1 ,0 , 0],
                                        [ 0 ,-1 , 0]])
            quat_put_in_left = trans_quat.mat2quat(mat_put_in_left)

    
    left_arm.target_pose_planning(position=pos_put_in_left, quat=quat_put_in_left)

def planning_to_measurement():  
    # 6.12 待填写

    pos_put_in_left = [(-353.306)/1000, (-122.213)/1000, 249.919/1000]
    rotation_z = R.from_euler('z', np.pi, degrees=False)
    rotation_y = R.from_euler('y', 0, degrees=False)
    rotation_x = R.from_euler('x', -np.pi/2, degrees=False)

    # 将旋转矩阵组合起来
    mat_for_measurement = rotation_z * rotation_y * rotation_x
    # mat_put_in_left = np.array([[0, 0, 1],
    #                             [ -1 ,0 , 0],
    #                             [ 0 ,-1 , 0]])
    quat_put_in_left = trans_quat.mat2quat(mat_for_measurement)
    
    left_arm.target_pose_planning(position=pos_put_in_left, quat=quat_put_in_left)

def planning_to_home():
    target_position = [np.pi / 3, np.pi / 2, np.pi * 3 / 4, np.pi * 3 /4, np.pi / 2, (135 * np.pi) / 180]
    pos_put_in_left = [-90/1000, -300/1000, 80/1000]
    mat_put_in_left = np.array([[0, 0, 1],
                                [ -1 ,0 , 0],
                                [ 0 ,-1 , 0]])
    quat_put_in_left = trans_quat.mat2quat(mat_put_in_left)

    left_arm.joint_position_planning(target_position)

def planning_to_left_side():
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
    
    # 定义左侧更换点的变换姿态
    rotation_z = R.from_euler('z', np.pi, degrees=False)
    rotation_y = R.from_euler('y', -np.pi/2, degrees=False)
    # rotation_x = R.from_euler('x', -np.pi/2, degrees=False)
    R_left_side_temp = rotation_y * rotation_z 
    R_left_side = R_left_side_temp.as_matrix()
    eef_wait_mat_in_right = eef_wait_mat_in_right @ R_left_side
    
    # 定义右边更换点的变换姿态
    # rotation_y = R.from_euler('y', -np.pi/2, degrees=False)
    # # rotation_x = R.from_euler('x', -np.pi/2, degrees=False)
    # R_right_side_temp = rotation_y
    # R_right_side = R_right_side_temp.as_matrix()
    # eef_wait_mat_in_right = eef_wait_mat_in_right @ R_right_side


    # 变换到左侧机器人坐标系下
    eef_wait_pos_in_right = eef_wait_pos_in_right.tolist()
    print(-28.0 /1000 + eef_wait_pos_in_right[0])
    eef_pos_wait_in_left = [float(-28.0 /1000 + eef_wait_pos_in_right[0]), float(-1101.0 /1000 + eef_wait_pos_in_right[1]),
                             float(1.0/1000 + eef_wait_pos_in_right[2])]
    eef_mat_wait_in_left = eef_wait_mat_in_right
    eef_quat_wait_in_left = trans_quat.mat2quat(eef_mat_wait_in_left)

    left_arm.target_pose_planning(position=eef_pos_wait_in_left, quat=eef_quat_wait_in_left)


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
    print(-28.0 /1000 + eef_wait_pos_in_right[0])
    eef_pos_wait_in_left = [float(-28.0 /1000 + eef_wait_pos_in_right[0]), float(-1101.0 /1000 + eef_wait_pos_in_right[1]),
                             float(1.0/1000 + eef_wait_pos_in_right[2])]
    eef_mat_wait_in_left = eef_wait_mat_in_right
    eef_quat_wait_in_left = trans_quat.mat2quat(eef_mat_wait_in_left)

    left_arm.target_pose_planning(position=eef_pos_wait_in_left, quat=eef_quat_wait_in_left)

if __name__ == '__main__':
    left_arm = Left_planning()
    gripper_pub = rospy.Publisher("gripper_siginal", gripper, queue_size=10)
    gri = gripper() # 夹爪控制
    for i in [0, 1]:
        gri.open = 1.0  #160
        gripper_pub.publish(gri)

    client = rospy.ServiceProxy("leftrobotservice",leftrobotsrv)
    client.wait_for_service()
    req = leftrobotsrvRequest() #机器人控制

    
    # planning_to_home()
    while (True):
        print("===================")
        print("please make a choice to select a goal to planning\n")
        print("0--> planning to put || 1--> planning to left side assembly || 2 --> planning to right side assembly \n")
        print("3--> planning to left side disassembly || 4 --> planning to right side disassembly \n")
        print("5--> planning to recycle || 8--> break || 9--> re-choose\n")
        print("===================") 
        choice = wait_for_choice()
        print('choice = ', choice)

        if choice == 0:
            print("please select a put choice:")
            choice = wait_for_choice()
            rospy.set_param("choice_int",choice) #设置为参数，供执行端读取
            planning_to_put(choice)

            req.num = 0
            resp = client.call(req)
        
        elif choice == 1:
            planning_to_left_side()
            req.num = 1
            resp = client.call(req)

        elif choice == 2:
            planning_to_right_side()
            req.num = 2
            resp = client.call(req)

        elif choice == 3:
            planning_to_left_side()
            rospy.set_param("left_right_dis",3) 
            req.num = 3
            resp = client.call(req)

        elif choice == 4:
            planning_to_right_side()
            rospy.set_param("left_right_dis",4) #设置为参数，供执行端读取
            req.num = 4
            resp = client.call(req)
        
        elif choice == 5:
            print("planning_to_recycle")
            print("please select a put choice:")
            choice = wait_for_choice()
            # rospy.set_param("choice_int",choice) #设置为参数，供执行端读取
            planning_to_recycle(choice)
            time.sleep(1)
            for i in [0, 1]:
                gri.open = 1.0
                gripper_pub.publish(gri)

        elif choice == 8:
            break

        elif choice == 9:
            continue
        # time.sleep(2)
            
        

        # time.sleep(2)
        # 



    








