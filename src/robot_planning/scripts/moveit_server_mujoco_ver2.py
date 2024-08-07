#!/home/yanji/anaconda3/envs/mujo/bin/python
# -*- coding: utf-8 -*-
import time

import mujoco as mj
import viewer as vi
import func_ki_dy as func_mj
import moveit_commander
import rospy
import numpy as np
import actionlib
from collections import deque
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectory
from std_msgs.msg import Empty
from std_srvs.srv import Empty
from std_srvs.srv import SetBool
from sensor_msgs.msg import JointState
import os
from threading import Thread, Lock
import queue



# def jointStates(left_joint_value):
#     global model, data
#     # lock.acquire()
#     id_left_base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j1')
#     left_index = np.arange(id_left_base, id_left_base + 6)
#     id_right_base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'r_j1')
#     right_index = np.arange(id_right_base, id_right_base + 6)
#     joint_state = True
#
#     for left_index_item in left_index:
#         ret = ((data.qpos[left_index_item] < left_joint_value[left_index_item - left_index[0]] + 0.2)
#                and (data.qpos[left_index_item] > left_joint_value[left_index_item - left_index[0]] - 0.2))
#         joint_state = joint_state and ret
#     # for right_index_item in right_index:
#     #     ret = ((data.qpos[right_index_item] < right_joint_value[right_index_item - left_index[0]] + 0.2)
#     #            and (data.qpos[right_index_item] > left_joint_value[right_index_item - left_index[0]] - 0.2))
#     #     joint_state = joint_state and ret
#     # lock.release()
#     return joint_state

def cb_arm(goal):
    global server, data
    global joint_values_queue
    print('cb_arm-------------*********************')
    lock.acquire()
    global which_arm
    global joint_values
    Joint_positions = goal.trajectory.points  # 所有的坐标点
    name = goal.trajectory.joint_names
    which_arm = name[0]
    print(name[0])
    print(goal)
    first_joint_index = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name[0])
    full_joint_index = np.arange(first_joint_index, first_joint_index + 6)
    joint_values = []
    # if joint_values_queue:
    #     joint_values_queue.clear()

    for item in Joint_positions:  # 循环执行
        joint_value = list(item.positions[0:6])  # 赋值
        joint_values.append(joint_value)
        rospy.sleep(0.05)
    lock.release()
    np.savetxt("joint_value.txt", np.array(joint_values))
    # while True:
    #     if jointStates(full_joint_index):
    #         print('served successfully')
    #         break
    #     if server.is_preempt_requested():
    #         rospy.loginfo('server preempted')
    #         print('server preempted')
    #         break

    server.set_succeeded()
    rospy.sleep(0.5)


def cb1(goal):
    # rospy.loginfo('server deals wsith: ' + str(goal))
    # Joint_positions = goal.trajectory.points  # 所有的坐标点
    # # print(Joint_positions)
    #
    # joint_values_queue.clear()
    #
    # # left_joint_value = np.arange(0, 6)
    # # right_joint_value = np.arange(0, 6) #用于填充的空值
    #
    # for i in range(0, len(Joint_positions)):  # 循环执行
    #     left_joint_value = Joint_positions.positions[0:6]  # 赋值
    #     right_joint_value = Joint_positions.positions[6:12]
    #     joint_values_queue.append({"left": left_joint_value,
    #                                     "right": right_joint_value})  # 加入队列
    #
    # while True:
    #     # if
    #     if server.is_preempt_requested():
    #         rospy.loginfo('server preempted')
    #         break
    # server.set_succeeded()
    # rospy.Duration(0.5)
    pass

def cb2(goal):
    # rospy.loginfo('server deals wsith: ' + str(goal))
    # Joint_positions = goal.trajectory.points  # 所有的坐标点
    # # print(Joint_positions)
    #
    # joint_values_queue.clear()
    #
    # # left_joint_value = np.arange(0, 6)
    # # right_joint_value = np.arange(0, 6) #用于填充的空值
    #
    # for i in range(0, len(Joint_positions)):  # 循环执行
    #     left_joint_value = Joint_positions.positions[0:6]  # 赋值
    #     right_joint_value = Joint_positions.positions[6:12]
    #     joint_values_queue.append({"left": left_joint_value,
    #                                     "right": right_joint_value})  # 加入队列
    #
    # while True:
    #     # if
    #     if server.is_preempt_requested():
    #         rospy.loginfo('server preempted')
    #         break
    # server.set_succeeded()
    # rospy.Duration(0.5)
    pass

def joint_state_callback():
    id_left_base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j1')
    left_index = np.arange(id_left_base, id_left_base + 6)
    id_right_base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'r_j1')
    right_index = np.arange(id_right_base, id_right_base + 6)
    joint_state_msg = JointState()


    for left_index_item in left_index:
        joint_state_msg.position.append(data.qpos[left_index_item])
        left_joint_name = 'l_j' + str(left_index_item - left_index[0] + 1)
        joint_state_msg.name.append(left_joint_name)
    for right_index_item in right_index:
        joint_state_msg.position.append(data.qpos[right_index_item])
        left_joint_name = 'r_j' + str(right_index_item - right_index[0] + 1)
        joint_state_msg.name.append(left_joint_name)
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_pub.publish(joint_state_msg)

def controller(model, data):
    # put the controller here. This function is called inside the simulation.
    global joint_values_queue
    global joint_values
    global which_arm
    global prev_time
    global index
    id_left_base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j1')
    left_index = np.arange(id_left_base, id_left_base + 6)
    id_right_base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'r_j1')
    right_index = np.arange(id_right_base, id_right_base + 6)
    data.ctrl[left_index], data.ctrl[right_index] = RbtMj.coriolis_gravity()
    # if not joint_values_queue:
    #     # rospy.logwarn("No joint angles received yet.")
    #     return
    # Joint = 1 + data.time * 0.1
    # initial_qpos = {
    #     'l_j1': 0,
    #     'l_j2': -0,
    #     'l_j3': Joint,
    #     'l_j4': 0,
    #     'l_j5': 1.6,
    #     'l_j6': 0,
    # }
    # next_value = joint_values_queue.popleft()
    # left_joint_value = next_value['left']
    # right_joint_value = next_value['right']


    lock.acquire()  # 在访问队列之前获取锁
    # if not joint_values:
    #     return
    # # Joint3_ref.append(Joint)
    if joint_values:


        # print(prev_time)
        # print(index)
        # print(len(joint_values))
        # print(joint_values)
        # print(which_arm)
        print('----------------------------')
        print(data.time)
        if prev_time is not None and data.time > prev_time and index < len(joint_values)-1:
            index = index + 1
            print(index)
            index_first = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, which_arm)
            print(index_first)
            _index = np.arange(index_first, index_first + 6)
            print(_index)
            print(joint_values[index])
            data.qpos[_index] = joint_values[index]

        elif index == len(joint_values) - 1:
            joint_values = []
            print(joint_values)
            index = 0
            which_arm = None
            # return

        prev_time = data.time

        # print(joint_values)
        # print(joint_values)

        # 提取名称和joint_value
        # 假设每个项的格式是 {name: joint_value}
        # for i in range(len(joint_values)):

        #
        #     rospy.sleep(0.05)  # 假设你想在循环中稍作延时

            # print(f"Name: {name}, Joint Value: {joint_value}")

    lock.release()  # 确保释放锁，无论上面的操作是否成功


    # id_left_base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j1')
    # left_index = np.arange(id_left_base, id_left_base + 6)
    # id_right_base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'r_j1')
    # right_index = np.arange(id_right_base, id_right_base + 6)
    # data.ctrl[left_index], data.ctrl[right_index] = RbtMj.coriolis_gravity()

    # for index_item in left_index:
    #     data.qposindex_item] = left_joint_value[left_index_item - left_index[0]]
    # for right_index_item in right_index:
    #     data.qpos[right_index_item] = right_joint_value[right_index_item - right_index[0]]


    # pass


def simulation_thread():
    global model, data
    sim = vi.Mujoco_Viewer(model, data, 500, controller, RbtMj) #实例化mujoco启动类
    sim.viewer()

# def communication_thread():
#     global model, data
#
#     joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
#     rospy.loginfo('Server started')
#
#     server = actionlib.SimpleActionServer('arm/follow_joint_trajectory', FollowJointTrajectoryAction, cb_arm, False)
#     server.start() #创建action
#
#     rate = rospy.Rate(100)
#     while not rospy.is_shutdown():
#         joint_state_callback() #发布topic
#         rate.sleep()

if __name__ == '__main__':
    default_joint = 0

    right_path = "../../robot_description/xml/dual_arm.xml"
    model = mj.MjModel.from_xml_path(right_path)  # MuJoCo model
    data = mj.MjData(model)  # MuJoCo data
    RbtMj = func_mj.Robot(data, model, 'grasping_frame', 'screw_frame') #计算相关参数
    joint_values_queue = deque()

    lock = Lock()
    Thread(target=simulation_thread).start()  #创建仿真程序
    time.sleep(0.1)

    joint_values = []
    which_arm = None
    prev_time = None
    index = 0

    rospy.init_node('moveit_server', anonymous=True)
    joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
    rospy.loginfo('Server started')

    server = actionlib.SimpleActionServer('arm/follow_joint_trajectory', FollowJointTrajectoryAction, cb_arm, False)
    server.start() #创建action
    server1 = actionlib.SimpleActionServer('screw_tool/follow_joint_trajectory', FollowJointTrajectoryAction, cb1, False)
    server1.start()
    server2 = actionlib.SimpleActionServer('gripper/follow_joint_trajectory', FollowJointTrajectoryAction, cb2, False)
    server2.start()


    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        joint_state_callback() #发布topic
        rate.sleep()


    # Thread(target=communication_thread).start() #创建通信程序









