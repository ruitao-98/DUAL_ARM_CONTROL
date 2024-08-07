#!/home/yanji/anaconda3/envs/mujo/bin/python
# -*- coding: utf-8 -*-

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



class MoveitServer:
    def __init__(self, model, data,  RbtMj):
        self.model = model
        self.data = data
        self.RbtMj = RbtMj
        self.server = actionlib.SimpleActionServer('/arm/follow_joint_trajectory', FollowJointTrajectoryAction, self.cb, False)
        self.server.start()
        self.joint_values_queue = deque()

        self.joint_state_pub = rospy.Publisher("/joint_states", JointState, 10)
        rospy.loginfo('Server started')

        sim = vi.Mujoco_Viewer(self.model, self.data, 500, self.controller, self.RbtMj)
        sim.viewer()

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.joint_state_callback()
            rate.sleep()

    def cb(self, goal):

        rospy.loginfo('server deals with: ' + str(goal))
        Joint_positions = goal.trajectory.points #所有的坐标点

        self.joint_values_queue.clear()

        # left_joint_value = np.arange(0, 6)
        # right_joint_value = np.arange(0, 6) #用于填充的空值

        for i in range(0, len(Joint_positions)):  #循环执行
            left_joint_value = Joint_positions.positions[0:6]  #赋值
            right_joint_value = Joint_positions.positions[6:12]
            self.joint_values_queue.append({"left": left_joint_value,
                                            "right": right_joint_value}) #加入队列

        while True:
            # if
            if self.server.is_preempt_requested():
                rospy.loginfo('server preempted')
                break
        self.server.set_succeeded(self)
        rospy.Duration(0.5)


    def controller(self, model, data):
        # put the controller here. This function is called inside the simulation.
        if not self.joint_values_queue:
            rospy.logwarn("No joint angles received yet.")
            return
        # Joint = 1 + data.time * 0.1
        # initial_qpos = {
        #     'l_j1': 0,
        #     'l_j2': -0,
        #     'l_j3': Joint,
        #     'l_j4': 0,
        #     'l_j5': 1.6,
        #     'l_j6': 0,
        # }
        next_value = self.joint_values_queue.popleft()
        left_joint_value = next_value['left']
        right_joint_value = next_value['right']


        # Joint3_ref.append(Joint)

        id_left_base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j1')
        left_index = np.arange(id_left_base, id_left_base + 6)
        id_right_base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'r_j1')
        right_index = np.arange(id_right_base, id_right_base + 6)

        for left_index_item in left_index:
            data.qpos[left_index_item] = left_joint_value[left_index_item - left_index[0]]
        for right_index_item in right_index:
            data.qpos[right_index_item] = right_joint_value[right_index_item-right_index[0]]

        data.ctrl[left_index], data.ctrl[right_index] = RbtMj.coriolis_gravity()
        # pass

    def joint_state_callback(self):
        id_left_base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j1')
        left_index = np.arange(id_left_base, id_left_base + 6)
        id_right_base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'r_j1')
        right_index = np.arange(id_right_base, id_right_base + 6)
        joint_state_msg = JointState()


        for left_index_item in left_index:
            joint_state_msg.position.append(data.qpos[left_index_item])
            left_joint_name = 'l_j' + str(left_index_item - left_index[0])
            joint_state_msg.name.append(left_joint_name)
        for right_index_item in right_index:
            joint_state_msg.position.append(data.qpos[left_index_item])
            left_joint_name = 'r_j' + str(right_index_item - right_index[0])
            joint_state_msg.name.append(left_joint_name)
        joint_state_msg.header.stamp = rospy.Time.now()
        self.joint_state_pub.publish(joint_state_msg)

        # return joint_state_msg










if __name__ == '__main__':
    # os.environ['PYTHONPATH'] = '/home/yanji/anaconda3/envs/mujo/bin/python'
    right_path = "../../robot_description/xml/dual_arm.xml"
    model = mj.MjModel.from_xml_path(right_path)  # MuJoCo model
    data = mj.MjData(model)  # MuJoCo data
    RbtMj = func_mj.Robot(data, model, 'grasping_frame', 'screw_frame')

    rospy.init_node('moveit_server', anonymous=True)


    server = MoveitServer(model, data,  RbtMj)
    rospy.spin()

