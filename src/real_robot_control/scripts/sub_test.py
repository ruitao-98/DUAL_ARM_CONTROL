#!/home/yanji/anaconda3/envs/mujo/bin/python3
# -*- coding: utf-8 -*-

import rospy
from real_robot_control.msg import pose
import numpy as np

def multiarray_to_numpy(multiarray):
    rows = multiarray.layout.dim[0].size
    cols = multiarray.layout.dim[1].size
    return np.array(multiarray.data).reshape((rows, cols))

def callback(data):
    for i, matrix_msg in enumerate(data.matrices):  #enumerate 用于同时获取索引和实际值
        matrix = multiarray_to_numpy(matrix_msg)
        rospy.loginfo(f"Matrix {i+1}:\n{matrix}")

def listener():
    rospy.init_node('matrix_subscriber')
    rospy.Subscriber('matrix_topic', pose, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()