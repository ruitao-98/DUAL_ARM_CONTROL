#! /home/yanji/anaconda3/envs/mujo/bin/python3

import rospy
import numpy as np
from real_robot_control.msg import pose
import open3d
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

def numpy_to_multiarray(matrix):
    multiarray = Float32MultiArray()
    multiarray.layout.dim.append(MultiArrayDimension())
    multiarray.layout.dim.append(MultiArrayDimension())
    multiarray.layout.dim[0].label = "rows"
    multiarray.layout.dim[0].size = matrix.shape[0]
    multiarray.layout.dim[0].stride = matrix.shape[0] * matrix.shape[1]
    multiarray.layout.dim[1].label = "cols"
    multiarray.layout.dim[1].size = matrix.shape[1]
    multiarray.layout.dim[1].stride = matrix.shape[1]
    multiarray.data = matrix.flatten().tolist()
    return multiarray

def publish_matrices():
    rospy.init_node('matrix_publisher')
    pub = rospy.Publisher('matrix_topic', pose, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        matrices = []
        for _ in range(np.random.randint(1, 5)):  # 随机生成1到4个矩阵
            matrix = np.random.rand(4, 4).astype(np.float32)
            matrices.append(numpy_to_multiarray(matrix))

        matrix_array_msg = pose()
        matrix_array_msg.matrices = matrices
        pub.publish(matrix_array_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_matrices()
    except rospy.ROSInterruptException:
        pass