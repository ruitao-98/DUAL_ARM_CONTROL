#! /home/yanji/anaconda3/envs/mujo/bin/python3
# -*- coding: utf-8 -*-

# # 将十进制数打印为十六进制格式，并在计算中使用整数
# print(f"十进制数: {decimal_number}")
# print(f"十六进制表示: {hex(decimal_number)}")  # 输出: 0x00ff

# # 进行一些计算
# result = decimal_number + 1
# print(f"计算结果: {result}")  # 输出: 256
# print(f"计算结果的十六进制表示: {hex(result)}")  # 输出: 0x0100

# # 如果需要填充零位，可以使用 format
# padded_hex = format(decimal_number, '#06x')
# print(f"填充零位后的表示: {padded_hex}")  # 输出: 0x00ff

# # 另一个示例，处理更大的数值
# big_decimal_number = 123
# padded_big_hex = format(big_decimal_number, '#06x')
# print(f"更大的数值填充零位后的表示: {padded_big_hex}")  # 输出: 0x007b
# import numpy
# Ma = numpy.array([-4.37061498e-11,  1.80234883e-06, -1.76238607e-06, -3.14200000e+01,
#   0.00000000e+00,  0.00000000e+00])
# adm_m = 0.1 * numpy.array([1, 1, 1, 1, 1, 1])
# print(numpy.divide(Ma, adm_m))
# print(numpy.ones(6) * 2000)

# import numpy as np
# from scipy.spatial.transform import Rotation as R

# # 定义旋转矩阵
# rotation_z = R.from_euler('z', -np.pi/2, degrees=False)
# rotation_y = R.from_euler('y', 0, degrees=False)
# rotation_x = R.from_euler('x', -np.pi/2, degrees=False)

# # 将旋转矩阵组合起来
# eef_offset_rotm = rotation_z * rotation_y * rotation_x

# # 提取旋转矩阵
# eef_offset_rotm_matrix = eef_offset_rotm.as_matrix()

# print(eef_offset_rotm_matrix)

# import pyRobotiqGripper
# from pyRobotiqGripper import RobotiqGripper

# gripper = RobotiqGripper()

# # gripper.activate()
# # gripper.calibrate(0, 40)
# # gripper.open()
# # gripper.close()
# gripper.goTo(100)
# position_in_bit = gripper.getPosition()
# print(position_in_bit)
# # gripper.goTomm(25)
# position_in_mm = gripper.getPositionmm()
# print(position_in_mm)


# for item in range(6):
#     print(item)
#     if item == 2:
#         continue
#     print('-----------------')

# import jkrc
# import ctypes

# # jkrc = ctypes.CDLL('./libjakaAPI.so')

# robot = jkrc.RC("192.168.3.200")#返回一个机器人对象
# robot.login() #登录
# # robot.power_on()
# robot.disable_robot()
# robot.logout() #登出
# import numpy as np
# from utils import *

# test = np.array([[ 0.51736281, -0.85362597,  0.06048471],
#                  [-0.85529045, -0.51342576,  0.06980094],
#                  [-0.02852949, -0.0878444,  -0.99572557]])

# rpy_temp_trans = from_matrix_to_eular(test)
# print(rpy_temp_trans)


import numpy as np
from scipy.spatial.transform import Rotation as R
import open3d as o3d

def calculate_axis_angles(N, theta):
    angles = []
    for i in range(N):
        phi = 2 * np.pi * i / N 
        n = np.array([np.sin(phi), -np.cos(phi), 0])
        axis_angle = (n, theta)
        angles.append(axis_angle)
    return angles

def axis_angle_to_rotation_matrix(axis, angle):
    rotation = R.from_rotvec(angle * axis)
    return rotation.as_matrix()

def create_coordinate_frame(rotation_matrix, origin=[0, 0, 0], size=0.1):
    mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=size)
    mesh_frame.rotate(rotation_matrix, center=[0, 0, 0])
    mesh_frame.translate(origin)
    return mesh_frame

N = 6  # 例如，10个点
theta = -np.radians(10)  # 例如，30度的旋转角

axis_angles = calculate_axis_angles(N, theta)

coordinate_frames = []
for idx, (axis, angle) in enumerate(axis_angles):
    rotation_matrix = axis_angle_to_rotation_matrix(axis, angle)
    coordinate_frame = create_coordinate_frame(rotation_matrix, origin=[0, 0, 0])
    coordinate_frames.append(coordinate_frame)

# 创建原始坐标系
original_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)

# 可视化所有坐标系
o3d.visualization.draw_geometries([original_frame] + coordinate_frames[:1])

