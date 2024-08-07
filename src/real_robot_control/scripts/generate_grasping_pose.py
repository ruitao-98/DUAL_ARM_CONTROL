#! /home/yanji/anaconda3/envs/mujo/bin/python3
# -*- coding: utf-8 -*-

import open3d as o3d
from scipy.spatial.transform import Rotation as R
import numpy as np
import ransac_icp as ran
import copy
from utils import *
def calculate_bounding_box(pcd):
    aabb = pcd.get_axis_aligned_bounding_box()
    aabb.color = (1, 0, 0)
    return aabb


def get_box_points(aabb):
    min_bound = aabb.get_min_bound()
    max_bound = aabb.get_max_bound()

    # 计算包围盒各个方向的长度
    length_x = max_bound[0] - min_bound[0]
    length_y = max_bound[1] - min_bound[1]
    length_z = max_bound[2] - min_bound[2]

    # 获取包围盒的顶点位置
    vertices = aabb.get_box_points()

    print(f"Length in X direction: {length_x}")
    print(f"Length in Y direction: {length_y}")
    print(f"Length in Z direction: {length_z}")
    print("Vertices of the bounding box:")
    print(vertices)

    for vertex in vertices:
        print(vertex)

    return  vertices

def get_grasping_points(vertices):
    points_parrell = []
    points_vertical = []
    max_z = min_z = 0
    threshold = 30
    # 生成水平交接的抓取点
    for item in vertices:

        # 统计最大最小z值
        if item[2] >= max_z:
            max_z = item[2]
        if item[2] <= min_z:
            min_z = item[2]
        # 添加符合水平递送条件的点
        if item[2] > 0:
            if item[2] > threshold:
                if [0, 0, (item[2] + threshold) / 2] in points_parrell:
                    pass
                else:
                    points_parrell.append([0, 0, (item[2] + threshold) / 2])
        else:
            if item[2] < -threshold:
                if [0, 0, (item[2] + threshold) / 2] in points_parrell:
                    pass
                else:
                    points_parrell.append([0, 0, (item[2] - threshold) / 2 + 10])

    if (abs(max_z) + abs(min_z) <= 42): # 竖直交接不能超过夹爪行程，夹爪行程为50mm
        points_vertical.append([0, 0, (max_z + min_z) / 2])

    return points_vertical, points_parrell

def get_grasping_poses(num):
    poses_vertical = []
    poses_parrell = []

    rotation_y = R.from_euler('y', -90 * np.pi / 180, degrees=False)
    # 将旋转矩阵组合起来
    rot_vert_base = rotation_y
    # 提取旋转矩阵
    rot_vert_base_matrix = rot_vert_base.as_matrix()

    for i in range(num):
        rpy_temp = [0, 0, 0]
        rpy_temp[0] = (2 * np.pi / num) * i  # 垂直是绕着x轴旋转

        rotation_z = R.from_euler('z', rpy_temp[2], degrees=False)
        rotation_y = R.from_euler('y', rpy_temp[1], degrees=False)
        rotation_x = R.from_euler('x', rpy_temp[0], degrees=False)
        # 将旋转矩阵组合起来
        r_rotm = rotation_z * rotation_y * rotation_x
        # 提取旋转矩阵
        r_rotm_matrix = r_rotm.as_matrix()
        poses_vertical.append(np.dot(rot_vert_base_matrix, r_rotm_matrix))


    rotation_y = R.from_euler('y', -90 * np.pi / 180, degrees=False)
    rotation_z = R.from_euler('z', 90 * np.pi / 180, degrees=False)
    # 将旋转矩阵组合起来
    rot_parr_base = rotation_y * rotation_z
    # 提取旋转矩阵
    rot_parr_base_matrix = rot_parr_base.as_matrix()

    for i in range(num):
        rpy_temp = [0, 0, 0]
        rpy_temp[1] = (2 * np.pi / num) * i  # 平行是绕着y轴旋转

        rotation_z = R.from_euler('z', rpy_temp[2], degrees=False)
        rotation_y = R.from_euler('y', rpy_temp[1], degrees=False)
        rotation_x = R.from_euler('x', rpy_temp[0], degrees=False)
        # 将旋转矩阵组合起来
        r_rotm = rotation_z * rotation_y * rotation_x
        # 提取旋转矩阵
        r_rotm_matrix = r_rotm.as_matrix()
        poses_parrell.append(np.dot(rot_parr_base_matrix, r_rotm_matrix))

    return poses_parrell, poses_vertical

def get_all_grasp_matrixes(num, vertices):
    "num 参数的含义为，绕轴生成可选抓取点的数量，决定了候选抓取点的密度，此数值越大，推理速度越慢，但精度越高"
    poses_parrell, poses_vertical =  get_grasping_poses(num)
    points_vertical, points_parrell = get_grasping_points(vertices)
    poss_trans = []
    rpys_trans = []
    T_grasp = []
    T_temp = np.eye(4)
    print("len(points_parrell)", len(points_parrell) )
    if len(points_parrell) != 0:
        for i in range(len(points_parrell)):
            for j in range(num):
                rpy_temp_trans = from_matrix_to_eular(poses_parrell[j])
                pos_temp_trans = points_parrell[i]
                # 生成抓取矩阵
                T_temp[:3, :3] = poses_parrell[j]
                T_temp[:3, 3] = np.array(points_parrell[i])
                T_grasp.append(T_temp.copy())
                rpys_trans.append(rpy_temp_trans)
                poss_trans.append(pos_temp_trans)

    if len(points_vertical) != 0:
        for i in range(len(points_vertical)):
            for j in range(num):
                rpy_temp_trans = from_matrix_to_eular(poses_vertical[j])
                pos_temp_trans = points_vertical[i]
                # 生成抓取矩阵
                T_temp[:3, :3] = poses_vertical[j]
                T_temp[:3, 3] = np.array(points_vertical[i])
                T_grasp.append(T_temp.copy())
                
                rpys_trans.append(rpy_temp_trans)
                poss_trans.append(pos_temp_trans)

    return poss_trans, rpys_trans, T_grasp

def screwing_poses(num):
    # 所有的旋拧位姿，都默认位置为0，这是提前标定好的最佳位姿
    # 姿态是可以变化的

    pos = [0,0,0]
    poss = []
    rpys = []
    T_s = []
    # 此处生成矩阵，没有经过测量结果变换的结果
    for i in range(num):
        T_temp = np.eye(4)
        rpy_temp = [0, 0, 0]
        rpy_temp[2] = (2 * np.pi / num) * i

        r_rotm_matrix =  from_eular_to_matrix(rpy_temp)
   
        T_temp[:3, 3] = np.array(pos)
        T_temp[:3, :3] = r_rotm_matrix  #将欧拉角转换为旋转矩阵，全部记录

        T_s.append(T_temp)
        rpys.append(rpy_temp)
        poss.append(pos)

    return poss, rpys, T_s






if __name__ == "__main__":
    src = o3d.io.read_point_cloud('/home/yanji/dual_arm_control/src/real_robot_control/scripts/grasp_detection/pointcloud/handle.pcd')  # 模板点云
    aabb = calculate_bounding_box(src)
    vertices = get_box_points(aabb)
    o3d.visualization.draw_geometries([src, aabb], window_name="result", width=1080, height=720,
                                      zoom=1.2,
                                      front=[0.6, -0.2, -0.7],
                                      lookat=[10, 1.5, 1.338],
                                      up=[-0.17, -0.5, 0.1608])
    poss_trans = []
    rpys_trans = []
    # 批量转换抓取点，生成抓取点的矩阵
    num = 15
    poss_trans, rpys_trans, T_grasp = get_all_grasp_matrixes(num, vertices)
    axis_pcds_trans = ran.generate_coodinate(poss_trans, rpys_trans)
    o3d.visualization.draw_geometries([src] + axis_pcds_trans, window_name="result", width=1080,
                                      height=720)

    mesh = o3d.io.read_point_cloud("/home/yanji/dual_arm_control/src/real_robot_control/scripts/grasp_detection/pointcloud/gripper.pcd")
    # mesh = o3d.io.read_triangle_mesh("/home/yanji/dual_arm_control/src/real_robot_control/scripts/grasp_detection/pointcloud/gripper.ply")
    meshs = []
    lines = []
    for i in range(len(axis_pcds_trans)):
        # 创建 mesh 的副本并应用第一次变换
        mesh_transformed = copy.deepcopy(mesh)
        # 设置 mesh 的颜色（例如，设置为白色）
        mesh_transformed.paint_uniform_color([1, 0.8, 0.8])
        mesh_transformed.transform(T_grasp[i])
        # 创建一个线框（wireframe）表示以增强轮廓
        # lineset = o3d.geometry.LineSet.create_from_triangle_mesh(mesh_transformed)
        # lineset.paint_uniform_color([0, 0, 0])  # 设置线框的颜色为黑色

        meshs.append(mesh_transformed)
        # lines.append(lineset)

    # 可视化 mesh 和线框
    o3d.visualization.draw_geometries(meshs + [src], width=1080,
                                      height=720)




