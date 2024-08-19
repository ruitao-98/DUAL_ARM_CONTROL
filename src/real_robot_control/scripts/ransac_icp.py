#!/home/yanji/anaconda3/envs/mujo/bin/python3
# -*- coding: utf-8 -*-

#*******************导入相关库***********************
import os
import open3d as o3d
import numpy as np
import time
from tkinter import filedialog
import matplotlib as mpl
import sys
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from utils import *
from real_robot_control.msg import pose
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import rospy
from generate_grasping_pose import *
import generate_grasping_pose as ge
import copy
#*******************导入mechmind***********************
import capture_pointcloud as cap
#*******************定义基础函数***********************

capture = cap.CapturePointCloud()

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

def draw_results(src_temp, tar_temp, result):
    source_temp = o3d.geometry.PointCloud(src_temp)
    target_temp = o3d.geometry.PointCloud(tar_temp)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(result)
    o3d.visualization.draw_geometries([source_temp, target_temp], window_name="result", width=1080, height=720)

def refine(T, src_down_o3, tar_down_o3, times):
    # ******************点云ICP配准*********************

    # target.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.08, max_nn=30))
    # source.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.08, max_nn=30))
    # print("ICPing...")
    if times == 2:
        threshold = 5
        reg_p2p_1 = o3d.pipelines.registration.registration_icp(
            src_down_o3, tar_down_o3, threshold, T,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50, relative_rmse=0.001))

        threshold = 0.5
        reg_p2p = o3d.pipelines.registration.registration_icp(
            src_down_o3, tar_down_o3, threshold, reg_p2p_1.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200, relative_rmse=0.001))

    elif times == 1:
        threshold = 3
        reg_p2p = o3d.pipelines.registration.registration_icp(
            src_down_o3, tar_down_o3, threshold, T,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200, relative_rmse=0.0001))
    else:
        print("error")
        return 0

    print("Transformation is:")
    print(reg_p2p.transformation)
    return reg_p2p.transformation

def execute_global_registration(src_down, tar_down, src_fpfh,
                                tar_fpfh, voxel_size):
    distance_threshold = 4
    print("   RANSAC registration on downsampled point clouds.")
    print("   Since the downsampling voxel size is %.3f," % voxel_size)
    print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        src_down, tar_down, src_fpfh, tar_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        4,  # Number of RANSAC iterations
        [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ],
        o3d.pipelines.registration.RANSACConvergenceCriteria(1000000, 300)
    )
    return result

def ransac(source, target):
    voxel_size = 0.3       #指定下采样体素大小

    source_down = source.voxel_down_sample(voxel_size)
    target_down = target.voxel_down_sample(voxel_size)

    source_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=30))
    target_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=30))

    source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        source_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))
    target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        target_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))

    result_ransac = execute_global_registration(source_down,target_down,source_fpfh,target_fpfh,voxel_size)

    return result_ransac


def trans_to_robot_base(pcd):
    """
    转换点云到基坐标系
    """
    points = np.array(pcd.points)
    present_x = rospy.get_param("present_x", -371.060)
    present_y = rospy.get_param("present_y", -146.762)
    present_z = rospy.get_param("present_z", 511.249)
    present_rx = rospy.get_param("present_rx", -171.289 * np.pi / 180)
    present_ry = rospy.get_param("present_ry", 7.497 * np.pi / 180)
    present_rz = rospy.get_param("present_rz", -44.49 * np.pi / 180)
    tcp_matrix = np.eye(4)
    # 定义旋转矩阵
    rotation_z = R.from_euler('z', present_rz, degrees=False)
    rotation_y = R.from_euler('y', present_ry, degrees=False)
    rotation_x = R.from_euler('x', present_rx, degrees=False)
    # 将旋转矩阵组合起来
    tcp_rotm = rotation_z * rotation_y * rotation_x
    # 提取旋转矩阵
    tcp_rotm_matrix = tcp_rotm.as_matrix()
    tcp_trans = np.array([present_x, present_y, present_z])

    tcp_matrix[:3, :3] = tcp_rotm_matrix
    tcp_matrix[:3, 3] = tcp_trans
    print(tcp_matrix)

    hand_eye_relation = np.array([[0.706330265364718,	-0.699256310511164,	-0.110173356307783,	81.6645139379991],
                                [0.695506456292791,	0.714500329812274,	-0.0758949797627990,	-48.1687434460167],
                                [0.131788942953691,	-0.0230193594278016,	0.991010486123473,	59.8289339845216],
                                [0,	0,	0,	1]])
    # hand_eye_relation = np.array([[0.693078358902796, -0.712313829861997, -0.110686025350721, 82.5957586617159],
    #                               [0.707931883219150, 0.701526837022340, -0.0818079804127876, -44.4147286980286],
    #                               [0.135922173107974, -0.0216588255629318, 0.990482739946962,  60.2262194514354],
    #                               [0, 0, 0, 1]])
    base_eye_relation = np.dot(tcp_matrix, hand_eye_relation)
    points_augment = np.hstack((points, np.ones((len(points), 1))))
    points_augment_base = np.dot(base_eye_relation, points_augment.T)
    points_augment_base = points_augment_base.T # 转换为n*4

    points_base = points_augment_base[:,0:3]
    points_base_pcd = o3d.geometry.PointCloud()
    points_base_pcd.points = o3d.utility.Vector3dVector(points_base)
    # cloud_temp = o3d.geometry.PointCloud(cloud)
    points_base_pcd.paint_uniform_color([1, 0.706, 0])

    return points_base_pcd


def pass_through_filter(point_cloud, axis, lower_limit, upper_limit):
    """
    直通滤波器
    """
    # 获取点云数据
    points = np.asarray(point_cloud.points)

    # 过滤点
    if axis == 'x':
        mask = (points[:, 0] >= lower_limit) & (points[:, 0] <= upper_limit)
    elif axis == 'y':
        mask = (points[:, 1] >= lower_limit) & (points[:, 1] <= upper_limit)
    elif axis == 'z':
        mask = (points[:, 2] >= lower_limit) & (points[:, 2] <= upper_limit)
    else:
        raise ValueError("Axis must be 'x', 'y' or 'z'")

    filtered_points = points[mask]
    # 创建新的点云对象
    filtered_cloud = o3d.geometry.PointCloud()
    filtered_cloud.points = o3d.utility.Vector3dVector(filtered_points)

    return filtered_cloud


def create_custom_coordinate_frame(size=1.0, origin=[0, 0, 0]):
    mesh = o3d.geometry.TriangleMesh()
    colors = [[0, 1, 0], [0, 0, 1]]  # X is red, Z is blue

    # X axis
    y_arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.02 * size, cone_radius=0.04 * size,
                                                     cylinder_height=0.8 * size, cone_height=0.2 * size)
    y_arrow.paint_uniform_color(colors[0])
    y_arrow.translate(origin)
    y_arrow.rotate(o3d.geometry.get_rotation_matrix_from_xyz([-np.pi / 2, 0, 0]), center=origin)

    # Z axis
    z_arrow = o3d.geometry.TriangleMesh.create_arrow(cylinder_radius=0.02 * size, cone_radius=0.04 * size,
                                                     cylinder_height=0.8 * size, cone_height=0.2 * size)
    z_arrow.paint_uniform_color(colors[1])
    z_arrow.translate(origin)
    z_arrow.rotate(o3d.geometry.get_rotation_matrix_from_xyz([0, 0, np.pi / 2]), center=origin)

    mesh += y_arrow
    mesh += z_arrow

    return mesh

def generate_coodinate(poss, rpys):
    axis_pcds = []
    for i in range(len(poss)):
        # axis_pcd = create_custom_coordinate_frame(size=30, origin=poss[i])
        axis_pcd = o3d.geometry.TriangleMesh.create_coordinate_frame(size=30, origin=poss[i])
        R = axis_pcd.get_rotation_matrix_from_zyx((rpys[i][2], rpys[i][1], rpys[i][0]))
        axis_pcd.rotate(R, center=(poss[i][0], poss[i][1], poss[i][2]))
        axis_pcds.append(axis_pcd)

    return axis_pcds




def detect_colision(pcd1, pcd2):
    # 降采样点云 (可选，但推荐)
    pcd1 = pcd1.voxel_down_sample(voxel_size=0.05)
    pcd2 = pcd2.voxel_down_sample(voxel_size=0.05)
    print("---*******------")
    # 将点云转换为凸包三角网格
    mesh1, _ = pcd1.compute_convex_hull()
    mesh1.compute_vertex_normals()
    mesh2, _ = pcd2.compute_convex_hull()
    mesh2.compute_vertex_normals()
    print("---------")
    # 设置颜色以区分两个凸包
    mesh1.paint_uniform_color([1, 0, 0])  # 红色
    mesh2.paint_uniform_color([0, 1, 0])  # 绿色
        # 计算两个网格是否相交
    intersection = mesh1.is_intersecting(mesh2)

    if intersection:
        print("两个物体出现了干涉")
    else:
        print("两个物体没有出现干涉")

    # 可视化点云和对应的凸包
    # o3d.visualization.draw_geometries([pcd1, pcd2, mesh1, mesh2], width=1080, height=720)
    return intersection


# 下面的函数，是对 main 中部分程序的封装，方便在grasp_planning中调用
#####################
def get_pointcloud_from_data():
    """
    src: 模板， tar：采集的数据
    """
    src = o3d.io.read_point_cloud('/home/yanji/dual_arm_control/src/real_robot_control/scripts/grasp_detection/pointcloud/handle.pcd')  # 模板点云
    # tar = o3d.io.read_point_cloud('/home/yanji/dual_arm_control/src/real_robot_control/scripts/grasp_detection/pointcloud/obj_7_0 - Cloud.pcd')  # 采集点云
    desk = o3d.io.read_point_cloud('/home/yanji/dual_arm_control/src/real_robot_control/scripts/grasp_detection/pointcloud/desk.pcd')  # 模板点云
    gripper = o3d.io.read_point_cloud("/home/yanji/dual_arm_control/src/real_robot_control/scripts/grasp_detection/pointcloud/gripper.pcd") # 夹爪点云，用于计算碰撞
    tar = o3d.io.read_point_cloud('/home/yanji/dual_arm_control/src/real_robot_control/scripts/grasp_detection/pointcloud/obj_7_1 - Cloud.pcd')  # 采集点云
    tar = trans_to_robot_base(tar)  # 变化到机器人基坐标系下
    tar = pass_through_filter(tar, axis='z', lower_limit=2, upper_limit=100)
    tar = pass_through_filter(tar, axis='y', lower_limit=-100, upper_limit=100)
    voxel_size2 = 0.6 #指定下采样体素大小
    voxel_size1 = 0.4
    tar_down_o3 = tar.voxel_down_sample(voxel_size2)
    src_down_o3 = src.voxel_down_sample(voxel_size1)
    # src, tar 基本只用于可视化
    return src, tar, src_down_o3, tar_down_o3, desk, gripper



def get_pointcloud_from_camera(file_name):
    base_path = "/home/yanji/dual_arm_control/src/real_robot_control/scripts/grasp_detection/pointcloud"
    path = os.path.join(base_path, file_name)
    src = o3d.io.read_point_cloud(path)  # 模板点云
    desk = o3d.io.read_point_cloud('/home/yanji/dual_arm_control/src/real_robot_control/scripts/grasp_detection/pointcloud/desk.pcd')  # 模板点云
    gripper = o3d.io.read_point_cloud("/home/yanji/dual_arm_control/src/real_robot_control/scripts/grasp_detection/pointcloud/gripper.pcd") # 夹爪点云，用于计算碰撞
    # 相机采集tar 数据，待更新。。。。
    tar_raw = capture.get_point_cloud()
    tar = trans_to_robot_base(tar_raw)  # 变化到机器人基坐标系下
    tar = pass_through_filter(tar, axis='z', lower_limit=4, upper_limit=100)
    tar = pass_through_filter(tar, axis='y', lower_limit=-200, upper_limit=100)
    voxel_size2 = 0.6 #指定下采样体素大小
    voxel_size1 = 0.4
    tar_down_o3 = tar.voxel_down_sample(voxel_size2)
    src_down_o3 = src.voxel_down_sample(voxel_size1)
    # src, tar 基本只用于可视化
    return src, tar, src_down_o3, tar_down_o3, desk, gripper


def get_boundingbox(pcd):
    aabb = ge.calculate_bounding_box(pcd)
    vertices = ge.get_box_points(aabb)  # 在模板点云上计算boundingbox 和 bounding box的点，供后续使用
    return aabb, vertices

def get_object_pose(src_down_o3, tar_down_o3):
    T = np.eye(4)
    src_temp, tar_temp = copy.deepcopy(src_down_o3), copy.deepcopy(tar_down_o3)
    result_ransac = ransac(src_down_o3, tar_down_o3)
    result = refine(result_ransac.transformation, src_down_o3, tar_down_o3, 2)
    draw_results(src_temp, tar_temp, result)

    print('please make sure the registration results')
    decision = wait_for_key_press()
    while decision == 0:
        result_ransac = ransac(src_down_o3, tar_down_o3)
        result = refine(result_ransac.transformation, src_down_o3, tar_down_o3, 2)
        draw_results(src_down_o3, tar_down_o3, result)
        decision = wait_for_key_press()
    
    return result

def get_transformed_grasp_poses(num_grasp, vertices, result, gripper, desk):
    """
    num_grasp: number of the grasp poses
    vertices: bounding box's corner points, with a mount of 8
    generate grasp poses that the robot can reach
    """
    poss_trans, rpys_trans, T_g = ge.get_all_grasp_matrixes(num_grasp, vertices)

    #批量转化抓取点
    T_g_trans = {}
    grippers = []
    for i, T_temp in enumerate(T_g):
        T_temp_trans = np.dot(result, T_temp).astype(np.float32)
        gripper_trans = copy.deepcopy(gripper)
        # 设置 mesh 的颜色（例如，设置为白色）
        gripper_trans.transform(T_temp_trans)
        interaction = detect_colision(desk, gripper_trans)
        if not interaction:
            grippers.append(gripper_trans)
            # T_g_trans.append(T_temp_trans)
            key = i  # 动态生成键
            value = T_temp_trans   # 动态生成值
            T_g_trans[key] = value
    
    return poss_trans, rpys_trans, T_g, T_g_trans

def calculate_relative_matrixs(T_g_i, T_s):
    # 给定其中某一个抓取位姿，计算所有的旋拧位姿下的抓取位姿的表示
    
    T_relatives = []
    
    for i in range(len(T_s)):
        T_temp = np.dot(np.linalg.inv(T_s[i]), T_g_i)
        T_relatives.append(T_temp)

    return T_relatives


######################

#主程序
if __name__ == "__main__":

    rospy.init_node('matrix_publisher')
    pub = rospy.Publisher('matrix_topic', pose, queue_size=10)

    #使用open3d读取
    gripper = o3d.io.read_point_cloud("/home/yanji/dual_arm_control/src/real_robot_control/scripts/grasp_detection/pointcloud/gripper.pcd") # 夹爪点云，用于计算碰撞

    src = o3d.io.read_point_cloud('/home/yanji/dual_arm_control/src/real_robot_control/scripts/grasp_detection/pointcloud/handle.pcd')  # 模板点云
    desk = o3d.io.read_point_cloud('/home/yanji/dual_arm_control/src/real_robot_control/scripts/grasp_detection/pointcloud/desk.pcd')  # 模板点云
    aabb = calculate_bounding_box(src)
    vertices = get_box_points(aabb)  # 在模板点云上计算boundingbox 和 bounding box的点，供后续使用

    tar = o3d.io.read_point_cloud('/home/yanji/dual_arm_control/src/real_robot_control/scripts/grasp_detection/pointcloud/obj_7_0 - Cloud.pcd')  # 采集点云
    tar = trans_to_robot_base(tar)  # 变化到机器人基坐标系下
    tar = pass_through_filter(tar, axis='z', lower_limit=2, upper_limit=100)
    tar = pass_through_filter(tar, axis='y', lower_limit=-100, upper_limit=100)

    voxel_size2 = 0.6 #指定下采样体素大小
    voxel_size1 = 0.4
    tar_down_o3 = tar.voxel_down_sample(voxel_size2)
    src_down_o3 = src.voxel_down_sample(voxel_size1)
    src_temp = src
    tar_temp = tar


    num = 6 # 这个数值指的是生成旋拧点的个数，取决于物体的对称程度，六边形就是6，只是对称就是2，圆形可以是任意。
    poss, rpys, T_s = screwing_poses(num)
   
    axis_pcds = generate_coodinate(poss, rpys)
    # o3d.visualization.draw_geometries([src, tar , axis_pcds[0]], window_name="result", width=1080, height=720)
    o3d.visualization.draw_geometries([src, tar] + axis_pcds, window_name="result", width=1080, height=720)


    # 位姿测量，直到配准成功
    T = np.eye(4)
    result_ransac = ransac(src_down_o3, tar_down_o3)
    result = refine(result_ransac.transformation, src_down_o3, tar_down_o3, 1)
    draw_results(src_temp, tar_temp, result)

    print('please make sure the registration results')
    decision = wait_for_key_press()
    while decision == 0:
        result_ransac = ransac(src_down_o3, tar_down_o3)
        result = refine(result_ransac.transformation, src_down_o3, tar_down_o3, 1)
        draw_results(src_temp, tar_temp, result)
        decision = wait_for_key_press()

    # 获取抓取点
    num_grasp = 20
    poss_trans, rpys_trans, T_grasp = get_all_grasp_matrixes(num_grasp, vertices)

    #批量转化抓取点
    T_g_trans = []
    grippers = []
    for i in range(len(T_grasp)):
        T_temp_trans = np.dot(result, T_grasp[i]).astype(np.float32)
        gripper_trans = copy.deepcopy(gripper)
        # 设置 mesh 的颜色（例如，设置为白色）
        gripper_trans.paint_uniform_color([1, 0.8, 0.8])
        gripper_trans.transform(T_temp_trans)
        interaction = detect_colision(desk, gripper_trans)
        if not interaction:
            grippers.append(gripper_trans)
            T_g_trans.append(numpy_to_multiarray(T_temp_trans))
    
    # 发布结果：
    matrix_array_msg = pose()
    matrix_array_msg.matrices = T_g_trans
    pub.publish(matrix_array_msg) # 这里发布了筛选了碰撞了以后的抓取点，并发布出去

    # 可视化测试
    # source_temp = o3d.geometry.PointCloud(src_temp)
    # target_temp = o3d.geometry.PointCloud(tar_temp)
    # source_temp.transform(result)
    # o3d.visualization.draw_geometries(meshs + [source_temp, target_temp], width=1080,
    #                                   height=720)

    # 建立每个抓取点和旋拧点的对应关系，记录其对应的矩阵
    # 为每个抓取点和旋拧点都有最佳的匹配对，根据筛选的抓取点，优先选择旋拧点，进行rrt规划，若不行，再换下一个
    # 闭环：若检测不到抓取点，执行筛选的动作原语


    poss_trans = []
    rpys_trans = []
    T_s_trans = []
    # 批量转换旋拧点，生成旋拧点的矩阵
    for i in range(len(T_s)):
        T_temp_trans = np.dot(result, T_s[i]).astype(np.float32) # 此处不能用这个result 乘，应用用最终的要传递时候的位姿计算
        rpy_temp_trans = from_matrix_to_eular(T_temp_trans[:3,:3])
        pos_temp_trans = T_temp_trans[:3, 3]
        T_s_trans.append(numpy_to_multiarray(T_temp_trans))
        rpys_trans.append(rpy_temp_trans)
        poss_trans.append(pos_temp_trans)
    



    axis_pcds_trans = generate_coodinate(poss_trans, rpys_trans)

    source_temp = o3d.geometry.PointCloud(src_temp)
    target_temp = o3d.geometry.PointCloud(tar_temp)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(result)
    o3d.visualization.draw_geometries([source_temp, target_temp, desk] + grippers + axis_pcds_trans, window_name="result", width=1080, height=720)



