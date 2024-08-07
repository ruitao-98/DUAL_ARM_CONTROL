#!/home/yanji/anaconda3/envs/mujo/bin/python3

# With this sample, you can obtain and save the untextured and textured point clouds.
# 需要实现以下功能：
"""
1. 操作相机实现拍摄
2. 保存点云为ply，到指定文件夹
3. 返回一个open3d的点云对象或者numpy的3*n的矩阵，方便后续处理
4. 2，3功能应封装为两个函数，供后续算法处理
"""
from mecheye.shared import *
from mecheye.area_scan_3d_camera import *
from mecheye.area_scan_3d_camera_utils import find_and_connect, confirm_capture_3d
import open3d as o3d


class CapturePointCloud(object):
    def __init__(self):
        self.camera = Camera()
        self.frame_all_2d_3d = Frame2DAnd3D()
        self.frame_3d = Frame3D()

    def capture_point_cloud(self):
        point_cloud_file = "/home/yanji/dual_arm_control/src/real_robot_control/scripts/grasp_detection/captured_data/PointCloud.pcd"
 
        self.camera.capture_3d(self.frame_3d)
        cloud = self.frame_3d.get_untextured_point_cloud()

        show_error(
            self.frame_3d.save_untextured_point_cloud(FileFormat_PCD, point_cloud_file))
        print("Capture and save the untextured point cloud: {}.".format(
            point_cloud_file))

    def get_point_cloud(self):
        self.camera.connect('192.168.3.2')
        self.capture_point_cloud()

        self.camera.disconnect()
        tar = o3d.io.read_point_cloud('/home/yanji/dual_arm_control/src/real_robot_control/scripts/grasp_detection/captured_data/PointCloud.pcd')
        print("Disconnected from the camera successfully.")

        return tar


if __name__ == '__main__':
    a = CapturePointCloud()
    a.get_point_cloud()
