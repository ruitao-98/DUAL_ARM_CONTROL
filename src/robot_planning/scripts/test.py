# from dm_control import mujoco
#
# # Load a model from an MJCF XML string.
# xml_string = """
# <mujoco>
#   <worldbody>
#     <light name="top" pos="0 0 1.5"/>
#     <geom name="floor" type="plane" size="1 1 .1"/>
#     <body name="box" pos="0 0 .3">
#       <joint name="up_down" type="slide" axis="0 0 1"/>
#       <geom name="box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
#       <geom name="sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
#     </body>
#   </worldbody>
# </mujoco>
# """
# physics = mujoco.Physics.from_xml_string(xml_string)
#
# # Render the default camera view as a numpy array of pixels.
# pixels = physics.render()
#
# # Reset the simulation, move the slide joint upwards and recompute derived
# # quantities (e.g. the positions of the body and geoms).
# with physics.reset_context():
#   physics.named.data.qpos['up_down'] = 0.5
#
# # Print the positions of the geoms.
# print(physics.named.data.geom_xpos)
# # FieldIndexer(geom_xpos):
# #            x         y         z
# # 0  floor [ 0         0         0       ]
# # 1    box [ 0         0         0.8     ]
# # 2 sphere [ 0.2       0.2       1       ]
#
# # Advance the simulation for 1 second.
# while physics.time() < 1.:
#   physics.step()
#
# # Print the new z-positions of the 'box' and 'sphere' geoms.
# print(physics.named.data.geom_xpos[['box', 'sphere'], 'z'])
# # [ 0.19996362  0.39996362]

import time
from threading import Thread, Lock
import numpy as np
lock = Lock()  # 创建锁对象
n = 0


def task1():
  global n
  # global lock
  lock.acquire()
  for i in range(800000):
    n += 1
  lock.release()


def task2():
  global n
  lock.acquire()
  print("task2: n is {}".format(n))
  lock.release()

adm_rotm = np.zeros((3,3))
print(adm_rotm)
# if __name__ == '__main__':
  # print("这里是主线程")
  # # 创建线程对象
  # t1 = Thread(target=task1)
  # t2 = Thread(target=task2)
  # # 启动
  # t1.start()
  # t2.start()
  # print("main: n is {}".format(n))  # 未使用lock的线程仍然访问到错误数据
  # time.sleep(0.3)
  # print("主线程结束了")
  # print( 0< 1 and 1>2)
  # # joint = [[2,3,4,5,6,7,8,9,10,11,12], [3,4,5,6,7,8,9,10,11]]
  # J = []
  # j1 = [1,2,3,4,5,6,7,8,9]
  # j2 = [1,2,3,4,5,6,7,8,0]
  # j3 = [1,2,3,4,5,6,7,8,1]
  # # for i in range(3):
  # J.append(j1)
  # J.append(j2)
  # J.append(j3)
  # print(J)
  # for i in range(len(J)):
  #   print(J[i])
    # roslaunch robot_moveit_config_ver2 demo.launch

# import threading
# import queue
# import time
# import random
#
# # 初始化共享队列和条件变量
# shared_queue = queue.Queue()
# queue_not_empty = threading.Event()
# queue_empty = threading.Event()
# queue_empty.set()  # 默认设置队列为空的事件，因为开始时队列确实是空的
#
# def producer():
#     while 1:  # 使用一个事件来控制生产者何时停止生产
#         queue_empty.wait()  # 等待队列为空
#         print("Producer is generating numbers...")
#         for _ in range(5):  # 一次性通过循环生成多个数字
#             item = random.randint(1, 100)  # 使用随机数来模拟生成的数据
#             shared_queue.put(item)
#             print(f"Produced {item}")
#         queue_not_empty.set()  # 设置队列不为空的事件
#         queue_empty.clear()  # 清除队列为空的事件标志
#         time.sleep(2)  # 假设生产数据需要一些时间
#
#
# def consumer():
#   while not stop_production.is_set():  # 同样使用stop_production来控制消费者何时停止
#     queue_not_empty.wait()  # 等待队列不为空
#     try:
#       while not shared_queue.empty():  # 消费队列中的所有项目
#         item = shared_queue.get_nowait()
#         print(f"Consumed {item}")
#         time.sleep(1)  # 假设处理数据需要一些时间
#     except queue.Empty:
#       pass
#
#     if shared_queue.empty():
#       queue_not_empty.clear()  # 清除队列不为空的事件标志，因为队列已经被清空
#       queue_empty.set()  # 设置队列为空的事件
#
#
# stop_production = threading.Event()  # 控制生产者和消费者停止的事件
#
# producer_thread = threading.Thread(target=producer)
# consumer_thread = threading.Thread(target=consumer)
#
# producer_thread.start()
# consumer_thread.start()
#
# # 运行一段时间后停止，例如10秒
# time.sleep(10)
# stop_production.set()  # 告诉生产者和消费者停止生产和消费
#
# producer_thread.join()
# consumer_thread.join()

import time
import mujoco as mj
import mujoco_viewer
import mujoco.viewer
import numpy as np
import func_ki_dy as func_mj


# model = mj.MjModel.from_xml_path("../../robot_description/xml/dual_arm.xml")
# data = mj.MjData(model)
# RbtMj = func_mj.Robot(data, model, 'grasping_frame', 'screw_frame')
#
# mj.mj_step(model,data)
# # viewer1 = mujoco_viewer.MujocoViewer(model, data)
# viewer2 = mj.viewer.launch_passive(model, data)
# # self.cam.azimuth = 0
# # self.cam.elevation = -40
# # self.cam.distance = 4
# # self.cam.lookat = np.array([0.3, 0.8, 0])
# viewer2.cam.azimuth = 0
# viewer2.cam.elevation = -40
# viewer2.cam.distance = 4
# viewer2.cam.lookat = np.array([0.3,0.8,0])
#
# for i in range(800):
#   id_left_base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j1')
#   left_index = np.arange(id_left_base, id_left_base + 6)
#   id_right_base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'r_j1')
#   right_index = np.arange(id_right_base, id_right_base + 6)
#   data.ctrl[left_index], data.ctrl[right_index] = RbtMj.coriolis_gravity()
#   initial_qpos = {
#     'l_j1': 0.506,
#     'l_j2': 1.2314,
#     'l_j3': 2.1612,
#     'l_j4': 2.8390,
#     'l_j5': 1.58,
#     'l_j6': 2.3884,
#     'r_j1': -0.7297,
#     'r_j2': 0.8122,
#     'r_j3': 2.0985,
#     'r_j4': 1.803,
#     'r_j5': -1.58,
#     'r_j6': 1.7431,
#   }
#
#   for name, value in initial_qpos.items():
#     data.qpos[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)] = value
#
#   mj.mj_step(model, data)
#   viewer2.sync()
  # viewer1.render()
# time.sleep(20)
# import numpy as np
# print(10 * np.array([1, 1, 1, 1, 1, 1]))
# adm_kp = 10 * np.array([1, 1, 1, 1, 1, 1])
# adm_m = 1 * np.array([1, 1, 1, 1, 1, 1])
# print( 4 * np.sqrt(np.multiply(adm_kp, adm_m)))
# import mujoco.viewer
# world_force = np.array([-12, 15, 9, -15, 1, 1, 10])
# world_force = np.clip(world_force, -10, 10)
# print(world_force)
# a = np.array([[0., 0., 0.],
#               [1,2,3]])
# b = np.array([[0, 1, 2],
#               [1,2,3]])
# print(np.concatenate((a, b), axis=1))
#
# a1 = np.array([[0., 0., 0.],
#               [1,2,3],
#               [1,2,3]])
# print(a1[0,2])


import numpy as np


def compute_transformation_matrix(points1, points2):
    assert points1.shape == points2.shape
    n = points1.shape[0]

    # Centroid of the points
    centroid1 = np.mean(points1, axis=0)
    centroid2 = np.mean(points2, axis=0)

    # Center the points
    centered_points1 = points1 - centroid1
    centered_points2 = points2 - centroid2

    # Compute covariance matrix
    H = np.dot(centered_points1.T, centered_points2)

    # Singular Value Decomposition
    U, S, Vt = np.linalg.svd(H)

    # Compute rotation matrix
    R = np.dot(Vt.T, U.T)

    # Special reflection case
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = np.dot(Vt.T, U.T)

    # Compute translation vector
    T = centroid2.T - np.dot(R, centroid1.T)

    return R, T


# Example usage
# points1 = np.array([[-244.564, -533.817, 126.904],
#                     [-190.022, -616.583, 121.246],
#                     [-274.066, -507.59, 117.855],
#                     [-48.404, -538.715, 125.217],
#                     [-250.114, -440.294, 125.272],
#                     [-152.869, -616.777, 147.857]])
# points2 = np.array([[-215.356, 547.047, 126.142],
#                     [-160.32, 484.284, 120.14],
#                     [-245.271, 593.112, 116.924],
#                     [-18.076, 562.381, 124.263],
#                     [-221.466, 660.207, 124.318],
#                     [-123.38, 484.291, 146.903]])

points1 = np.array([
                    [-48.404, -538.715, 125.217],
                    [-250.114, -440.294, 125.272],
                    [-152.869, -616.777, 147.857],
                    [-153.419, -617.897, 160.188],
                    [-248.848, -649.433, 105.295],
                    [-316.964, -606.287, 104.076],
                    [-65.46, -507.979, 154.688]])
points2 = np.array([
                    [-18.076, 562.381, 124.263],
                    [-221.466, 660.207, 124.318],
                    [-123.38, 484.291, 146.903],
                    [-124.029, 483.404, 158.872],
                    [-220.325, 451.124, 104.132],
                    [-288.031, 494.534, 103.076],
                    [-36.801, 593.185, 153.384]])

R, T = compute_transformation_matrix(points1, points2)
print("Rotation Matrix:\n", R)
print("Translation Vector:\n", T)