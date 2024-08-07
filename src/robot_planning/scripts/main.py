import time

import dm_control as dm
import mujoco as mj
import mujoco.viewer
import numpy as np
from dm_control import mjcf
from dm_control import mujoco
import viewer as vi
from dm_control import suite
# import moveit_ros_planning
import math
import numpy as np
import func_ki_dy as func_mj
right_path = "../../robot_description/xml/dual_arm.xml"
model = mj.MjModel.from_xml_path(right_path)  # MuJoCo model
data = mj.MjData(model)  # MuJoCo data

initial_qpos = {
    'l_j1': 0,
    'l_j2': 0,
    'l_j3': 0,
    'l_j4': 0,
    'l_j5': 0,
    'l_j6': 0,
}
Joint3 = []
Joint3_ref = []
RbtMj = func_mj.Robot(data, model, 'grasping_frame', 'screw_frame')
joints = np.loadtxt("joint_value.txt")
# print(joints)
item = 0
prev_time = 0
# def controller(model, data):
#     global item, prev_time, joints
#     # print(joints)
#     # Joint = 1 + data.time * 0.1
#     # initial_qpos = {
#     #     'l_j1': 1,
#     #     'l_j2': -0,
#     #     'l_j3': 2,
#     #     'l_j4': 0,
#     #     'l_j5': 1.6,
#     #     'l_j6': 0,
#     # }
#     # for item in range(joints.shape[0]):
#     if joints is not None:
#         if data.time > prev_time and item < joints.shape[0]:
#             print(item)
#             print(joints[item,:])
#             data.qpos[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j1')] = joints[item,0]
#             data.qpos[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j2')] = joints[item,1]
#             data.qpos[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j3')] = joints[item,2]
#             data.qpos[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j4')] = joints[item,3]
#             data.qpos[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j5')] = joints[item,4]
#             data.qpos[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j6')] = joints[item,5]
#             item = item + 1
#             prev_time = data.time
#         if item == joints.shape[0] - 1:
#             item = 0
#             prev_time = 0
#             joints = None
#     # # for name, value in initial_qpos.items():
#     # #     data.qpos[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, name)] = value
#     # # Joint3_ref.append(Joint)
#     #
#     id_left_base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j1')
#     left_index = np.arange(id_left_base, id_left_base + 6)
#     id_right_base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'r_j1')
#     right_index = np.arange(id_right_base, id_right_base + 6)
#     data.ctrl[left_index],data.ctrl[right_index]  = RbtMj.coriolis_gravity()
    # put the controller here. This function is called inside the simulation.
    # data.qpos[0] = 1
    # pass

# sim = vi.Mujoco_Viewer(model, data, 20, controller, RbtMj)
# sim.viewer()
# for i in range(1000):
#     sim.step_viewer()
#     time.sleep(0.001)

viewer = mj.viewer.launch_passive(model, data)

for i in range(100):
    if joints is not None:
        for item in range(joints.shape[0]):
            print(item)
            print(joints[item,:])
            data.qpos[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j1')] = joints[item,0]
            data.qpos[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j2')] = joints[item,1]
            data.qpos[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j3')] = joints[item,2]
            data.qpos[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j4')] = joints[item,3]
            data.qpos[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j5')] = joints[item,4]
            data.qpos[mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j6')] = joints[item,5]

            body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_SITE, 'grasping_frame')
            pos = data.xpos[body_id]
            print('pos', pos)

            id_left_base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'l_j1')
            left_index = np.arange(id_left_base, id_left_base + 6)
            id_right_base = mj.mj_name2id(model, mj.mjtObj.mjOBJ_JOINT, 'r_j1')
            right_index = np.arange(id_right_base, id_right_base + 6)
            data.ctrl[left_index], data.ctrl[right_index] = RbtMj.coriolis_gravity()
            mj.mj_step(model, data)
            viewer.sync()
            time.sleep(0.05)














##dm_control 版本，没有解决左右机械臂坐标系对齐问题
# arena = mjcf.RootElement()
# arena.asset.add('texture', name="texplane", type="2d", builtin="checker",
#                 rgb1=".2 .3 .4", rgb2=".1 0.15 0.2", width="256", height="256",
#                 mark="cross", markrgb=".8 .8 .8")
# arena.asset.add('material', name="matplane", texture="texplane",
#                 texrepeat="3 3", texuniform="true")
#
# arena.worldbody.add('geom', name='ground', type='plane', size=[10, 10, 1], material="matplane")
# attachment_site_l = arena.worldbody.add("site", name="left_frame", pos="-1.1 0.025 0", quat="1. 0. 0. 1.")
# attachment_site_r = arena.worldbody.add("site", name="right_frame", pos="0 0 0", quat="1. 0. 0. 1.")
# arena.worldbody.add("light", directional="true", diffuse=".4 .4 .4", specular="0.1 0.1 0.1", pos="0 0 5.0", dir="0 0 -1", castshadow="false")
# arena.worldbody.add("light", directional="true", diffuse=".6 .6 .6", specular="0.2 0.2 0.2", pos="0 0 4", dir="0 0 -1")
# attachment_site_table = arena.worldbody.add("site", name="table_frame", pos="0 0 0", quat="1. 0. 0. 1.")
# left_path = "./dual_robot/left/xml/left_arm.xml"
# right_path = "./dual_robot/right/xml/right_arm.xml"
#
# left_arm = mjcf.from_path(left_path)
# right_arm = mjcf.from_path(right_path)
# # 将child模型附加到parent模型当中，确保附加的根位置和方向符合attachment_site
# #
# # 获取site的位置和方向属性
#
# right_site_pos = attachment_site_r.pos
# right_site_quat = attachment_site_r.quat  # 假设site有quat属性；如果没有，这行将会报错
#
#
# left_site = right_arm.find('site', 'left_arm_site_visual')
# left_body_pos = left_site.pos
# left_body_quat = left_site.quat  # 假设site有quat属性；如果没有，这行将会报错
#
# child_root_left = left_arm.find('body', 'left_base')
# child_root_left.set_attributes(pos=left_body_pos, quat=left_body_quat)
# right_arm.attach(left_arm)
#
# # 把调整好位置和方向的child模型加入到parent中的attachment_site的位置
# arena.worldbody.attach(right_arm)
#
# physics = mjcf.Physics.from_mjcf_model(arena)
# data, model = physics.data.ptr, physics.model.ptr
# model.opt.gravity[:] = [0, 0, 0]
# model.opt.timestep = 0.002