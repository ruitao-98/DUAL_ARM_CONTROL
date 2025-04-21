#! /home/yanji/anaconda3/envs/mujo/bin/python3
# -*- coding: utf-8 -*-

# import rosbag
# import rospy

# def extract_and_save_messages(bag_file, output_txt_file, topic_name):
#     with rosbag.Bag(bag_file, 'r') as bag:
#         with open(output_txt_file, 'w') as file:
#             for topic, msg, t in bag.read_messages(topics=[topic_name]):
#                 # 假设消息类型是包含 X, Y, Z, MX, MY, MZ 的结构
#                 file.write(f"{msg.X} {msg.Y} {msg.Z} {msg.MX} {msg.MY} {msg.MZ}\n")

# if __name__ == "__main__":
#     # 设置bag文件路径和输出txt文件路径
#     bag_file = '2024-08-15-15-36-48.bag'  # 替换为你的bag文件路径
#     output_txt_file = 'output.txt'  # 你想要保存的txt文件路径
#     topic_name = '/robot_force'  # 替换为你的话题名称

#     extract_and_save_messages(bag_file, output_txt_file, topic_name)

import rosbag
import rospy
import os
import matplotlib.pyplot as plt
import numpy as np



# file_path = "/home/yanji/dual_arm_control/rosbag_record/grad_descent"
# file_path_save = "/home/yanji/dual_arm_control/rosbag_record/grad_descent_transferred"

# file_path = "/home/yanji/dual_arm_control/rosbag_record/random_search"
# file_path_save = "/home/yanji/dual_arm_control/rosbag_record/random_search_transferred"

# file_path = "/home/yanji/dual_arm_control/rosbag_record/tips_compare"
# file_path_save = "/home/yanji/dual_arm_control/rosbag_record/tips_compare_transferred"

# file_path = "/home/yanji/dual_arm_control/rosbag_record/screwing"
# file_path_save = "/home/yanji/dual_arm_control/rosbag_record/screwing_trans"

file_path = "/home/yanji/dual_arm_control/rosbag_record/screwing_revised/Method_2"
file_path_save = "/home/yanji/dual_arm_control/rosbag_record/screwing_revised_trans/Method_2"

# 获取文件夹中所有 .bag 文件
bag_files = [f for f in os.listdir(file_path) if f.endswith('.bag')]

# 
for bag_file in bag_files:
    bag_file_path = os.path.join(file_path, bag_file)

    # 检查文件是否存在
    if not os.path.exists(bag_file_path):
        print(f"Bag file {bag_file_path} does not exist!")
        exit()

    # 打开bag文件
    bag = rosbag.Bag(bag_file_path)

    # 初始化字典存储数据
    width_data = []
    for_pos_data = []
    robot_force_data = []
    current_data = []
    pose_force_data = []
    ori_data = []

    # 遍历bag中的消息
    # for topic, msg, t in bag.read_messages(topics=['/for_pos', '/width_p', '/robot_force']):
    # for topic, msg, t in bag.read_messages(topics=['/width_p', '/current_p']): 
    # for topic, msg, t in bag.read_messages(topics=['/robot_pose']):
    for topic, msg, t in bag.read_messages(topics=['/robot_force', '/robot_pose', '/ori_adj']):
        if topic == '/width_p':
            # 假设消息类型是std_msgs/Float64，存储width
            width_data.append(msg.width)

        elif topic == '/ori_adj':
            ori_data.append([msg.phi, msg.point_num, msg.record_item, msg.Rx, msg.Ry, msg.Rz])

        elif topic == '/current_p':
            # 假设消息类型是包含X, Y, Z, MX, MY, MZ的自定义消息，分别提取数据
            current_data.append([msg.current])
        
        elif topic == '/for_pos':
            # 假设消息类型是包含X, Y, Z, MX, MY, MZ的自定义消息，分别提取数据
            for_pos_data.append([msg.X, msg.Y, msg.Z, msg.FX, msg.FY, msg.FZ])
        
        elif topic == '/robot_force':
            # 假设消息类型是包含FX, FY, FZ, X, Y, Z, theta的自定义消息，分别提取数据
            robot_force_data.append([msg.X, msg.Y, msg.Z, msg.MX, msg.MY, msg.MZ])

        elif topic == '/robot_pose':
            # 假设消息类型是包含FX, FY, FZ, X, Y, Z, theta的自定义消息，分别提取数据
            pose_force_data.append([msg.X, msg.Y, msg.Z, msg.RX, msg.RY, msg.RZ, msg.FX, msg.FY, msg.FZ, msg.theta])

    bag.close()

    base_file_name = os.path.splitext(bag_file)[0]  # 去掉.bag扩展名
    save_name_pf = os.path.join(file_path_save, base_file_name + "_pos_force.txt")
    pose_force_data = np.array(pose_force_data)  # 提取前7列
    # 保存为 n*7 的格式
    np.savetxt(save_name_pf, pose_force_data, delimiter=" ", fmt="%.6f")

    save_name_pf = os.path.join(file_path_save, base_file_name + "_ori_adj.txt") #更新save_name
    ori_data = np.array(ori_data)  # 提取前7列
    # 保存为 n*7 的格式
    np.savetxt(save_name_pf, ori_data, delimiter=" ", fmt="%.6f")

    

    # 生成与bag文件同名的文本文件名，但替换扩展名为 _width.txt 和 _current.txt
    # base_file_name = os.path.splitext(bag_file)[0]  # 去掉.bag扩展名
    # save_name_width = os.path.join(file_path_save, base_file_name + "_width.txt")
    # save_name_pos = os.path.join(file_path_save, base_file_name + "_pos.txt")
    # save_name_force = os.path.join(file_path_save, base_file_name + "_force.txt")

    # width_data_np = np.array(width_data)
    # for_pos_data = np.array(for_pos_data)
    # robot_force_data = np.array(robot_force_data)
    # np.savetxt(save_name_width, width_data_np, delimiter=" ", fmt="%.6f")
    # np.savetxt(save_name_pos, for_pos_data, delimiter=" ", fmt="%.6f")
    # np.savetxt(save_name_force, robot_force_data, delimiter=" ", fmt="%.6f")


    # 创建两个子图
    # fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(6, 8))

    # # 在第一个子图绘制 list1
    # ax1.plot(width_data,  linestyle='-',color='b', linewidth = 1)


    # # 在第二个子图绘制 list2
    # ax2.plot(current_data, linestyle='-', color='g', linewidth = 1)
    # print(len(width_data))
    # print(len(current_data))

    # # 显示图形
    # plt.tight_layout()
    # plt.show()




