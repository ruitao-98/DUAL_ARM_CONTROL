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

def process_and_plot_ori_adj(ori_adj):
    """
    处理并可视化 ori_adj 数组，计算误差并绘制折线图。
    
    参数:
        ori_adj: List[List[float]] - 输入数组，例如 [[50, 180, 0, ...], ...]
    
    输出:
        - 打印处理后的数组（索引和误差）
        - 显示误差的折线图（横坐标为 item，纵坐标为误差）
    """
    # 初始化结果列表
    processed_data = []
    
    # 处理每个子列表
    for item_idx, sublist in enumerate(ori_adj):
        index = sublist[0]  # 第一个元素，例如 50
        total = sublist[1]  # 第二个元素，例如 180
        
        # 计算角度：(index / total) * 2π
        angle = (index / total) * 2 * np.pi
        
        # 计算误差：角度 - π/2
        error = (angle - (np.pi / 2)) *180 / np.pi
        
        # 存储结果
        processed_data.append([item_idx, error])
    
    # 输出处理后的数组
    print("处理后的数组（[item_idx, error]）：")
    for data in processed_data:
        print(data)
    
    # 转换为 numpy 数组便于绘图和计算
    processed_data = np.array(processed_data)
    item_indices = processed_data[:, 0]  # 横坐标：item_idx
    errors = processed_data[:, 1]        # 纵坐标：误差（弧度）
    
    # 计算平均误差（弧度转换为度）
    mean_error_radians = np.mean(errors)
    mean_error_degrees = mean_error_radians
    print(f"\n平均误差（度）：{mean_error_degrees:.6f}")
    
    # 绘制折线图
    plt.figure(figsize=(10, 6))
    plt.plot(item_indices, errors, marker='o', linestyle='-', color='b')
    plt.xlabel('Item')
    plt.ylabel('Err')
    plt.title('ori_adj Err')
    plt.grid(True)
    plt.legend()
    plt.show()


file_path = "/home/yanji/dual_arm_control/rosbag_record/screwing_revised/Proposed"
# file_path = "/home/yanji/dual_arm_control/rosbag_record/grad_descent"
file_path_save = "/home/yanji/dual_arm_control/rosbag_record/current_obj_trans"


file_name = "santong_5"
# file_name = "m12_5_flat"
file_name_txt = file_name + "_0.bag"
# file_name_txt = "default_suffix_1.bag"
# 设置bag文件路径
# bag_file_path = "/home/yanji/dual_arm_control/rosbag_record/current_0928/base_new_4.bag"
bag_file_path = os.path.join(file_path, file_name_txt)

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
ori_data = []

# 遍历bag中的消息
# for topic, msg, t in bag.read_messages(topics=['/width_p', '/for_pos', '/robot_force']):
# for topic, msg, t in bag.read_messages(topics=['/robot_pose']):
for topic, msg, t in bag.read_messages(topics=['/robot_force', '/robot_pose', '/ori_adj']):
# for topic, msg, t in bag.read_messages(topics=['/width_p', '/current_p']): 
    if topic == '/width_p':
        # 假设消息类型是std_msgs/Float64，存储width
        width_data.append(msg.width)
    
    elif topic == '/ori_adj':
        ori_data.append([msg.phi, msg.point_num, msg.record_item, msg.Rx, msg.Ry, msg.Rz])

    # elif topic == '/current_p':
    #     # 假设消息类型是包含X, Y, Z, MX, MY, MZ的自定义消息，分别提取数据
    #     current_data.append([msg.current])
    
    elif topic == '/for_pos':
        # 假设消息类型是包含X, Y, Z, MX, MY, MZ的自定义消息，分别提取数据
        for_pos_data.append([msg.X, msg.Y, msg.Z])
    
    elif topic == "/robot_pose":
        for_pos_data.append([msg.X, msg.Y, msg.Z, msg.RX, msg.RY, msg.RZ, msg.FX, msg.FY, msg.FZ, msg.theta])
    
    # elif topic == '/robot_force':
    #     # 假设消息类型是包含FX, FY, FZ, X, Y, Z, theta的自定义消息，分别提取数据
    #     robot_force_data.append([msg.X, msg.Y, msg.Z, msg.MX, msg.MY, msg.MZ])

bag.close()

process_and_plot_ori_adj(ori_data)
print(ori_data)

# 创建两个子图
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 8))
x_data = [pos[0] for pos in for_pos_data]  # 提取 X 列
y_data = [pos[1] for pos in for_pos_data]  # 提取 X 列
z_data = [pos[2] for pos in for_pos_data]  # 提取 X 列
m_data = [pos[9] for pos in for_pos_data]  # 提取 X 列
# 在第一个子图绘制 list1
ax1.plot(x_data, linestyle='-', color='g', linewidth = 1)
ax1.plot(y_data, linestyle='-', color='b', linewidth = 1)
ax1.plot(z_data, linestyle='-', color='r', linewidth = 1)

x_data = [pos[3] for pos in for_pos_data]  # 提取 X 列
y_data = [pos[4] for pos in for_pos_data]  # 提取 X 列
z_data = [pos[5] for pos in for_pos_data]  # 提取 X 列
# print(for_pos_data)
# for_pos_data = np.array(for_pos_data)
# 在第二个子图绘制 list2
ax2.plot(x_data, linestyle='-', color='g', linewidth = 1)
ax2.plot(y_data, linestyle='-', color='b', linewidth = 1)
ax2.plot(z_data, linestyle='-', color='r', linewidth = 1)

x_data = [pos[6] for pos in for_pos_data]  # 提取 X 列
y_data = [pos[7] for pos in for_pos_data]  # 提取 X 列
z_data = [pos[8] for pos in for_pos_data]  # 提取 X 列
# print(for_pos_data)
# for_pos_data = np.array(for_pos_data)
# 在第二个子图绘制 list2
ax3.plot(x_data, linestyle='-', color='g', linewidth = 1)
ax3.plot(y_data, linestyle='-', color='b', linewidth = 1)
ax3.plot(z_data, linestyle='-', color='r', linewidth = 1)
# print(len(width_data))
# print(len(current_data))

# 显示图形
plt.tight_layout()
plt.show()

# save_name_width = file_name + "_width.txt"
# save_name_current = file_name + "_current.txt"
# save_name_width = os.path.join(file_path_save, file_name + "_width.txt")
# save_name_current = os.path.join(file_path_save, file_name + "_current.txt")
# width_data_np = np.array(width_data)
# current_data_np = np.array(current_data)
# np.savetxt(save_name_width, width_data_np, delimiter="\n")
# np.savetxt(save_name_current, current_data_np, delimiter="\n")


# print("\nFor Pos Data (/robot_force):")
# for data in robot_force_data:
#     print(f"X: {data[0]}, Y: {data[1]}, Z: {data[2]}, MX: {data[3]}, MY: {data[4]}, MZ: {data[5]}")

# print("\nRobot Force Data (/for_pos):")
# for data in for_pos_data:
#     print(f"X: {data[0]}, Y: {data[1]}, Z: {data[2]}, rx: {data[3]}, ry: {data[4]}, rz: {data[5]}")


