#! /home/yanji/anaconda3/envs/mujo/bin/python3
# -*- coding: utf-8 -*-

import rosbag
import rospy

def extract_and_save_messages(bag_file, output_txt_file, topic_name):
    with rosbag.Bag(bag_file, 'r') as bag:
        with open(output_txt_file, 'w') as file:
            for topic, msg, t in bag.read_messages(topics=[topic_name]):
                # 假设消息类型是包含 X, Y, Z, MX, MY, MZ 的结构
                file.write(f"{msg.X} {msg.Y} {msg.Z} {msg.MX} {msg.MY} {msg.MZ}\n")

if __name__ == "__main__":
    # 设置bag文件路径和输出txt文件路径
    bag_file = '2024-08-15-15-36-48.bag'  # 替换为你的bag文件路径
    output_txt_file = 'output.txt'  # 你想要保存的txt文件路径
    topic_name = '/robot_force'  # 替换为你的话题名称

    extract_and_save_messages(bag_file, output_txt_file, topic_name)