import matplotlib.pyplot as plt

# 读取txt文件内容
def read_txt_file(file_path):
    with open(file_path, 'r') as file:
        data = file.readlines()
    # 将数据转换为浮点数
    data = [float(line.strip()) for line in data]
    return data

# 定义文件路径
# file_path_x = 'x_true_2024-08-13_23-46-46.txt'
# file_path_y = 'y_true_2024-08-13_23-46-46.txt'
# file_path_x = 'x_2024-08-13_23-46-46.txt'
# file_path_y = 'y_2024-08-13_23-46-46.txt'
file_path_x = 'x_robot_2024-08-13_23-46-46.txt'
file_path_y = 'y_robot_2024-08-13_23-46-46.txt'
# file_path_x = 'x_2024-08-13_23-51-45.txt'
# file_path_y = 'y_2024-08-13_23-51-45.txt'
# file_path_x = 'x_robot_2024-08-13_23-51-45.txt'
# file_path_y = 'y_robot_2024-08-13_23-51-45.txt'

# 读取数据
x_data = read_txt_file(file_path_x)
y_data = read_txt_file(file_path_y)

# 检查数据长度是否匹配
if len(x_data) != len(y_data):
    raise ValueError("The data files must have the same number of elements.")

# 绘制散点图
plt.figure(figsize=(10, 10))
# plt.scatter(x_data, y_data, s=1)  # s=1 设置点的大小
plt.plot(x_data[:3000], y_data[:3000], linewidth=1)  # linewidth=1 设置线的宽度
plt.xlabel('X-axis Data')
plt.ylabel('Y-axis Data')
plt.title('Scatter Plot of X vs Y')
plt.grid(True)
plt.tight_layout()  # 使得图像填满画布

# 显示图像
plt.show()