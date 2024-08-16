#!/home/yanji/anaconda3/envs/mujo/bin/python3
import matplotlib.pyplot as plt
import rospy
import numpy as np
from real_robot_control.msg import orientation_pub
from real_robot_control.msg import robot_pos_pub
from real_robot_control.msg import force_pos_pub

class cal_pos:
    def __init__(self):
        rospy.init_node('orientation_cal')
        self.theta = 0
        self.FZ = 0
        self.pub = rospy.Publisher('orientation_phi', orientation_pub, queue_size=100)
        self.orien = orientation_pub()

    def callback(self, data):
        self.theta = data.theta
        self.FZ = data.FZ
        
    def listener(self):
        rospy.Subscriber('for_pos', force_pos_pub, self.callback, queue_size=10)
    
    def main(self):
        FZ_array = []
        theta_array = []
        old_theta = 0
        circle_number = 0 # 记录圈数
        rate = rospy.Rate(500) 
        is_periodic = 0
        done = 0
        old_circle_number = 0
        print('start')
        while not rospy.is_shutdown():

            if old_theta != self.theta: # 只有更新了才会被加入
                # print(self.FZ)
                FZ_array.append(self.FZ)
                theta_array.append(self.theta)
                old_theta = self.theta
                

                if (len(theta_array) % 260 == 0):
                    print(len(theta_array))
                    circle_number = circle_number + 1

                    if not is_periodic: #只计算一次，后续都不计算了
                        if circle_number > 6:
                            is_periodic = self.analyze_signal_with_autocorrelation(FZ_array[-260 * 6:])  # 计算是否出现周期性
                        else:
                            is_periodic = self.analyze_signal_with_autocorrelation(FZ_array)  # 计算是否出现周期性 

                    if is_periodic:
                        temp_array = FZ_array[-260-1:]  #提取出最近的一个周期
                        # 找到列表中的最小值
                        min_value = min(temp_array)
                        max_value = max(temp_array)
                        if done == 0: # 只在第一次发布使用
                            if max_value - min_value > 5: #卡一个阈值
                                print( "is_periodic=", is_periodic)
                                # 找到最小值的索引
                                min_index = temp_array.index(min_value)
                                print(min_index)
                                self.orien.phi = min_index
                                self.pub.publish(self.orien) #将产生的偏转角度发布出去
                                old_circle_number = circle_number
                                done = 1

                        if circle_number == old_circle_number + 2:
                            print("max=", max_value, "min=", min_value)
                            if max_value - min_value > 5: #卡一个阈值
                                print( "is_periodic=", is_periodic)
                                # 找到最小值的索引
                                min_index = temp_array.index(min_value)
                                print(min_index)
                                self.orien.phi = min_index
                                self.pub.publish(self.orien) #将产生的偏转角度发布出去
                                old_circle_number = circle_number
                        
            rate.sleep()
            


    def autocorrelation(self, signal):
        """
        计算信号的自相关函数
        """
        n = len(signal)
        result = np.correlate(signal, signal, mode='full')
        return result[result.size // 2:]

    def analyze_signal_with_autocorrelation(self, signal, threshold=0.3):
        """
        分析信号的周期性（自相关分析）
        参数:
        - signal: 时序信号
        - threshold: 判断周期性的阈值
        
        返回:
        - is_periodic: 是否具有周期性（1表示周期性，0表示非周期性）
        """
        # 计算自相关函数
        autocorr = self.autocorrelation(signal)
        
        # 归一化自相关函数
        autocorr /= autocorr[0]
        
        # 找到自相关函数中的峰值
        peaks = np.where((autocorr[1:] > threshold) & (autocorr[1:] > np.roll(autocorr, 1)[1:]) & (autocorr[1:] > np.roll(autocorr, -1)[1:]))[0] + 1
        print(peaks)
        # 判断是否具有周期性
        if len(peaks) > 0:
            is_periodic = 1
        else:
            is_periodic = 0
        # 绘制自相关图
        # plt.plot(autocorr)
        # plt.title('Autocorrelation')
        # plt.xlabel('Lag')
        # plt.ylabel('Autocorrelation')
        # plt.grid()
        # plt.show()
        return is_periodic

if __name__ == '__main__':
    pos_pub = cal_pos()
    pos_pub.listener()
    pos_pub.main()

