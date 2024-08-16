import numpy as np
from scipy.signal import find_peaks
from scipy.fft import fft, fftfreq
import matplotlib.pyplot as plt
from scipy.signal import butter, lfilter
# def is_periodic_signal(signal, amplitude_threshold, frequency_threshold):
#     print('------------')
#     # 计算信号的标准差（作为波动幅度的一个衡量）
#     std_dev = np.std(signal)
#     print("std_dev", std_dev)
#     # if std_dev < amplitude_threshold:
#     #     return False
    
#     # 进行傅里叶变换，检测周期性
#     N = len(signal)
#     yf = fft(signal)
#     xf = fftfreq(N, 1)[:N//2]
    
#     # 找到频谱中的峰值
#     peaks, _ = find_peaks(np.abs(yf[:N//2]), height=frequency_threshold)
#     print(peaks)
#     # 如果存在明显的频率峰值，则认为是周期性信号
#     if len(peaks) > 0:
#         return True
#     else:
#         return False

def butter_lowpass(cutoff, fs, order=5):
    """
    设计低通滤波器
    参数:
    - cutoff: 截止频率
    - fs: 采样率
    - order: 滤波器阶数
    
    返回:
    - b, a: 滤波器系数
    """
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

def lowpass_filter(data, cutoff, fs, order=5):
    """
    应用低通滤波器
    参数:
    - data: 输入信号
    - cutoff: 截止频率
    - fs: 采样率
    - order: 滤波器阶数
    
    返回:
    - y: 滤波后的信号
    """
    b, a = butter_lowpass(cutoff, fs, order=order)
    y = lfilter(b, a, data)
    return y

# 读取TXT文件并提取第三列数据
def read_third_column(file_path):
    data = np.loadtxt(file_path)
    third_column = data[:, 2]  # 提取第三列
    return third_column
file_path = 'output.txt'  # 替换为你的数据文件路径
cutoff_freq = 3
sampling_rate = 125
signal = read_third_column(file_path)
filtered_signal = lowpass_filter(signal, cutoff=cutoff_freq, fs=sampling_rate)
x_data = np.arange(0, len(signal))
plt.figure(figsize=(20, 6))
# plt.scatter(x_data, signal, s=1)  # s=1 设置点的大小
plt.plot(x_data, signal, linewidth=2)  # linewidth=1 设置线的宽度
plt.plot(x_data, filtered_signal, linewidth=1)  # linewidth=1 设置线的宽度
plt.xlabel('X-axis Data')
plt.ylabel('Y-axis Data')
plt.title('Scatter Plot of X vs Y')
plt.grid(True)
plt.tight_layout()  # 使得图像填满画布

# 显示图像
plt.show()

def autocorrelation(signal):
    """
    计算信号的自相关函数
    """
    n = len(signal)
    result = np.correlate(signal, signal, mode='full')
    return result[result.size // 2:]

def analyze_signal_with_autocorrelation(signal, threshold=0.5):
    """
    分析信号的周期性（自相关分析）
    参数:
    - signal: 时序信号
    - threshold: 判断周期性的阈值
    
    返回:
    - is_periodic: 是否具有周期性（1表示周期性，0表示非周期性）
    """
    # 计算自相关函数
    autocorr = autocorrelation(signal)
    
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
    plt.plot(autocorr)
    plt.title('Autocorrelation')
    plt.xlabel('Lag')
    plt.ylabel('Autocorrelation')
    plt.grid()
    plt.show()
    
    return is_periodic

# 示例信号
# signal = np.sin(2 * np.pi * np.arange(0, 10, 0.1)) + 0.5 * np.random.normal(size=100)

# 参数设置
# amplitude_threshold = 5
# frequency_threshold = 10  # 根据信号的性质设置适当的频率阈值

# # 判断信号是否具有周期性且波动幅度达到要求
# result = is_periodic_signal(signal, amplitude_threshold, frequency_threshold)
# print(result)


# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.fft import fft, fftfreq

def analyze_discrete_signal(signal, sampling_rate, noise_threshold=1.5):
    """
    分析离散信号的周期性
    参数:
    - signal: 离散信号，n*1向量
    - sampling_rate: 采样率
    - noise_threshold: 噪声阈值，主频幅度与平均噪声水平的比例，默认为1.5
    
    返回:
    - is_periodic: 是否具有周期性（1表示周期性，0表示非周期性）
    """
    N = len(signal)  # 信号长度
    
    # 进行傅里叶变换
    yf = fft(signal)
    xf = fftfreq(N, 1 / sampling_rate)
    
    # 仅取正频率部分
    xf = xf[:N // 2]
    yf = 2.0 / N * np.abs(yf[:N // 2])
    
    # 计算频谱中的主频峰值和平均噪声水平
    peak_amplitude = np.max(yf)
    mean_noise_level = np.mean(yf)
    
    # 判断是否具有周期性
    if peak_amplitude / mean_noise_level > noise_threshold:
        is_periodic = 1
    else:
        is_periodic = 0
    
    # 绘制频谱图
    plt.plot(xf, yf)
    plt.title('Frequency Spectrum')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Amplitude')
    plt.grid()
    plt.show()
    
    return is_periodic

# 示例：离散信号
sampling_rate = 100  # 假设采样率为1000Hz
# signal = np.array([...])  # 你的n*1离散信号向量
# analyze_signal_with_autocorrelation(filtered_signal)
bias = 2000
is_periodic = analyze_signal_with_autocorrelation(signal[0+bias:260*8+bias])
print("Is the signal periodic?", bool(is_periodic))
# 分析信号的周期性
# is_periodic = analyze_discrete_signal(filtered_signal[0000:500]-np.mean(filtered_signal[000:500]), sampling_rate)
# print("Is the signal periodic?", bool(is_periodic))