import numpy as np
import matplotlib.pyplot as plt

PixelDist_Meter_Ratio = 10.0
SamPer_sec = 0.1
Acc_Meter = 0
count = 0

# 观测值z已经包含噪声
# z_mat = np.mat(sensor_x)
# print(z_mat)
# 定义x的初始状态
x_mat = np.mat([[0, ], [0, ]])
# x_mat = np.array([[0, ], [0, ]], np.float32)
# 定义初始状态协方差矩阵
p_mat = np.mat([[1, 0], [0, 1]])
# p_mat = np.array([[1, 0], [0, 1]], np.float32)
# 定义状态转移矩阵，因为每秒钟采一次样，所以delta_t = 0.1
f_mat = np.mat([[1, 0.1], [0, 1]])
# f_mat = np.array([[1, 0.1], [0, 1]], np.float32)
# 定义状态转移协方差矩阵，这里我们把协方差设置的很小，因为觉得状态转移矩阵准确度高
q_mat = np.mat([[0.5, 0], [0, 0.00005]])
# q_mat = np.array([[0.5, 0], [0, 0.00005]], np.float32)
# 定义观测矩阵
h_mat = np.mat([1, 0])
# h_mat = np.array([1, 0], np.float32)
# 定义观测噪声协方差
r_mat = np.mat([0.01])
# r_mat = np.array([0.01], np.float32)
# 定义控制矩阵
b_mat = np.mat([[0, ], [0, ]])
# b_mat = np.array([[0, ], [0, ]], np.float32)
# 定义加速度
ut = 0.0

def kalman(z_mat):
    global x_mat,p_mat,f_mat,q_mat,h_mat,r_mat,b_mat,ut,count
    # print(z_mat)
    x_predict = f_mat * x_mat + b_mat * ut
    p_predict = f_mat * p_mat * f_mat.T + q_mat
    kalman = p_predict * h_mat.T / (h_mat * p_predict * h_mat.T + r_mat)
    x_mat = x_predict + kalman * (z_mat - h_mat * x_predict)
    p_mat = (np.identity(2) - kalman * h_mat) * p_predict

    # print("卡尔曼滤波位置：", x_mat[0][0], "检测位置:", z_mat, "差值：", x_mat[0][0] - z_mat)
    count = count +1
    return x_mat[0][0],z_mat,(x_mat[0][0] - z_mat),count

