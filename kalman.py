#吴伟老师写的卡尔曼滤波算法

import cv2
import numpy as np
import matplotlib.pyplot as plt

# sensor_x = 3.56562
sensor_y = 3.56562
PixelDist_Meter_Ratio = 10.0
SamPer_sec = 0.1
Acc_Meter = 0
count = 0

def outputstateFromesti():
    global CurrentStates,EsticurrStates
    CurrentStates = np.array([EsticurrStates[0], EsticurrStates[1]], np.float32)
    # print("CurrentState:",CurrentStates[0])

def EstiFromOutputStates():
    global EsticurrStates,CurrentStates
    EsticurrStates = np.array([[CurrentStates[0]],[CurrentStates[1]],[1]], np.float32)
    # print(EsticurrStates)
def GetAcceleration_Pixel(Acc_Meter):
    global accelaration_pixel
    accelaration_pixel = Acc_Meter * PixelDist_Meter_Ratio

def Velocity_Calculation(v_last,pos_cur,pos_last):
    # print(v_last,pos_cur,pos_last)
    return -(1/7)*v_last+8/(7*SamPer_sec)*(pos_cur-pos_last)

def SystemMatrixUpdate():
    global SystemMatrix
    SystemMatrix = np.array([[1,SamPer_sec,0.5*accelaration_pixel*pow(SamPer_sec,2)],[0,1,accelaration_pixel*SamPer_sec],[0,0,1]], np.float32)

def updatePisiton(current_x):
    global sensor_x
    sensor_x = current_x

def initParameter():
    global EsticurrStates, P_k_matrix, R_0_matrix, Q_0_matrix
    P_k_matrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 0]], np.float32)    # 协方差矩阵
    R_0_matrix = np.array([[0.01, 0, 0], [0, 0.01, 0], [0, 0, 0.01]], np.float32)   # 噪声
    Q_0_matrix = np.array([[0.5, 0, 0], [0, 0.00005, 0], [0, 0, 0]], np.float32)    # 噪声

    EsticurrStates = np.array([[sensor_x], [0], [1]], np.float32)

    outputstateFromesti()
    EstiFromOutputStates()

def Localization(Acc_Meter):
    global Temp,Temp_State,EstiCurrStates, P_k_Matrix,count

    GetAcceleration_Pixel(Acc_Meter)
    SystemMatrixUpdate()
    EstiFromOutputStates()
    Temp =np.zeros((3, 3), np.float32)
    Temp_State = np.zeros((3, 1), np.float32)

    Temp = P_k_matrix + R_0_matrix
    Temp_State = np.array([[sensor_x],[Velocity_Calculation(CurrentStates[1],sensor_x,CurrentStates[0])],[1]], np.float32)

    EstiCurrStates = SystemMatrix * EsticurrStates + SystemMatrix * P_k_matrix * np.linalg.inv(Temp) * (Temp_State - EsticurrStates)    # x
    P_k_Matrix = SystemMatrix * P_k_matrix * (np.identity(3) - np.linalg.inv(Temp) * P_k_matrix)* SystemMatrix.transpose() + Q_0_matrix # p

    outputstateFromesti()
    # print("卡尔曼滤波位置：",EstiCurrStates[0][0],"检测位置:",sensor_x,"差值：",EstiCurrStates[0][0] - sensor_x)
    count = count +1
    return EstiCurrStates[0][0],sensor_x,(EstiCurrStates[0][0] - sensor_x),count


    # print(count)
# for num in range(1,50):
#     initParameter()
#     Localization(Acc_Meter)
#     print(EstiCurrStates[0][0],sensor_x)
