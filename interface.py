import numpy as np
import cv2
import imutils
import tkinter as tk
import tkinter.messagebox
import matplotlib.pyplot as plt
from PIL import Image, ImageTk  # Python Imaging Library
import time
import kalman
import SerialandAngle
from math import *

# 支持中文
plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号

# 初始化，平衡平板
SerialandAngle.Angle2SerPort(0.4, -0.4)

# vs = cv2.VideoCapture('BlueBal.avi')
vs = cv2.VideoCapture(1, cv2.CAP_DSHOW)
vs.set(3, 640)
vs.set(4, 480)
print("Open camera succeed.")

getPixelColor = False  # flag to get the pixel color of the ball when needed
camHeight = 640
camWidth = 480
H, S, V = 0, 0, 0  # the color properties of the Pixel to track
mouseX, mouseY = 0, 0  # declare variables to capture mouse position for color tracking

# 展示了系统控制面板
controllerWindow = tk.Tk()  # initializes this tk interpreter and creates the root window
controllerWindow.title("Ball Plate Control Window ")  # define title of the root window
controllerWindow.geometry("800x640")  # define size of the root window
controllerWindow["bg"] = "lightgrey"  # define the background color of the root window
controllerWindow.resizable(0, 0)  # define if the root window is resizable or not for Vertical and horizontal，不可以拉伸

# 主控制面板背景图片
canvas = tk.Canvas(controllerWindow, width=380, height=92, bd=0, highlightthickness=0)
image_file = tk.PhotoImage(file='C:\\Users\\50578\\working\\Ball-Tracking\\background-en.gif')
image = canvas.create_image(0, 0, anchor='nw', image=image_file)  # 放置着张图片
canvas.place(x=410, y=10)

# 展示了实时画面的控制面板
videoWindow = tk.Toplevel(controllerWindow)  # a new window derived from the root window "controllerwindow"
videoWindow.title("Live Camera")  # define title of videowindow
videoWindow.resizable(0, 0)  # Cannot resize the window
lmain = tk.Label(videoWindow)  # create an empty label widget in the videoWindow
lmain.pack()  # adjust the size of the videowindow to fit the label lmain
videoWindow.withdraw()  # hide the window

# 展示PID控制下输入输出的跟踪情况
graphWindow = tk.Toplevel(controllerWindow)  # a new window derived from the root window "graphwindow"
graphWindow.title("Position in function of time")  # define window title
graphWindow.resizable(0, 0)  # define if resizable or not
graphCanvas = tk.Canvas(graphWindow, width=500 + 200, height=515)  # create a canvas widget in graphwindow
graphCanvas.pack()  # pack the canvas widget
graphWindow.withdraw()  # hide the graphwindow

# 声明一些滑条将用到的变量
sliderHDefault = 0
sliderSDefault = 0
sliderVDefault = 0
sliderCoefPDefault = 0.028
sliderCoefIDefault = 0
sliderCoefDDefault = 0.022
sliderRadiusDefault = 20
sliderSpeedDefault = 10
sliderRefXDefault = 190
sliderRefYDefault = 190

pointsListCircle = []  # create an empty list to put points refinates that describes a circle patern


def createPointsListCircle():  # create an array of 360 points to describe a whole circle with the argument as radius
    global pointsListCircle
    pointsListCircle = []
    for angle in range(0, 360):
        angle = angle - 90
        pointsListCircle.append(
            [sliderRadius.get() * cos(radians(angle)) + 170, sliderRadius.get() * sin(radians(angle)) + 180])
    # print(pointsListCircle)


drawCircleBool = False  # flag to draw Circle


def startDrawCircle():
    # function triggered by Circle pattern Button as a Toggle
    createPointsListCircle()
    global drawCircleBool, drawEightBool, refX, refY
    if drawCircleBool == False:
        drawCircleBool = True
        BballDrawCircle["text"] = "Disable Circle Trajectory"
    else:
        drawCircleBool = False
        refX, refY = 170, 180
        # sliderCoefP.set(sliderCoefPDefault)
        BballDrawCircle["text"] = "Enable Circle Trajectory"
    resetPID()


# 球的运动轨迹是圆还是椭圆，一共分为三步
# 第一步：创建圆与椭圆的运动点列表
# 第二步：编写按键控制的bool逻辑
# 第三步：判断bool逻辑，是否要进入圆与椭圆运行状态。
pointCounter = 0  # a counter that will cover the whole 360 points in case of draw circle or eight


def drawWithBall():  # function triggered after the startDrawCircle or startDrawEight
    global pointCounter, refX, refY, pointsListCircle
    if drawCircleBool == True:
        if pointCounter >= len(pointsListCircle):
            pointCounter = 0
        point = pointsListCircle[pointCounter]
        refX, refY = point[0], point[1]
        pointCounter += sliderSpeed.get()


useKalmanBool = False


def UseKalmanJudge():  # function to judge weather use kalman filter or not.
    global useKalmanBool
    if useKalmanBool == False:
        useKalmanBool = True
        Bkalman["text"] = "Kalman Filter ON"
    else:
        useKalmanBool = False
        Bkalman["text"] = "Kalman Filter OFF"


# 验证小球运动的轨道
showCalqueCalibrationBool = False


def showCalqueCalibration():
    global showCalqueCalibrationBool
    showCalqueCalibrationBool = not showCalqueCalibrationBool


def setRefWithButton():  # set refX and refY based on the mousePosition, mousePosition is the realtime position of the mouse not a saved variable
    global refX, refY
    refX, refY = sliderRefX.get(), sliderRefY.get()
    resetPID()


# 取色笔，可以获得像素点的HSV的值
def getMouseClickPosition(mousePosition):  # get mouse click position
    global mouseX, mouseY
    global getPixelColor
    mouseX, mouseY = mousePosition.x, mousePosition.y
    getPixelColor = True


# 用鼠标设定小球的运动位置，为其设定参考点
refX = 160  # reference refinate X
refY = 180  # reference refinate Y


def setRefWithMouse(mousePosition):
    global refX, refY
    if mousePosition.y > 10:
        refreshGraph()
        refX, refY = mousePosition.x, mousePosition.y


# 左上角，实时视频窗口，控制是不是要显示
showVideoWindow = False


def showCameraFrameWindow():  # function to toggle the showVideoWindow and change the label text of the button
    global showVideoWindow, showGraph
    global BShowVideoTxt
    if showVideoWindow == False:
        # if showGraph == True:
        #    graphWindow.withdraw()
        #    showGraph = False
        #    BShowGraph["text"] = "Show Plot"
        videoWindow.deiconify()
        showVideoWindow = True
        BShowVideo["text"] = "Hide Live CAM"
    else:
        videoWindow.withdraw()
        showVideoWindow = False
        BShowVideo["text"] = "Show Live CAM"


# 控制是否显示建模图。建模图显示了给定与输入输出之间的实时关系。
showGraph = False  # bool for Graph window


def showGraphWindow():  # function that toggles the Graph window and update the show graph button
    global showGraph, showVideoWindow
    global BShowGraph

    if showGraph == False:
        # if showVideoWindow == True:
        #    videoWindow.withdraw()
        #    showVideoWindow = False
        #    BShowVideo["text"] = "Show Live CAM feed"
        showGraph = True
        BShowGraph["text"] = "Hide Plot"
    else:
        showGraph = False
        BShowGraph["text"] = "Show Plot"


t = 500  # time variable for the plotting and initialize at 480 for a good visualization
Ts = 0


def paintGraph():  # function to plot in realtime the graphWindow
    global t, refX, refY, x, y, prevX, Ts, prevY, alpha, prevAlpha
    global showGraphPositionX, showGraphPositionY, showGraphAlpha
    if showGraph == True:
        t += Ts * 100
        graphWindow.deiconify()  # 显示画面
        if showGraphPositionX.get() == 1:
            graphCanvas.create_line(t - Ts * 100, prevX, t, x, fill="#b20000", width=2)
            graphCanvas.create_line(t - Ts * 100, prevRefX, t, refX, fill="#ff7777", width=2)

        if showGraphPositionY.get() == 1:
            graphCanvas.create_line(t - Ts * 100, prevY, t, y, fill="#0069b5", width=2)
            graphCanvas.create_line(t - Ts * 100, prevRefY, t, refY, fill="#6f91f7", width=2)
        if showGraphAlpha.get() == 1:
            graphCanvas.create_line(t - Ts * 100, 240 - prevAlpha * 3, t, 240 - alpha * 3, fill="#8f0caf", width=2)
        if t >= 500:
            t = 0
            graphCanvas.delete("all")

            for i in range(4):
                graphCanvas.create_line(0, 120 * (i + 1), 500, 120 * (i + 1), fill="black", width=1)
                graphCanvas.create_line(100 * (i + 1), 0, 100 * (i + 1), 480, fill="black", width=1)

            graphCanvas.create_line(3, 3, 500, 3, fill="black", width=3)
            graphCanvas.create_line(3, 500, 500, 500, fill="black", width=3)
            graphCanvas.create_line(3, 3, 3, 500, fill="black", width=3)
            graphCanvas.create_line(500, 3, 500, 500, fill="black", width=3)
            graphCanvas.create_line(550, 32, 740, 32, fill="#b20000", width=5)
            graphCanvas.create_line(550, 53, 740, 53, fill="#0069b5", width=5)
            graphCanvas.create_line(550, 73, 740, 73, fill="#8f0caf", width=5)

    else:
        graphWindow.withdraw()


def refreshGraph():  # function that reset the time variable to 480 if the graph is full
    global t
    t = 480


# 更新圆形的路线
def radiusUpdate(self):
    createPointsListCircle()


def resetPID():  # function that compact the plate
    global totalErrorX, totalErrorY, prevErrorX, prevErrorY, prevIntegX, prevIntegY, prevDerivX, prevDerivY
    totalErrorX = 0
    totalErrorY = 0
    prevErrorX = 0
    prevErrorY = 0
    prevIntegX = 0
    prevIntegY = 0
    prevDerivX = 0
    prevDerivY = 0


# PID控制程序
totalErrorX = 0
totalErrorY = 0

alpha, beta, prevAlpha, prevBeta = 0, 0, 0, 0

prevDerivX = 0  # previous derivative
prevDerivY = 0  # previous derivative
prevIntegX = 0
prevIntegY = 0
prevErrorX = 0
prevErrorY = 0

prevBallPosX = 0
prevBallPosY = 0

delivery_time = 0

Ix, Iy = 0, 0
vec_lastx = 0
vec_lasty = 0
ballx = []
bally = []
vecxlist = []
vecylist = []
vecxFilterlist = []
vecyFilterlist = []
timelist = []
refxdraw = []


def PIDcontrol(ballPosX, ballPosY, refX, refY):
    global totalErrorX, totalErrorY
    global alpha, beta, prevAlpha, prevBeta
    global Ts, delivery_time
    global prevDerivX, prevDerivY, prevIntegX, prevIntegY
    global prevErrorX, prevErrorY, Ix, Iy, prevY, prevX, vec_lastx, vec_lasty
    global vecxlist, vecylist, timelist, vecxFilterlist, vecyFilterlist
    global ballx, bally, refxdraw

    Kp = sliderCoefP.get()
    Ki = sliderCoefI.get()
    Kd = sliderCoefD.get()

    Ts = time.time() - delivery_time  # sampling time
    delivery_time = time.time()

    errorX = refX - ballPosX
    errorY = refY - ballPosY
    # print('curPosx:', ballPosX, 'curPosy:',ballPosY)

    # 使用PD控制器，I这一项完全为零
    Cix = Ki * totalErrorX  # prevIntegX + errorX*Ki*Ts                    #Ki * totalErrorX
    Ciy = Ki * totalErrorY  # prevIntegY + errorY*Ki*Ts                    #Ki * totalErrorX

    Cdx = Kd * (0.25 * vec_lastx + 0.75 * ((prevX - ballPosX) / Ts))
    Cdy = Kd * (0.25 * vec_lasty + 0.75 * ((prevY - ballPosY) / Ts))

    # 绘制速度曲线
    print(time.time())
    timelist.append(time.time())
    # vecxlist.append(((prevX - ballPosX) / Ts))
    # vecylist.append(((prevY - ballPosY) / Ts))
    # vecxFilterlist.append((0.25 * vec_lastx + 0.75 * ((prevX - ballPosX) / Ts)))
    # vecyFilterlist.append((0.25 * vec_lasty + 0.75 * ((prevY - ballPosY) / Ts)))

    # 绘制小球运动路线轨迹
    ballx.append(ballPosX)
    bally.append(ballPosY)
    refxdraw.append(refX)

    vec_lastx = (prevX - ballPosX) / Ts
    vec_lasty = (prevY - ballPosY) / Ts

    # print('vec:', ((prevX - ballPosX) / Ts), ((prevY - ballPosY) / Ts))

    Ix = Kp * errorX + Cix + Cdx
    Iy = Kp * errorY + Ciy + Cdy
    # print('errorX:', errorX,'X:', 'P:', Kp * errorX, 'I:', Cix, 'D:', Cdx, 'outputX:', Ix)
    # print('errorY:', errorY,'Y:', 'P:', Kp * errorY, 'I:', Ciy, 'D:', Cdy, 'outputY:', Iy,'\n')

    if (Ix < 0 and Iy < 0) or (Ix > 0 and Iy > 0):
        SerialandAngle.Angle2SerPort(-Ix + 0.4, -Iy - 0.4)
    else:
        SerialandAngle.Angle2SerPort(Ix + 0.4, Iy - 0.4)

    prevDerivX = Cdx
    prevDerivY = Cdy
    prevIntegX = Cix
    prevIntegY = Ciy
    prevErrorX = errorX
    prevErrorY = errorY

    prevX = ballPosX
    prevY = ballPosY

    return Ix, Iy


def drawVecPlot():
    global vecxlist, vecylist, timelist, vecxFilterlist, vecyFilterlist
    global ballx, bally, refxdraw

    plt.xlabel("x/像素点")  # x轴上的名字
    plt.ylabel("y/像素点")  # y轴上的名字

    # plt.plot(timelist, vecxlist,linestyle='--',label='没有滤波的速度曲线')
    # plt.plot(timelist, vecxFilterlist,label='加入滤波算法后的速度曲线')

    plt.plot(ballx, bally,label='小球运动轨迹')

    # plt.plot(timelist, ballx, linestyle='--', label='随时间变化x轴运动轨迹')
    # plt.plot(timelist, refxdraw, label='参考轨迹曲线')

    plt.legend()
    plt.show()

# 退出interface
def endProgam():
    drawVecPlot()
    SerialandAngle.ser.close()
    controllerWindow.destroy()


# function that does nothing, may be used for delay
def donothing():
    pass


prevX, prevY = 0, 0
prevRefX, prevRefY = 0, 0


def main():
    global H, S, V
    global getPixelColor
    global refX, refY, totalErrorX, totalErrorY
    global x, y, alpha, beta, Ix, Iy
    global prevX, prevY, prevAlpha, prevBeta, prevRefX, prevRefY
    global camWidth, camHeight

    _, frame = vs.read()
    frame = frame[70:420, 160:480]

    # print(camHeight,((camWidth - camHeight) / 2),(camWidth - ((camWidth - camHeight) / 2)))
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # 相当于取色笔
    if getPixelColor == True and mouseY > 0 and mouseY < 720 and mouseX < 720:
        pixelColorOnClick = frame[mouseY, mouseX]
        pixelColorOnClick = np.uint8([[pixelColorOnClick]])
        pixelColorOnClick = cv2.cvtColor(pixelColorOnClick, cv2.COLOR_BGR2HSV)
        H = pixelColorOnClick[0, 0, 0]
        S = pixelColorOnClick[0, 0, 1]
        V = pixelColorOnClick[0, 0, 2]
        print(mouseX, mouseY)
        # print(H, S, V)
        getPixelColor = False

    # 银质绿面小球
    greenLower = (43 - sliderLH.get(), 30 - sliderLS.get(), 140 - sliderLV.get())
    greenUpper = (105 + sliderUH.get(), 160 + sliderUS.get(), 255 + sliderUV.get())

    mask_before = cv2.inRange(hsv, greenLower, greenUpper)
    mask_erode = cv2.erode(mask_before, None, iterations=2)
    mask = cv2.dilate(mask_erode, None, iterations=2)
    # cv2.imshow('mask_before', mask_before)
    # cv2.imshow('mask_erode', mask_erode)
    # cv2.imshow('mask', mask)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    cv2.circle(frame, (int(refX), int(refY)), int(4), (255, 0, 0), 2)
    if showCalqueCalibrationBool == True:
        cv2.circle(frame, (240, 240), 220, (255, 0, 0), 2)
        cv2.circle(frame, (240, 240), 160, (255, 0, 0), 2)
        cv2.line(frame, (240, 240), (240, 240 + 160), (255, 0, 0), 2)
        cv2.line(frame, (240, 240), (240 + 138, 240 - 80), (255, 0, 0), 2)
        cv2.line(frame, (240, 240), (240 - 138, 240 - 80), (255, 0, 0), 2)

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        center_float = (M["m10"] / M["m00"], M["m01"] / M["m00"])
        # 添加一步滤波，因为当接收到的数据，在1~2个像素点的误差内剧烈抖动时，会对系统的控制输出造成很大的影响。
        a = x
        d = y
        if radius > 10:
            cv2.putText(frame, str(int(x)) + ";" + str(int(y)).format(0, 0), (int(x) - 50, int(y) - 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            # if useKalmanBool == False:
            # 这里其实是kalman的两种算法
            #     a, b, c, d = kalman_new.kalman(np.mat(center_float[0]))
            #     d, e, f, g = kalman_new.kalman(np.mat(center_float[1]))
            #     a, b = kalman.updatePisiton(x, Ix)
            #     d, e = kalman.updatePisiton(y, Iy)

            # PIDcontrol(int(a), int(d), refX, refY)
        else:
            totalErrorX, totalErrorY = 0, 0
    else:
        SerialandAngle.Angle2SerPort(0.4, -0.4)

    if showVideoWindow == True:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # 转换颜色从BGR到RGB
        frame = Image.fromarray(frame)  # 将图像转换成Image对象
        imgtk = ImageTk.PhotoImage(image=frame)
        lmain.imgtk = imgtk
        lmain.configure(image=imgtk)

    # 延迟20ms之后，进入主程序，从而形成循环。
    lmain.after(20, main)
    # 查询是否要画圆和椭圆
    drawWithBall()
    paintGraph()

    prevRefX, prevRefY = refX, refY
    prevAlpha = alpha
    prevBeta = beta


'''
可以控制并改变HSV的上下值范围。
*************
here  ** 
      **
*************
      **
      **
*************
'''
FrameVideoControl = tk.LabelFrame(controllerWindow, text="Identify Control Panel")
FrameVideoControl.place(x=10, y=10, width=390, height=550)
EmptyLabel = tk.Label(FrameVideoControl)
EmptyLabel.pack()
BShowVideo = tk.Button(FrameVideoControl, text="Show Live CAM", command=showCameraFrameWindow)
BShowVideo.place(x=120, y=0)
BPositionCalibration = tk.Button(FrameVideoControl, text="Toggle Calibration View", command=showCalqueCalibration)
BPositionCalibration.place(x=230, y=0)

sliderUH = tk.Scale(FrameVideoControl, from_=-50, to=50, orient="horizontal", label="upper H", length=350,
                    tickinterval=10)
sliderUH.set(sliderHDefault)
sliderUH.place(x=15, y=30)
sliderUS = tk.Scale(FrameVideoControl, from_=-50, to=50, orient="horizontal", label="upper S", length=350,
                    tickinterval=10)
sliderUS.set(sliderSDefault)
sliderUS.place(x=15, y=107)
sliderUV = tk.Scale(FrameVideoControl, from_=-50, to=50, orient="horizontal", label="upper V", length=350,
                    tickinterval=10)
sliderUV.set(sliderVDefault)
sliderUV.place(x=15, y=184)

sliderLH = tk.Scale(FrameVideoControl, from_=-50, to=50, orient="horizontal", label="lower H", length=350,
                    tickinterval=10)
sliderLH.set(sliderHDefault)
sliderLH.place(x=15, y=261)
sliderLS = tk.Scale(FrameVideoControl, from_=-50, to=50, orient="horizontal", label="lower S", length=350,
                    tickinterval=10)
sliderLS.set(sliderSDefault)
sliderLS.place(x=15, y=338)
sliderLV = tk.Scale(FrameVideoControl, from_=-50, to=50, orient="horizontal", label="lower V", length=350,
                    tickinterval=10)
sliderLV.set(sliderVDefault)
sliderLV.place(x=15, y=415)
'''
系统控制，实现退出，连接等功能。
*************
      ** 
      **
*************
 here **
      **
*************
'''
FrameServosControl = tk.LabelFrame(controllerWindow, text="System Control")
FrameServosControl.place(x=10, y=570, width=390, height=63)
BQuit = tk.Button(FrameServosControl, text="Program Quit", command=endProgam)
BQuit.place(x=75, y=5)
# BQuit.pack()
Bkalman = tk.Button(FrameServosControl, text="Kalman Filter On", command=UseKalmanJudge)
Bkalman.place(x=185, y=5)

'''
控制PID参数
*************
      ** here
      **
*************
      **
      **
*************
'''

FramePIDCoef = tk.LabelFrame(controllerWindow, text="PID coefficients")
FramePIDCoef.place(x=410, y=112, width=380, height=310)
BShowGraph = tk.Button(FramePIDCoef, text="Plot on Graph", command=showGraphWindow)
BShowGraph.pack()
sliderCoefP = tk.Scale(FramePIDCoef, from_=0, to=0.1, orient="horizontal", label="P", length=350, tickinterval=0.01,
                       resolution=0.001)
sliderCoefP.set(sliderCoefPDefault)
sliderCoefP.pack()
sliderCoefI = tk.Scale(FramePIDCoef, from_=0, to=0.1, orient="horizontal", label="I", length=350, tickinterval=0.01,
                       resolution=0.001)
sliderCoefI.set(sliderCoefIDefault)
sliderCoefI.pack()
sliderCoefD = tk.Scale(FramePIDCoef, from_=0, to=0.1, orient="horizontal", label="D", length=350, tickinterval=0.01,
                       resolution=0.001)
sliderCoefD.set(sliderCoefDDefault)
sliderCoefD.pack()
'''
这一块是小球轨迹的控制程序，可以实现小球稳定在球面中心，画圆形，画8字型
*************
      ** 
      **
*************
      ** here
      **
*************
'''
FrameBallControl = tk.LabelFrame(controllerWindow, text="Ball Control")
FrameBallControl.place(x=410, y=432, width=380, height=200)

sliderRadius = tk.Scale(FrameBallControl, from_=0, to=300, orient="horizontal", label="Radius", length=350,
                        tickinterval=50, resolution=1, command=radiusUpdate)
sliderRadius.set(sliderRadiusDefault)
sliderRadius.pack()
sliderSpeed = tk.Scale(FrameBallControl, from_=0, to=20, orient="horizontal", label="Speed", length=350, tickinterval=1,
                       resolution=1)
sliderSpeed.set(sliderSpeedDefault)
sliderSpeed.pack()
BballDrawCircle = tk.Button(FrameBallControl, text="Enable Circle Trajectory", command=startDrawCircle)
BballDrawCircle.place(x=215, y=0)

showGraphPositionX = tk.IntVar()
showGraphPositionX.set(1)
CheckbuttonPositionX = tk.Checkbutton(graphWindow, text="X Position", variable=showGraphPositionX, command=refreshGraph)
CheckbuttonPositionX.place(x=520, y=20)
showGraphPositionY = tk.IntVar()
showGraphPositionY.set(1)
CheckbuttonPositionY = tk.Checkbutton(graphWindow, text="Y Position", variable=showGraphPositionY, command=refreshGraph)
CheckbuttonPositionY.place(x=520, y=40)
showGraphAlpha = tk.IntVar()
CheckbuttonAlpha = tk.Checkbutton(graphWindow, text="Plate Inclination", variable=showGraphAlpha, command=refreshGraph)
CheckbuttonAlpha.place(x=520, y=60)
sliderRefX = tk.Scale(graphWindow, from_=0, to=480, orient="horizontal", label="RefX", length=150,
                      tickinterval=100)
sliderRefX.set(sliderRefXDefault)
sliderRefX.place(x=520, y=100)
sliderRefY = tk.Scale(graphWindow, from_=0, to=480, orient="horizontal", label="RefY", length=150,
                      tickinterval=100)
sliderRefY.set(sliderRefYDefault)
sliderRefY.place(x=520, y=200)
BsetReference = tk.Button(graphWindow, text="Set Ref", command=setRefWithButton)
BsetReference.place(x=520, y=350)
# Blog = tk.Button(graphWindow, text="Write Log", command=startLog)
# Blog.place(x=520, y=370)

# 获取鼠标的位置，转换成像素值。
# https://www.cnblogs.com/progor/p/8505599.html
videoWindow.protocol("WM_DELETE_WINDOW", donothing)
videoWindow.bind("<Button-3>", getMouseClickPosition)
videoWindow.bind("<Button-1>", setRefWithMouse)  # mouse click to set reference position

main()
tk.mainloop()
