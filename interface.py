from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import tkinter as tk  # Python GUI
import tkinter.messagebox
from PIL import Image, ImageTk  # Python Imaging Library
import kalman_new
import SerialandAngle
from math import *

# 初始化，平衡平板
SerialandAngle.Angle2SerPort(0, 0)

# vs = cv2.VideoCapture('C:/Users/50578/working/Ball-Tracking/BlueBal.avi')
vs = cv2.VideoCapture(1, cv2.CAP_DSHOW)
vs.set(3, 1280)
vs.set(4, 720)
print("Open camera succeed.")
time.sleep(.2)

getPixelColor = False  # flag to get the pixel color of the ball when needed
camHeight = 1280
camWidth = 720
H, S, V = 0, 0, 0  # the color properties of the Pixel to track
mouseX, mouseY = 0, 0  # declare variables to capture mouse position for color tracking
lostballcount = 0

# 展示了系统控制面板
controllerWindow = tk.Tk()  # initializes this tk interpreter and creates the root window
controllerWindow.title("2DOF Ball-Plate Control Window ")  # define title of the root window
controllerWindow.geometry("1200x800")  # define size of the root window
controllerWindow["bg"] = "lightgrey"  # define the background color of the root window
controllerWindow.resizable(0, 0)  # define if the root window is resizable or not for Vertical and horizontal，不可以拉伸

# 主控制面板背景图片
# canvas = tk.Canvas(controllerWindow, width=1200,height=800,bd=0, highlightthickness=0)
# imgpath = 'background.gif'
# img = Image.open(imgpath)
# photo = ImageTk.PhotoImage(img)
# canvas.create_image(500, 20, image=photo)
# canvas.pack()

# 展示了实时画面的控制面板
videoWindow = tk.Toplevel(controllerWindow)  # a new window derived from the root window "controllerwindow"
videoWindow.title("Cam Footage")  # define title of videowindow
videoWindow.resizable(0, 0)  # Cannot resize the window
lmain = tk.Label(videoWindow)  # create an empty label widget in the videoWindow
lmain.pack()  # adjust the size of the videowindow to fit the label lmain
videoWindow.withdraw()  # hide the window

# 展示PID控制下输入输出的跟踪情况
graphWindow = tk.Toplevel(controllerWindow)  # a new window derived from the root window "graphwindow"
graphWindow.title("Position in function of time")  # define window title
graphWindow.resizable(0, 0)  # define if resizable or not
graphCanvas = tk.Canvas(graphWindow, width=500 + 200, height=camHeight)  # create a canvas widget in graphwindow
graphCanvas.pack()  # pack the canvas widget
graphWindow.withdraw()  # hide the graphwindow

# 声明一些滑条将用到的变量
sliderHDefault = 0
sliderSDefault = 0
sliderVDefault = 0
sliderCoefPDefault = 0.010
sliderCoefIDefault = 0.010
sliderCoefDDefault = 0.010
sliderRadiusDefault = 10
sliderSpeedDefault = 10
sliderRefXDefault = camWidth / 2
sliderRefYDefault = camHeight / 2

pointsListCircle = []  # create an empty list to put points refinates that describes a circle patern


def createPointsListCircle():  # create an array of 360 points to describe a whole circle with the argument as radius
    global pointsListCircle
    pointsListCircle = []
    for angle in range(0, 360):
        angle = angle - 90
        pointsListCircle.append(
            [sliderRadius.get() * cos(radians(angle)) + 240, sliderRadius.get() * sin(radians(angle)) + 240])


pointsListEight = []  # create an empty list to put points refinates that describes an Eight patern


def createPointsListEight():  # create an array of 360 points to describe an Eight shape with the argument as radius
    global pointsListEight
    pointsListEight = []
    for angle in range(270, 270 + 360):
        pointsListEight.append([sliderRadius.get() * cos(radians(angle)) + 240,
                                sliderRadius.get() * sin(radians(angle)) + 240 + sliderRadius.get()])
    for angle in range(360, 0, -1):
        angle = angle + 90
        pointsListEight.append([sliderRadius.get() * cos(radians(angle)) + 240,
                                sliderRadius.get() * sin(radians(angle)) + 240 - sliderRadius.get()])


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
        refX, refY = 240, 240
        # sliderCoefP.set(sliderCoefPDefault)
        BballDrawCircle["text"] = "Enable Circle Trajectory"
    resetPID()


drawEightBool = False


def startDrawEight():  # function triggered by Eight pattern Button as a Toggle
    global drawEightBool, drawCircleBool, refX, refY
    createPointsListEight()
    if drawEightBool == False:
        drawEightBool = True
        BballDrawEight["text"] = "Disable Eight Trajectory"
    else:
        drawEightBool = False
        refX, refY = 240, 240
        # sliderCoefP.set(sliderCoefPDefault)
        BballDrawEight["text"] = "Enable Eight Trajectory"
    resetPID()


# 球的运动轨迹是圆还是椭圆，一共分为三步
# 第一步：创建圆与椭圆的运动点列表
# 第二步：编写按键控制的bool逻辑
# 第三步：判断bool逻辑，是否要进入圆与椭圆运行状态。
pointCounter = 0  # a counter that will cover the whole 360 points in case of draw circle or eight


def drawWithBall():  # function triggered after the startDrawCircle or startDrawEight
    global pointCounter, refX, refY
    if drawCircleBool == True:
        if pointCounter >= len(pointsListCircle):
            pointCounter = 0
            point = pointsListCircle[pointCounter]
        refX, refY = point[0], point[1]
        pointCounter += sliderSpeed.get()
    if drawEightBool == True:
        if pointCounter >= len(pointsListEight):
            pointCounter = 0
        point = pointsListEight[pointCounter]
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


# 取色笔，可以获得像素点的HSV的值
def getMouseClickPosition(mousePosition):  # get mouse click position
    global mouseX, mouseY
    global getPixelColor
    mouseX, mouseY = mousePosition.x, mousePosition.y
    getPixelColor = True


# 用鼠标设定小球的运动位置，为其设定参考点
refY = 360  # reference refinate Y
refX = 360  # reference refinate X


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
        BShowVideo["text"] = "Hide Live CAM feed"
    else:
        videoWindow.withdraw()
        showVideoWindow = False
        BShowVideo["text"] = "Show Live CAM feed"


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


def refreshGraph():  # function that reset the time variable to 480 if the graph is full
    global t
    t = 480


# 更新圆形与8字型曲线的路线
def radiusUpdate(self):
    createPointsListCircle()
    createPointsListEight()


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
timeInterval = 1
alpha, beta, prevAlpha, prevBeta = 0, 0, 0, 0
N = 20  # Derivative Coefficient
prevDerivX = 0  # previous derivative
prevDerivY = 0  # previous derivative
prevIntegX = 0
prevIntegY = 0
delivery_time = 0
prevErrorX = 0
prevErrorY = 0


def PIDcontrol(ballPosX, ballPosY, prevBallPosX, prevBallPosY, refX, refY):
    global totalErrorX, totalErrorY
    global alpha, beta, prevAlpha, prevBeta
    global startBalanceBall, arduinoIsConnected
    global Ts, delivery_time, N
    global prevDerivX, prevDerivY, prevIntegX, prevIntegY
    global prevErrorX, prevErrorY

    Kp = sliderCoefP.get()
    Ki = sliderCoefI.get()
    Kd = sliderCoefD.get()

    Ts = 0.1
    # Ts = time.time() - delivery_time  # sampling time
    # delivery_time = time.time()
    # print(Ts)
    errorX = refX - ballPosX
    errorY = refY - ballPosY
    # print(errorX,errorY)

    try:
        derivX = (prevBallPosX - ballPosX) / Ts
    except ZeroDivisionError:
        derivX = 0

    try:
        derivY = (prevBallPosY - ballPosY) / Ts
    except ZeroDivisionError:
        derivY = 0
    # 使用PD控制器，I这一项完全为零
    Cix = Ki * totalErrorX  # prevIntegX + errorX*Ki*Ts                    #Ki * totalErrorX
    Ciy = Ki * totalErrorY  # prevIntegY + errorY*Ki*Ts                    #Ki * totalErrorX
    # Cix = prevIntegX + errorX * Ki * Ts
    # Ciy = prevIntegY + errorY * Ki * Ts

    Cdx = Ts / (1 + N * Ts) * (
            N * Kd * derivX + prevDerivX / Ts)  # (Kd*N*(errorX-prevErrorX)+prevDerivX)/(1+N*Ts)# #Kd * ((errorX - prevErrorX)/Ts)
    Cdy = Ts / (1 + N * Ts) * (
            N * Kd * derivY + prevDerivY / Ts)  # (Kd*N*(errorY-prevErrorY)+prevDerivY)/(1+N*Ts) # #Kd * ((errorY - prevErrorY)/Ts)
    # Cdx = Kd * ((errorX - prevErrorX)/Ts)
    # Cdy = Kd * ((errorY - prevErrorY)/Ts)

    Ix = Kp * errorX + Cix + Cdx
    Iy = Kp * errorY + Ciy + Cdy
    Ix = round(Ix, 1)
    Iy = round(Iy, 1)

    print(Ix, Iy)
    if (Ix < 0 and Iy < 0) or (Ix > 0 and Iy > 0):
        SerialandAngle.Angle2SerPort(-Ix, -Iy)
    else:
        SerialandAngle.Angle2SerPort(Ix, Iy)

    prevDerivX = Cdx
    prevDerivY = Cdy
    prevIntegX = Cix
    prevIntegY = Ciy
    prevErrorX = errorX
    prevErrorY = errorY
    # print(totalErrorX)


# 退出interface
def endProgam():
    SerialandAngle.ser.close()
    controllerWindow.destroy()


# function that does nothing, may be used for delay
def donothing():
    pass


prevX, prevY = 0, 0
prevRefX, prevRefY = 0, 0
start_time = 0
delivery_time11 = 0


def main():
    global H, S, V
    global getPixelColor
    global refX, refY, totalErrorX, totalErrorY
    global x, y, alpha, beta
    global prevX, prevY, prevAlpha, prevBeta, prevRefX, prevRefY
    global camWidth, camHeight, lostballcount
    global timeInterval, start_time, delivery_time11

    # Ts = time.time() - delivery_time11  # sampling time
    # delivery_time11 = time.time()
    # print(Ts)

    _, frame = vs.read()
    # frame = frame[0:720, 280:1030]
    # frame = frame[10:700, 310:1010]
    frame = frame[40:670, 340:980]
    # frame = imutils.resize(frame, width=600) # 视频验证窗口
    frame = imutils.resize(frame, width=720)

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
        # print(mouseX, mouseY)
        print(H, S, V)
        getPixelColor = False

    # 银质绿面小球
    greenLower = (43 - sliderLH.get(), 30 - sliderLS.get(), 140 - sliderLV.get())
    greenUpper = (105 + sliderUH.get(), 160 + sliderUS.get(), 255 + sliderUV.get())
    # 绿色弹力球
    # greenLower = (25 - sliderLH.get(), 28 - sliderLS.get(), 250 - sliderLV.get())
    # greenUpper = (35 + sliderUH.get(), 60 + sliderUS.get(), 255 + sliderUV.get())

    mask_before = cv2.inRange(hsv, greenLower, greenUpper)
    mask_erode = cv2.erode(mask_before, None, iterations=2)
    mask = cv2.dilate(mask_erode, None, iterations=2)
    # cv2.imshow('mask_before', mask_before)
    # cv2.imshow('mask_erode', mask_erode)
    # cv2.imshow('mask', mask)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    a = cnts
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

        if radius > 10:
            cv2.putText(frame, str(int(x)) + ";" + str(int(y)).format(0, 0), (int(x) - 50, int(y) - 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            if useKalmanBool == False:
                a, b, c, d = kalman_new.kalman(np.mat(center_float[0]))
                d, e, f, g = kalman_new.kalman(np.mat(center_float[1]))
                print("卡尔曼滤波位置：", (int(a), int(d)), "检测位置:", (int(b), int(e)), "差值：",
                      ((int(a) - int(b)), (int(d) - int(e))))

            PIDcontrol(int(a), int(d), prevX, prevY, refX, refY)
        else:
            totalErrorX, totalErrorY = 0, 0
    else:
        # lostballcount = lostballcount + 1
        # print('lost ball:', lostballcount)
        SerialandAngle.Angle2SerPort(0, 0)
    # cv2.imshow('frame', frame)
    if showVideoWindow == True:
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # 转换颜色从BGR到RGB
        frame = Image.fromarray(frame)  # 将图像转换成Image对象
        imgtk = ImageTk.PhotoImage(image=frame)
        lmain.imgtk = imgtk
        lmain.configure(image=imgtk)

    # fps = int(1/timeInterval)
    # a = 0
    # a = a + 1
    # avefps = 0
    # avefps = (avefps + fps)/a
    # print(avefps)

    # 延迟20ms之后，进入主程序，从而形成循环。
    lmain.after(20, main)
    # 查询是否要画圆和椭圆
    drawWithBall()
    try:
        prevX, prevY = int(x), int(y)
    except:
        pass
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
FrameVideoControl = tk.LabelFrame(controllerWindow, text="Video Control")
FrameVideoControl.place(x=20, y=20, width=390)
EmptyLabel = tk.Label(FrameVideoControl)
EmptyLabel.pack()
BShowVideo = tk.Button(FrameVideoControl, text="Show Live CAM feed", command=showCameraFrameWindow)
BShowVideo.place(x=100, y=-5)
BPositionCalibration = tk.Button(FrameVideoControl, text="Toggle Calibration View", command=showCalqueCalibration)
BPositionCalibration.place(x=230, y=-5)

sliderUH = tk.Scale(FrameVideoControl, from_=-50, to=50, orient="horizontal", label="upper H", length=350,
                    tickinterval=10)
sliderUH.set(sliderHDefault)
sliderUH.pack()
sliderUS = tk.Scale(FrameVideoControl, from_=-50, to=50, orient="horizontal", label="upper S", length=350,
                    tickinterval=10)
sliderUS.set(sliderSDefault)
sliderUS.pack()
sliderUV = tk.Scale(FrameVideoControl, from_=-50, to=50, orient="horizontal", label="upper V", length=350,
                    tickinterval=10)
sliderUV.set(sliderVDefault)
sliderUV.pack()

sliderLH = tk.Scale(FrameVideoControl, from_=-50, to=50, orient="horizontal", label="lower H", length=350,
                    tickinterval=10)
sliderLH.set(sliderHDefault)
sliderLH.pack()
sliderLS = tk.Scale(FrameVideoControl, from_=-50, to=50, orient="horizontal", label="lower S", length=350,
                    tickinterval=10)
sliderLS.set(sliderSDefault)
sliderLS.pack()
sliderLV = tk.Scale(FrameVideoControl, from_=-50, to=50, orient="horizontal", label="lower V", length=350,
                    tickinterval=10)
sliderLV.set(sliderVDefault)
sliderLV.pack()
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
FrameServosControl.place(x=20, y=580, width=390)
BQuit = tk.Button(FrameServosControl, text="Quit", command=endProgam)
BQuit.pack()
Bkalman = tk.Button(FrameServosControl, text="Kalman Filter OFF", command=UseKalmanJudge)
Bkalman.pack()

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
FramePIDCoef.place(x=420, y=20, width=380)
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
FrameBallControl.place(x=420, y=350, width=380, height=200)

sliderRadius = tk.Scale(FrameBallControl, from_=0, to=300, orient="horizontal", label="Radius", length=350,
                        tickinterval=50, resolution=1, command=radiusUpdate)
sliderRadius.set(sliderRadiusDefault)
sliderRadius.pack()
sliderSpeed = tk.Scale(FrameBallControl, from_=0, to=20, orient="horizontal", label="Speed", length=350, tickinterval=1,
                       resolution=1)
sliderSpeed.set(sliderSpeedDefault)
sliderSpeed.pack()
BballDrawCircle = tk.Button(FrameBallControl, text="Enable Circle Trajectory", command=startDrawCircle)
BballDrawCircle.place(x=70, y=-5)
BballDrawEight = tk.Button(FrameBallControl, text="Enable Eight Trajectory", command=startDrawEight)
BballDrawEight.place(x=220, y=-5)

# 获取鼠标的位置，转换成像素值。
# https://www.cnblogs.com/progor/p/8505599.html
videoWindow.protocol("WM_DELETE_WINDOW", donothing)
videoWindow.bind("<Button-3>", getMouseClickPosition)
videoWindow.bind("<Button-1>", setRefWithMouse)  # mouse click to set reference position

main()
tk.mainloop()
