from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import tkinter as tk  # Python GUI
import tkinter.messagebox
from PIL import Image, ImageTk  # Python Imaging Library

vs = cv2.VideoCapture('ball_tracking_example.mp4')
# time.sleep(1.0)

getPixelColor = False  # flag to get the pixel color of the ball when needed
camHeight = 480
camWidth = 640
H, S, V = 0, 0, 0  # the color properties of the Pixel to track
mouseX, mouseY = 0, 0  # declare variables to capture mouse position for color tracking

controllerWindow = tk.Tk()  # initializes this tk interpreter and creates the root window
controllerWindow.title("2DOF Ball-Plate Control Window ")  # define title of the root window
controllerWindow.geometry("1200x800")  # define size of the root window
controllerWindow["bg"] = "lightgrey"  # define the background color of the root window
controllerWindow.resizable(0, 0)  # define if the root window is resizable or not for Vertical and horizontal，不可以拉伸

videoWindow = tk.Toplevel(controllerWindow)  # a new window derived from the root window "controllerwindow"
videoWindow.title("Cam Footage")  # define title of videowindow
videoWindow.resizable(0, 0)  # Cannot resize the window
lmain = tk.Label(videoWindow)  # create an empty label widget in the videoWindow
lmain.pack()  # adjust the size of the videowindow to fit the label lmain
videoWindow.withdraw()  # hide the window

graphWindow = tk.Toplevel(controllerWindow)  # a new window derived from the root window "graphwindow"
graphWindow.title("Position in function of time")  # define window title
graphWindow.resizable(0, 0)  # define if resizable or not
graphCanvas = tk.Canvas(graphWindow, width=500 + 200, height=camHeight)  # create a canvas widget in graphwindow
graphCanvas.pack()  # pack the canvas widget
graphWindow.withdraw()  # hide the graphwindow

# Defining all the slider default values
sliderHDefault = 0
sliderSDefault = 0
sliderVDefault = 0
sliderCoefPDefault = 0.035
sliderCoefIDefault = 0.0
sliderCoefDDefault = 0.015
sliderRadiusDefault = 10
sliderSpeedDefault = 10
sliderRefXDefault = camWidth / 2
sliderRefYDefault = camHeight / 2


# def showCameraFrameWindow():
#     print('show camera frame window.')

def showCalqueCalibration():
    print('show calque calibration.')


def setRefWithMouse(
        mousePosition):  # set refX and refY based on the mousePosition, mousePosition is the realtime position of the mouse not a saved variable
    global refX, refY
    if mousePosition.y > 10:
        refreshGraph()
        refX, refY = mousePosition.x, mousePosition.y


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


def PIDcontrol():
    Kp = sliderCoefP.get()
    Ki = sliderCoefI.get()
    Kd = sliderCoefD.get()
    # print(Kp,Ki,Kd)


def refreshGraph():  # function that reset the time variable to 480 if the graph is full
    global t
    t = 480


def endProgam():  # function to close root window
    controllerWindow.destroy()


start_time = 0


def main():
    global H, S, V
    global getPixelColor
    global refX, refY, totalErrorX, totalErrorY
    global camWidth, camHeight
    global timeInterval, start_time

    start_time = time.time()

    _, frame = vs.read()
    frame = imutils.resize(frame, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    greenLower = (29, 86, 6)
    greenUpper = (64, 255, 255)
    # greenLower = (29 - sliderLH.get(), 86 - sliderLS.get(), 6 - sliderLV.get())
    # greenUpper = (64 + sliderUH.get(), 255 + sliderUS.get() , 255 + sliderUV.get())

    mask_before = cv2.inRange(hsv, greenLower, greenUpper)
    mask_erode = cv2.erode(mask_before, None, iterations=2)
    mask = cv2.dilate(mask_erode, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    a = cnts
    cnts = imutils.grab_contours(cnts)
    center = None

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        # print((x,y),radius)
        if radius > 10:
            cv2.putText(frame, str(int(x)) + ";" + str(int(y)).format(0, 0), (int(x) - 50, int(y) - 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 5, (0, 0, 255), -1)
            PIDcontrol()

    # cv2.imshow('mask_before', mask_before)
    # cv2.imshow('frame', frame)
    print('line170')
    if showVideoWindow == True:
        print('line172')
        cv2.imshow('frame', frame)
        # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # frame = Image.fromarray(frame)
        # imgtk = ImageTk.PhotoImage(image=frame)
        # lmain.imgtk = imgtk
        # lmain.configure(image=imgtk)
    lmain.after(5, main)


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
                        tickinterval=50, resolution=1)
sliderRadius.set(sliderRadiusDefault)
sliderRadius.pack()
sliderSpeed = tk.Scale(FrameBallControl, from_=0, to=20, orient="horizontal", label="Speed", length=350, tickinterval=1,
                       resolution=1)
sliderSpeed.set(sliderSpeedDefault)
sliderSpeed.pack()
BballDrawCircle = tk.Button(FrameBallControl, text="Enable Circle Trajectory", command=showCalqueCalibration)
BballDrawCircle.place(x=70, y=-5)
BballDrawEight = tk.Button(FrameBallControl, text="Enable Eight Trajectory", command=showCalqueCalibration)
BballDrawEight.place(x=220, y=-5)

main()
tk.mainloop()