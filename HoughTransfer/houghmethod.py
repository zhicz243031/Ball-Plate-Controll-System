# 这一部分代码是霍夫变换的测试代码
import cv2
import time
from matplotlib import pyplot as plt
import numpy as np

vs = cv2.VideoCapture(1, cv2.CAP_DSHOW)
# vs = cv2.VideoCapture('C:/Users/50578/working/Ball-Tracking/BlueBal.avi')
vs.set(3, 1280)
vs.set(4, 720)
time.sleep(.5)

while vs.isOpened():
    ret, frame = vs.read()
    frame = frame[0:720, 280:1030]

    # frame = cv2.GaussianBlur(frame, (15, 15), 0)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20, param1=130, param2=30, minRadius=0, maxRadius=25)
    if circles is not None:
        for x, y, r in circles[0]:
            if r > 5:
                print(x,y,r)
                # center = (x,y)
                cv2.putText(frame, str(int(x)) + ";" + str(int(y)).format(0, 0), (int(x) - 50, int(y) - 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.circle(frame, (int(x), int(y)), int(r), (0, 255, 255), 2)
                cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)

    else:
        print('nothing.')


    if cv2.waitKey(15) == ord('q'):
        vs.release()
        cv2.destroyAllWindows()
        break

