import signal
import time
import cv2
import imutils
# import samplePro

vs = cv2.VideoCapture(2)
vs.set(3, 640)
vs.set(4, 480)
print("Open camera succeed.")

refX=240
refY=240
# Define signal handler function
def myHandler(signum, frame):
    print(time.clock())

    ret, frame = vs.read()
    frame = frame[20:460, 120:560]

    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    silverLower = (43, 30, 140)
    silverUpper = (105, 160, 255)

    mask_before = cv2.inRange(hsv, silverLower, silverUpper)
    mask_erode = cv2.erode(mask_before, None, iterations=2)
    mask = cv2.dilate(mask_erode, None, iterations=2)

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    center = None

    cv2.circle(frame, (int(refX), int(refY)), int(4), (255, 0, 0), 2)
    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        # M = cv2.moments(c)
        # center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # if radius > 10:
        #     cv2.putText(frame, str(int(x)) + ";" + str(int(y)).format(0, 0), (int(x) - 50, int(y) - 50),
        #                 cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        #     cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
        #     cv2.circle(frame, center, 5, (0, 0, 255), -1)
            # print(x,y)

    # cv2.imshow('frame', frame)


def main():
    signal.signal(signal.SIGALRM, myHandler)
    signal.setitimer(signal.ITIMER_REAL,1,0.3)
    while (1):
        pass

main()
