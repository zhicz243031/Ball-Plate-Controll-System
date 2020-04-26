import signal
import time
import cv2
import samplePro

vs = cv2.VideoCapture(0)
vs.set(3, 1280)
vs.set(4, 720)


# vs.set(3, 1280)
# vs.set(4, 720)
# time.sleep(2)
# print("Open camera succeed.")

# Define signal handler function
def myHandler(signum, frame):

    signal.setitimer(signal.ITIMER_REAL, 0.5)
    a, b, c = samplePro.ImgProcess(vs, 360, 360)
    print(a, b, c)
    # print("此处可调用自己想要执行的函数")


def main():
    signal.signal(signal.SIGALRM, myHandler)
    signal.setitimer(signal.ITIMER_REAL, 0.5)
    while (1):
        # vs = cv2.VideoCapture(0)
        # vs.set(3, 1280)
        # vs.set(4, 720)
        pass
    exit()


main()
