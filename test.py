import cv2

vs = cv2.VideoCapture(2)
print("Open camera succeed.")

while(1):
    ret, frame = vs.read()
    frame = frame[20:460, 120:560]
    cv2.imshow('frame', frame)

    k = cv2.waitKey(60) & 0xff
    if k == 27:
        break

cv2.destroyAllWindows()
vs.release()