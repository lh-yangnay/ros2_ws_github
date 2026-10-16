import cv2
import numpy as np

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print('无法打开摄像头')
    exit()

print('按 q 键退出')

while True:
    ret, frame = cap.read()
    if not ret:
        print('无法读取画面')
        break

    # BGR 转 HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 分离 H、S、V 三个通道
    h, s, v = cv2.split(hsv)

    # 原图与三通道分别显示
    cv2.imshow('original', frame)
    cv2.imshow('H', h)
    cv2.imshow('S', s)
    cv2.imshow('V', v)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
