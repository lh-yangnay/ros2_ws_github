import cv2
import numpy as np

cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print('无法打开摄像头')
    exit()

cv2.namedWindow('result')
cv2.namedWindow('mask')

def nothing(x):
    pass

# HSV 阈值调节滑块
cv2.createTrackbar('H_min', 'result', 0, 179, nothing)
cv2.createTrackbar('H_max', 'result', 179, 179, nothing)
cv2.createTrackbar('S_min', 'result', 0, 255, nothing)
cv2.createTrackbar('S_max', 'result', 255, 255, nothing)
cv2.createTrackbar('V_min', 'result', 0, 255, nothing)
cv2.createTrackbar('V_max', 'result', 255, 255, nothing)
# 腐蚀膨胀调节滑块
cv2.createTrackbar('erode', 'result', 0, 10, nothing)
cv2.createTrackbar('dilate', 'result', 0, 10, nothing)

print('按 q 键退出')

while True:
    ret, frame = cap.read()
    if not ret:
        print('无法读取画面')
        break

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    h_min = cv2.getTrackbarPos('H_min', 'result')
    h_max = cv2.getTrackbarPos('H_max', 'result')
    s_min = cv2.getTrackbarPos('S_min', 'result')
    s_max = cv2.getTrackbarPos('S_max', 'result')
    v_min = cv2.getTrackbarPos('V_min', 'result')
    v_max = cv2.getTrackbarPos('V_max', 'result')

    lower = np.array([h_min, s_min, v_min])
    upper = np.array([h_max, s_max, v_max])
    mask = cv2.inRange(hsv, lower, upper)

    # 腐蚀膨胀操作
    erode_iter = cv2.getTrackbarPos('erode', 'result')
    dilate_iter = cv2.getTrackbarPos('dilate', 'result')
    kernel = np.ones((3, 3), np.uint8)
    if erode_iter > 0:
        mask = cv2.erode(mask, kernel, iterations=erode_iter)
    if dilate_iter > 0:
        mask = cv2.dilate(mask, kernel, iterations=dilate_iter)

    # 用 mask 提取原图中的目标区域
    result = cv2.bitwise_and(frame, frame, mask=mask)

    cv2.imshow('original', frame)
    cv2.imshow('result', result)
    cv2.imshow('mask', mask)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
