import cv2

# 打开摄像头（默认设备索引0）
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

    cv2.imshow('camera', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
