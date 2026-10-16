import cv2
import numpy as np

# 创建 300x300 的黑色背景图片
img = np.zeros((300, 300, 3), dtype=np.uint8)

# 第三行到第十行改为红色 (BGR: 0,0,255)
img[2:10, :] = (0, 0, 255)

# 第100行100列到第200行200列改为蓝色 (BGR: 255,0,0)
img[99:200, 99:200] = (255, 0, 0)

# 显示效果
cv2.imshow('image', img)
cv2.waitKey(0)
cv2.destroyAllWindows()
