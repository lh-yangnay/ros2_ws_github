import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


def nothing(x):
    pass


class HsvImageNode(Node):

    def __init__(self):
        super().__init__('hsv_image_node')
        self.bridge_ = CvBridge()
        self.subscription_ = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)

        cv2.namedWindow('result')
        cv2.namedWindow('mask')

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

        self.get_logger().info('HSV 图像处理节点已启动.')

    def image_callback(self, msg):
        frame = self.bridge_.imgmsg_to_cv2(msg, 'bgr8')
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

        # 查找轮廓并框选
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:
                continue

            # 外接矩形（红色框选）
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

            # 计算中心点坐标
            cx = x + w // 2
            cy = y + h // 2
            cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)
            cv2.putText(frame, f'({cx}, {cy})', (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        cv2.imshow('original', frame)
        cv2.imshow('mask', mask)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = HsvImageNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
