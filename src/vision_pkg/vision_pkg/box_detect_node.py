import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

# ==================== 可配置参数（按需修改） ====================
# HSV 颜色阈值范围 [H, S, V]，H: 0-179, S/V: 0-255
HSV_LOWER = (0, 255, 22)        # 颜色下限
HSV_UPPER = (0, 255, 255)  # 颜色上限
# 腐蚀/膨胀次数，0 表示不执行
ERODE_ITER = 0
DILATE_ITER = 0
# 最小识别面积（像素），小于该值的轮廓将被忽略
MIN_AREA = 500
# 位置信息发布话题名
POSITION_TOPIC = '/box_center'


class BoxDetectNode(Node):

    def __init__(self):
        super().__init__('box_detect_node')
        self.bridge_ = CvBridge()
        self.subscription_ = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.position_pub_ = self.create_publisher(Point, POSITION_TOPIC, 10)
        self.get_logger().info('目标检测节点已启动.')

    def image_callback(self, msg):
        frame = self.bridge_.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 使用固定阈值生成 mask
        mask = cv2.inRange(hsv, np.array(HSV_LOWER), np.array(HSV_UPPER))

        # 固定腐蚀膨胀操作
        kernel = np.ones((3, 3), np.uint8)
        if ERODE_ITER > 0:
            mask = cv2.erode(mask, kernel, iterations=ERODE_ITER)
        if DILATE_ITER > 0:
            mask = cv2.dilate(mask, kernel, iterations=DILATE_ITER)

        # 查找轮廓并框选
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            if cv2.contourArea(cnt) < MIN_AREA:
                continue

            # 外接矩形（红色框选）
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

            # 计算中心点坐标并标注
            cx = x + w // 2
            cy = y + h // 2
            cv2.circle(frame, (cx, cy), 4, (0, 255, 0), -1)
            cv2.putText(frame, f'({cx}, {cy})', (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # 发布目标中心点位置
            pos = Point()
            pos.x = float(cx)
            pos.y = float(cy)
            self.position_pub_.publish(pos)

        cv2.imshow('original', frame)
        cv2.imshow('mask', mask)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = BoxDetectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
