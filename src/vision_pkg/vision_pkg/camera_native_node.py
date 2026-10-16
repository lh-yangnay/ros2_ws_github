import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraNativeNode(Node):

    def __init__(self):
        super().__init__('camera_native_node')
        self.bridge_ = CvBridge()
        self.subscription_ = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10)
        self.get_logger().info('摄像头订阅节点已启动.')

    def image_callback(self, msg):
        frame = self.bridge_.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow('camera', frame)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = CameraNativeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
