import rclpy
from rclpy.node import Node

class ThirdNode(Node):
    def __init__(self):
        super().__init__('third_node')
        self.get_logger().info('我是第三个节点，third_node 启动了！')

def main(args=None):
    rclpy.init(args=args)
    node = ThirdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
