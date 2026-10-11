import rclpy
from rclpy.node import Node

class SecondNode(Node):
    def __init__(self):
        super().__init__('second_node')
        self.get_logger().info('我是第二个节点，second_node 启动了！')

def main(args=None):
    rclpy.init(args=args)
    node = SecondNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
