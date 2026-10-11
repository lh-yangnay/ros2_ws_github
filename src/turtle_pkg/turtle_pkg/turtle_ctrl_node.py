import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtleCtrlNode(Node):

    def __init__(self):
        super().__init__('turtle_ctrl_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer_ = self.create_timer(0.5, self.publish_velocity)
        self.get_logger().info('Turtle control node started.')

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.5
        self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = TurtleCtrlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
