import sys
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from PyQt5.QtWidgets import (QApplication, QDoubleSpinBox, QLabel,
                             QPushButton, QVBoxLayout, QWidget)


class TurtleGuiCtrlNode(Node):

    def __init__(self):
        super().__init__('turtle_gui_ctrl_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('小乌龟控制器节点已启动.')

    def send_command(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)
        self.get_logger().info(f'已发送速度指令: 线速度={linear}, 角速度={angular}')


class TurtleCtrlWindow(QWidget):

    def __init__(self, node):
        super().__init__()
        self.node_ = node
        self.setWindowTitle('小乌龟控制器')

        self.linear_spin_ = QDoubleSpinBox()
        self.linear_spin_.setRange(-10.0, 10.0)
        self.linear_spin_.setSingleStep(0.5)

        self.angular_spin_ = QDoubleSpinBox()
        self.angular_spin_.setRange(-10.0, 10.0)
        self.angular_spin_.setSingleStep(0.5)

        send_btn = QPushButton('发送')
        send_btn.clicked.connect(self.on_send)

        layout = QVBoxLayout()
        layout.addWidget(QLabel('线速度'))
        layout.addWidget(self.linear_spin_)
        layout.addWidget(QLabel('角速度'))
        layout.addWidget(self.angular_spin_)
        layout.addWidget(send_btn)
        self.setLayout(layout)

    def on_send(self):
        self.node_.send_command(self.linear_spin_.value(),
                                self.angular_spin_.value())


def main():
    rclpy.init()
    node = TurtleGuiCtrlNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = TurtleCtrlWindow(node)
    window.show()
    app.exec_()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
