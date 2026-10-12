import sys
import threading
from math import radians

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QDoubleSpinBox, QFormLayout,
                             QHBoxLayout, QLabel, QPushButton, QVBoxLayout,
                             QWidget)


class TurtleAdvceCtrlNode(Node):

    def __init__(self):
        super().__init__('turtle_advce_ctrl_node')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription_ = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10)
        self.pose = None
        self.pose_listener = None
        self.get_logger().info('小乌龟高级控制器节点已启动.')

    def send_command(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.publisher_.publish(msg)
        self.get_logger().info(f'已发送速度指令: 线速度={linear}, 角速度={angular}')

    def pose_callback(self, msg):
        self.pose = msg
        if self.pose_listener is not None:
            self.pose_listener(msg)


class PoseSignals(QObject):
    pose_received = pyqtSignal(object)


class TurtleCtrlWindow(QWidget):

    def __init__(self, node):
        super().__init__()
        self.node_ = node
        self.setWindowTitle('小乌龟控制器')

        self.signals_ = PoseSignals()
        self.signals_.pose_received.connect(self.update_pose)
        self.node_.pose_listener = self.signals_.pose_received.emit

        self.linear_spin_ = QDoubleSpinBox()
        self.linear_spin_.setRange(-10.0, 10.0)
        self.linear_spin_.setSingleStep(0.5)

        self.angular_spin_ = QDoubleSpinBox()
        self.angular_spin_.setRange(-3600.0, 3600.0)
        self.angular_spin_.setSingleStep(10.0)

        send_btn = QPushButton('发送')
        send_btn.clicked.connect(self.on_send)

        ctrl_layout = QVBoxLayout()
        ctrl_layout.addWidget(QLabel('线速度'))
        ctrl_layout.addWidget(self.linear_spin_)
        ctrl_layout.addWidget(QLabel('角速度(度)'))
        ctrl_layout.addWidget(self.angular_spin_)
        ctrl_layout.addWidget(send_btn)

        self.x_label_ = QLabel('x: --')
        self.y_label_ = QLabel('y: --')
        self.theta_label_ = QLabel('theta: --')
        self.linear_vel_label_ = QLabel('线速度: --')
        self.angular_vel_label_ = QLabel('角速度: --')

        pose_layout = QFormLayout()
        pose_layout.addRow('位置 x', self.x_label_)
        pose_layout.addRow('位置 y', self.y_label_)
        pose_layout.addRow('朝向 theta', self.theta_label_)
        pose_layout.addRow('当前线速度', self.linear_vel_label_)
        pose_layout.addRow('当前角速度', self.angular_vel_label_)

        main_layout = QHBoxLayout()
        main_layout.addLayout(ctrl_layout)
        main_layout.addLayout(pose_layout)
        self.setLayout(main_layout)

    def on_send(self):
        self.node_.send_command(self.linear_spin_.value(),
                                radians(self.angular_spin_.value()))

    def update_pose(self, pose):
        self.x_label_.setText(f'x: {pose.x:.2f}')
        self.y_label_.setText(f'y: {pose.y:.2f}')
        self.theta_label_.setText(f'theta: {pose.theta:.2f}')
        self.linear_vel_label_.setText(f'线速度: {pose.linear_velocity:.2f}')
        self.angular_vel_label_.setText(f'角速度: {pose.angular_velocity:.2f}')


def main():
    rclpy.init()
    node = TurtleAdvceCtrlNode()

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
