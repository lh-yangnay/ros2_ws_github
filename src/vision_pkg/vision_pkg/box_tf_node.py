import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf2_geometry_msgs  # noqa: F401  注册 PointStamped 的 tf2 转换支持

# ==================== 可配置参数（按需修改） ====================
# 机械臂基座坐标系 / 相机坐标系
ROBOT_BASE_FRAME = 'base_link'
CAMERA_FRAME = 'camera_link'
# 相机在机械臂基座坐标系中的位置与姿态（单位：米 / 弧度）
CAMERA_X = 0.01
CAMERA_Y = 0.25
CAMERA_Z = 0.5
CAMERA_ROLL = 0.0
CAMERA_PITCH = 0.0
CAMERA_YAW = 0.0
# 像素与物理尺寸的比例值（单位：米/像素）
PIXEL_SCALE = 0.001
# 图像中心点像素坐标（一般取图像宽高的一半）
IMAGE_CENTER_X = 320.0
IMAGE_CENTER_Y = 240.0
# 被测物体到相机的距离（沿相机光轴方向，单位：米）
OBJECT_DISTANCE = 0.3
# 订阅的像素坐标话题 / 发布真实坐标的话题
CENTER_TOPIC = '/box_center'
OUTPUT_TOPIC = '/box_center_robot'


def euler_to_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class BoxTfNode(Node):

    def __init__(self):
        super().__init__('box_tf_node')
        self.base_frame_ = self.declare_parameter(
            'robot_base_frame', ROBOT_BASE_FRAME).value
        self.camera_frame_ = self.declare_parameter(
            'camera_frame', CAMERA_FRAME).value
        self.camera_x_ = self.declare_parameter('camera_x', CAMERA_X).value
        self.camera_y_ = self.declare_parameter('camera_y', CAMERA_Y).value
        self.camera_z_ = self.declare_parameter('camera_z', CAMERA_Z).value
        self.camera_roll_ = self.declare_parameter(
            'camera_roll', CAMERA_ROLL).value
        self.camera_pitch_ = self.declare_parameter(
            'camera_pitch', CAMERA_PITCH).value
        self.camera_yaw_ = self.declare_parameter(
            'camera_yaw', CAMERA_YAW).value
        self.pixel_scale_ = self.declare_parameter(
            'pixel_scale', PIXEL_SCALE).value
        self.image_center_x_ = self.declare_parameter(
            'image_center_x', IMAGE_CENTER_X).value
        self.image_center_y_ = self.declare_parameter(
            'image_center_y', IMAGE_CENTER_Y).value
        self.object_distance_ = self.declare_parameter(
            'object_distance', OBJECT_DISTANCE).value
        center_topic = self.declare_parameter(
            'center_topic', CENTER_TOPIC).value
        output_topic = self.declare_parameter(
            'output_topic', OUTPUT_TOPIC).value

        # 发布机械臂基座坐标系 -> 相机坐标系的静态变换
        self.tf_broadcaster_ = StaticTransformBroadcaster(self)
        self.publish_static_transform()

        # tf2 缓冲与监听，用于查询变换关系
        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.subscription_ = self.create_subscription(
            Point, center_topic, self.center_callback, 10)
        self.publisher_ = self.create_publisher(PointStamped, output_topic, 10)
        self.get_logger().info('坐标转换节点已启动.')

    def publish_static_transform(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.base_frame_
        tf.child_frame_id = self.camera_frame_
        tf.transform.translation.x = float(self.camera_x_)
        tf.transform.translation.y = float(self.camera_y_)
        tf.transform.translation.z = float(self.camera_z_)
        qx, qy, qz, qw = euler_to_quaternion(
            self.camera_roll_, self.camera_pitch_, self.camera_yaw_)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster_.sendTransform(tf)

    def center_callback(self, msg):
        # 像素坐标 -> 相机坐标系下的三维点（相机光学系：x 向右，y 向下，z 向前）
        point_cam = PointStamped()
        point_cam.header.stamp = self.get_clock().now().to_msg()
        point_cam.header.frame_id = self.camera_frame_
        point_cam.point.x = (msg.x - self.image_center_x_) * self.pixel_scale_
        point_cam.point.y = (msg.y - self.image_center_y_) * self.pixel_scale_
        point_cam.point.z = float(self.object_distance_)

        # 通过 tf 转换到机械臂基座坐标系
        try:
            point_base = self.tf_buffer_.transform(
                point_cam, self.base_frame_)
        except TransformException as ex:
            self.get_logger().warn(f'坐标变换失败: {ex}')
            return

        self.publisher_.publish(point_base)
        self.get_logger().info(
            f'像素({msg.x:.0f}, {msg.y:.0f}) -> 机械臂坐标'
            f'({point_base.point.x:.3f}, {point_base.point.y:.3f}, '
            f'{point_base.point.z:.3f})')


def main():
    rclpy.init()
    node = BoxTfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
