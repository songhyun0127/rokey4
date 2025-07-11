import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import time

from visualization_msgs.msg import Marker


class DepthCalculator(Node):
    def __init__(self, x, y):
        super().__init__('depth_calculator_terminal')

        self.bridge = CvBridge()
        self.depth_image = None
        self.rgb_image = None

        self.fx = self.fy = self.cx = self.cy = None

        self.target_x = x
        self.target_y = y

        self.last_print_time = 0

        # ROS2 Publishers/Subscribers
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.camera_info_callback, 10)
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        self.print_depth()

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def print_depth(self):
        if self.depth_image is None:
            return

        try:
            depth = float(self.depth_image[self.target_y, self.target_x])
        except IndexError:
            self.get_logger().warn(f"좌표 ({self.target_x}, {self.target_y})가 이미지 범위를 벗어났습니다.")
            return

        if depth == 0 or np.isnan(depth):
            self.get_logger().warn(f"({self.target_x}, {self.target_y}) → No depth data")
            return

        now = time.time()
        if now - self.last_print_time < 1.0:
            return
        self.last_print_time = now

        depth_m = depth / 1000.0
        self.get_logger().info(f"({self.target_x}, {self.target_y}) → {depth_m:.3f} m")

        if None not in (self.fx, self.fy, self.cx, self.cy):
            X = (self.target_x - self.cx) * depth_m / self.fx
            Y = (self.target_y - self.cy) * depth_m / self.fy
            Z = depth_m
            self.get_logger().info(f"  3D coordinates: X={X:.3f}, Y={Y:.3f}, Z={Z:.3f}")
            self.publish_marker(X, Y, Z)

    def publish_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "camera_depth_optical_frame"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "depth_point"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    try:
        x = int(input("X 좌표를 입력하세요: "))
        y = int(input("Y 좌표를 입력하세요: "))
    except Exception as e:
        print(f"좌표 입력 오류: {e}")
        return

    node = DepthCalculator(x, y)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
