# depth_calculator.py

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class DepthCalculator(Node):
    def __init__(self):
        super().__init__('depth_calculator')

        # 브리지 및 변수 초기화
        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.fx = self.fy = self.cx = self.cy = None

        # 토픽 구독자 설정
        self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.camera_info_callback, 10)

        # OpenCV 윈도우 및 마우스 콜백
        cv2.namedWindow("RGB Image")
        cv2.setMouseCallback("RGB Image", self.mouse_callback)

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if self.rgb_image is not None:
            cv2.imshow("RGB Image", self.rgb_image)
            cv2.waitKey(1)

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.depth_image is None:
                self.get_logger().warn('Depth image not received yet.')
                return

            try:
                depth = float(self.depth_image[y, x])
            except IndexError:
                self.get_logger().warn(f'Clicked point ({x}, {y}) is out of bounds.')
                return

            if depth == 0 or np.isnan(depth):
                self.get_logger().warn(f'Depth at ({x}, {y}) is zero or invalid.')
                return

            # 거리 단위 보정 (mm → m)
            depth_m = depth / 1000.0
            self.get_logger().info(f"Clicked ({x}, {y}) → Depth: {depth_m:.3f} m")

            # 선택적으로 3D 좌표 변환
            if None not in (self.fx, self.fy, self.cx, self.cy):
                X = (x - self.cx) * depth_m / self.fx
                Y = (y - self.cy) * depth_m / self.fy
                Z = depth_m
                self.get_logger().info(f"3D Coordinates → X: {X:.3f}, Y: {Y:.3f}, Z: {Z:.3f}")
            else:
                self.get_logger().warn("Camera intrinsics not yet received.")


def main(args=None):
    rclpy.init(args=args)
    node = DepthCalculator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
