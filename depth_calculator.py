# depth_calculator.py

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import numpy as np


class DepthCalculator(Node):
    def __init__(self, x, y):
        super().__init__('depth_calculator')

        self.bridge = CvBridge()
        self.rgb_image = None
        self.depth_image = None
        self.fx = self.fy = self.cx = self.cy = None

        # 사용자 입력 좌표
        self.target_x = x
        self.target_y = y

        # 구독자 등록
        self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 10)
        self.create_subscription(Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.camera_info_callback, 10)

        cv2.namedWindow("RGB Image")

    def rgb_callback(self, msg):
        self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.display_with_depth()

    def depth_callback(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def camera_info_callback(self, msg):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]

    def display_with_depth(self):
        if self.rgb_image is None or self.depth_image is None:
            return

        image = self.rgb_image.copy()

        try:
            depth = float(self.depth_image[self.target_y, self.target_x])
        except IndexError:
            self.get_logger().warn(f"좌표 ({self.target_x}, {self.target_y}) 가 이미지 범위를 벗어났습니다.")
            return

        if depth == 0 or np.isnan(depth):
            text = f"({self.target_x}, {self.target_y}) → No Depth Data"
        else:
            depth_m = depth / 1000.0
            text = f"({self.target_x}, {self.target_y}) → {depth_m:.2f} m"

        cv2.circle(image, (self.target_x, self.target_y), 5, (0, 0, 255), -1)
        cv2.putText(image, text, (self.target_x + 10, self.target_y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        cv2.imshow("RGB Image", image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    # 사용자 입력 받기
    try:
        x = int(input("X 좌표를 입력하세요: "))
        y = int(input("Y 좌표를 입력하세요: "))
    except Exception as e:
        print("좌표 입력 오류:", e)
        return

    node = DepthCalculator(x, y)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()
