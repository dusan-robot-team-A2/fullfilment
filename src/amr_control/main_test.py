# aruco_detector.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
from aruco import Aruco

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        # create ROS communication
        self.publisher_ = self.create_publisher(Float32MultiArray, 'aruco_relative_position', 10)
        self.subscription = self.create_subscription(Image, '/camera/image_raw/Compress_image', self.image_callback, 10)
        self.bridge = CvBridge()

        camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])  # 카메라 행렬
        dist_coeffs = np.array([k1, k2, p1, p2, k3])  # 왜곡 계수
        self.aruco = Aruco(camera_matrix, dist_coeffs)  # Aruco 클래스의 인스턴스를 생성합니다.

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        corners, ids = self.aruco.detect_markers(frame)
        if ids is not None and len(ids) >= 2:
            rvecs, tvecs = self.aruco.estimate_pose(corners)
            rvec1, tvec1 = rvecs[0], tvecs[0]
            rvec2, tvec2 = rvecs[1], tvecs[1]
            dx, dy = self.aruco.calculate_relative_position(rvec1, tvec1, rvec2, tvec2)
            if abs(dx) > 0.01 and abs(dy) > 0.01:
                self.publish_relative_position(dx, dy)
            return

    def publish_relative_position(self, dx, dy):
        msg = Float32MultiArray()
        msg.data = [dx, dy]
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
