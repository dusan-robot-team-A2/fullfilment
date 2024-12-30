import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from std_msgs.msg import Int16
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Point
from fullfilment_interfaces.action import JobAction, MoveBoxes
from std_srvs.srv import SetBool
import cv2
from cv_bridge import CvBridge
<<<<<<< HEAD
import numpy as np

from util.transform import create_transformation_matrix, transformation_3d, extract_rotation_translation
from aruco.aruco import Aruco
from aruco.camera_util import CameraUtil

from .custom_coordinate import CustomCoordinateSystem
from .coords_manager import CoordsManager
=======
from fullfilment_interfaces.srv import MoveBasket
>>>>>>> update_manipulator

class main_node(Node):
    def __init__(self):
        super().__init__('main_node')  # 노드 이름
<<<<<<< HEAD
=======
        self.robot_status_pub = self.create_publisher(Int16, 'robot_status', 10)
        self.conveyor_status_sub = self.create_subscription(Int16, 'conveyor_status', self.conveyor_status, 10)
        self.image_pub = self.create_publisher(CompressedImage, 'global_camera', 10)
>>>>>>> update_manipulator

        self.camera_util = CameraUtil()
        mtx, dist = self.camera_util.get_camera_mtx_dist()
        self.aruco = Aruco(mtx, dist)
        self.coords_manager:CoordsManager = CoordsManager()

        self.robot_status_pub = self.create_publisher(Int16, 'robot_status', 10)
        self.conveyor_status_sub = self.create_subscription(Int16, 'conveyor_status', self.conveyor_status, 10)
        self.image_pub = self.create_publisher(CompressedImage, 'global_camera', 10)
        self.image_sub = self.create_subscription(CompressedImage, 'image_raw/compressed', self.subscribe_image, 10)

        self.aruco_id_matcher = {
            3: "global",
            60: "base",
            62: "goal1",
            63: "goal2",
            64: "goal3",
            65: "robot",
        }


        self.job_action_server = ActionServer(
            self,
            JobAction,
            'job_command',
            self.execute_callback
        )

        self.job_action_client = ActionClient(
            self,
            MoveBoxes,
            'job_command2amr'
        )

        self.basket_client = self.create_client(MoveBasket, 'move_basket')

        self.conveyor_service = self.create_service(SetBool, '/conveyor_control', self.conveyor_callback)
        self.order_to_conveyor = self.create_client(SetBool, '/conveyor_move')
        self.conveyor_mode = 0
        # OpenCV와 ROS 간 변환을 위한 CvBridge 초기화
        self.bridge = CvBridge()

        # 주기적인 이미지 전송을 위한 타이머 설정 (주기: 1초)
        # self.timer = self.create_timer(0.1, self.publish_image)

    def subscribe_image(self, msg:CompressedImage):
        if self.bridge is None:
            return
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.publish_image(cv_image)

    def publish_image(self,frame):
        # 프레임 읽기 변경 필요 -> image_sub로 읽어오기
        # frame = cv2.imread('aruco/images/image.png')
        frame = self.aruco_analyzer(frame)
        # frame = self.test_draw_all(frame)


        # OpenCV 이미지 (BGR)을 JPEG로 압축
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]  # 90은 압축 품질
        _, compressed_image = cv2.imencode('.jpg', frame, encode_param)

        # 압축된 이미지를 CompressedImage 메시지로 변환
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()  # 타임스탬프 추가
        msg.header.frame_id = "camera"  # 프레임 ID 설정
        msg.format = "jpeg"  # 압축 형식 설정
        msg.data = compressed_image.tobytes()  # 압축된 이미지 데이터

        # CompressedImage 퍼블리시
        self.image_pub.publish(msg)
        self.get_logger().info('Publishing compressed image...')

    def aruco_analyzer(self, frame):
        ids, rvecs, tvecs = self.aruco.get_matrix(frame)
        if ids is not None:
            for i in range(len(ids)):
                id = ids[i][0]
                rvec, tvec = rvecs[i], tvecs[i]

                if self.aruco_id_matcher[id] == "global":
                    if self.coords_manager.global_coor == None:
                        coor_system = CustomCoordinateSystem(rvec=rvec, tvec=tvec)
                        self.coords_manager.set_global_coor(coor_system)
                    else:
                        self.coords_manager.update_global_coor(rvec,tvec)
                elif self.aruco_id_matcher[id] == "base":
                    if self.coords_manager.base_coor == None:
                        coor_system = CustomCoordinateSystem(rvec=rvec, tvec=tvec)
                        self.coords_manager.set_base_coor(coor_system)
                    else:
                        self.coords_manager.update_base_coor(rvec,tvec)
                elif self.aruco_id_matcher[id] == "goal1":
                    if self.coords_manager.goals_coor[0] == None:
                        coor_system = CustomCoordinateSystem(rvec=rvec, tvec=tvec)
                        self.coords_manager.set_goal_coor(coor_system,0)
                    else:
                        self.coords_manager.update_goal_coor(rvec,tvec,0)
                elif self.aruco_id_matcher[id] == "goal2":
                    if self.coords_manager.goals_coor[1] == None:
                        coor_system = CustomCoordinateSystem(rvec=rvec, tvec=tvec)
                        self.coords_manager.set_goal_coor(coor_system,1)
                    else:
                        self.coords_manager.update_goal_coor(rvec,tvec,1)
                elif self.aruco_id_matcher[id] == "goal3":
                    if self.coords_manager.goals_coor[2] == None:
                        coor_system = CustomCoordinateSystem(rvec=rvec, tvec=tvec)
                        self.coords_manager.set_goal_coor(coor_system,2)
                    else:
                        self.coords_manager.update_goal_coor(rvec,tvec,2)
                elif self.aruco_id_matcher[id] == "robot":
                    if self.coords_manager.robot_coor == None:
                        coor_system = CustomCoordinateSystem(rvec=rvec, tvec=tvec)
                        self.coords_manager.set_robot_coor(coor_system)
                    else:
                        self.coords_manager.update_robot_coor(rvec,tvec)

        if self.coords_manager is not None:
            self.coords_manager.draw_frame(frame, self.aruco.camera_matrix, self.aruco.dist_coeffs)

        return frame
    

    def test_draw_all(self, frame):
        ids, rvecs, tvecs = self.aruco.get_matrix(frame)
        if ids is not None:
            for i in range(len(ids)):
                # 각 마커에 대해 축 그리기
                cv2.drawFrameAxes(frame, self.aruco.camera_matrix, self.aruco.dist_coeffs, rvecs[i], tvecs[i], 0.1)

                id_text = str(ids[i][0])  # ID는 2D 배열이므로 첫 번째 값을 가져옵니다

                # tvecs[i]는 카메라 좌표계에서의 위치, 이를 이미지 좌표계로 변환하기
                img_points, _ = cv2.projectPoints(np.array([[0, 0, 0]], dtype=np.float32), rvecs[i], tvecs[i], self.aruco.camera_matrix, self.aruco.dist_coeffs)
                position = (int(img_points[0][0][0]), int(img_points[0][0][1]))  # 변환된 이미지 좌표

                # ID를 텍스트로 표시
                cv2.putText(frame, id_text, position, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        return frame


    def conveyor_status(self, msg):
        self.conveyor_mode = msg
    
    def conveyor_callback(self, request, response):
        status = request.data

        self.move_conveyor(status)
        response.message = self.conveyor_mode
        
        response.success = True
        return response 

    async def execute_callback(self, goal_handle):
        self.get_logger().info(f"Goal Received: red_num={goal_handle.request.red_num}, "
                               f"blue_num={goal_handle.request.blue_num}, goal_num={goal_handle.request.goal_num}")

        # move robot to boxes
        self.send_job(goal_handle.request.red_num, goal_handle.request.blue_num)

        # move robot to basket
        self.pick_up_basket()
        # move robot to goal
        self.place_basket()

        # Complete the goal
        goal_handle.succeed()
        result = JobAction.Result()
        result.message = f"Calculation complete: Result ="
        result.success = True
        self.get_logger().info(f"Goal Completed: {result.success}")
        return result
    
    def pick_up_basket(self):
        request = MoveBasket.Request()
        request.message = 'pick'
        future = self.basket_client.call(request)
        response = future.result()
    
    def place_basket(self):
        request = MoveBasket.Request()
        request.message = 'place'
        future = self.basket_client.call(request)
        response = future.result()
    
    def send_job(self, red_num, blue_num):
        self.get_logger().info(f"Red: {red_num}, Blue: {blue_num}")
        goal_msg = MoveBoxes.Goal()
        goal_msg.red_num, goal_msg.blue_num = red_num, blue_num

        # 액션을 동기적으로 호출하고 Future를 반환
        future = self.job_action_client.send_goal_async(goal_msg, feedbackcallback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        return future
    
    def feedback_callback(self, feedback_msg):
        self.move_conveyor(True)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal was rejected')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        # 결과를 기다리고 처리
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result:JobAction.Result = future.result().result
        self.get_logger().info(f'Result: {result.message}')
    
    def move_conveyor(self, satus):
        request = SetBool.Request()
        request.data = satus
        future = self.order_to_conveyor.call_async(request)
        future.add_done_callback(self.set_conveyor_response_callback)

    def set_conveyor_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Success: {response.message}')
            else:
                self.get_logger().info(f'Failed: {response.message}')
        except Exception as e:
            self.get_logger().info(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    node = main_node()  # 퍼블리셔 노드 생성

    try:
        rclpy.spin(node)  # 노드 실행
    except KeyboardInterrupt:
        pass  # Ctrl+C로 종료
    finally:
        node.destroy_node()  # 노드 소멸
        rclpy.shutdown()  # ROS 2 종료

if __name__ == '__main__':
    main()
