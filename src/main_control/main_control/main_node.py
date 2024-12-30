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

class main_node(Node):
    def __init__(self):
        super().__init__('main_node')  # 노드 이름
        self.robot_status_pub = self.create_publisher(Int16, 'robot_status', 10)
        self.conveyor_status_sub = self.create_subscription(Int16, 'conveyor_status', 10, self.conveyor_status)
        self.image_pub = self.create_publisher(CompressedImage, 'global_camera', 10)

        self.get_logger().info('Publisher node has been started.')

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

        self.conveyor_service = self.create_service(SetBool, '/conveyor_control', self.conveyor_callback)
        self.order_to_conveyor = self.create_client(SetBool, '/conveyor_move')
        self.conveyor_mode = 0
        # OpenCV와 ROS 간 변환을 위한 CvBridge 초기화
        self.bridge = CvBridge()

        # 주기적인 이미지 전송을 위한 타이머 설정 (주기: 1초)
        self.timer = self.create_timer(0.1, self.publish_image)

        # OpenCV 비디오 캡처 객체 생성 (카메라 0번 장치 사용)
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        # self.cap = cv2.VideoCapture(2)
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
        self.cap.set(cv2.CAP_PROP_FPS, 25)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def publish_image(self):
        # 카메라에서 한 프레임 읽기
        ret, frame = self.cap.read()

        if ret:
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

        self.send_job(goal_handle.request.red_num, goal_handle.request.blue_num)

        self.goal_num = goal_handle.request.goal_num
        # Complete the goal
        goal_handle.succeed()
        result = JobAction.Result()
        result.message = f"Calculation complete: Result ="
        result.success = True
        self.get_logger().info(f"Goal Completed: {result.success}")
        return result
    
    def send_job(self, red_num, blue_num):
        self.get_logger().info(f"Red: {red_num}, Blue: {blue_num}")
        goal_msg = MoveBoxes.Goal()
        goal_msg.red_num, goal_msg.blue_num = red_num, blue_num

        # 액션을 동기적으로 호출하고 Future를 반환
        future = self.job_action_client.send_goal(goal_msg, feedbackcallback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        return future
    
    def feedback_callback(self, feedback_msg):
        self.move_conveyor()

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
