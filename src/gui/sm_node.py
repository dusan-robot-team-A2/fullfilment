import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int16
from std_srvs.srv import SetBool
from sensor_msgs.msg import Image, CompressedImage
# from cv_bridge import CvBridge
from rclpy.action import ActionClient
from PyQt5.QtCore import pyqtSignal, QVariant


from fullfilment_interfaces.action import JobAction
from geometry_msgs.msg import Point
# from .app import MainWindow
from .mail_management import send_email

import time


class SystemMonitoringNode(Node):
    def __init__(self):
        super().__init__('data_subscriber')

        #init_variables
        self.last_global_image_received_time = time.time()
        self.mail_send_status = False
        self.init_variables()

        # robot status
        # 0: Ready, 1: Move, 2: Pick and Place
        self.robot_status_subscription = self.create_subscription(
            Int16,
            'robot_status',
            self.robot_status_callback,
            10
        )

        # conveyorbelt status
        self.conveyor_status_subscription = self.create_subscription(
            Bool,
            'conveyor_status',
            self.conveyor_status_callback,
            10
        )

        # subscribe compressed image
        self.global_image_subscribe = self.create_subscription(
            CompressedImage,
            # '/image_raw/compressed',  # 토픽 이름
            'global_camera',
            self.global_image_callback,
            10
        )

        self.job_command_action = ActionClient(self, JobAction, 'job_command')
        self.conveyor_client = self.create_client(SetBool, 'conveyor_control')

        self.check_camera_connection_timer = self.create_timer(1.0, self.check_camera_connection_timer_callback)

        # load to main_window initial status
        # self.update_variables()
        self.bridge = None

    def init_qthread_variables(self, bridge, robot_status_signal, conveyor_status_signal, global_cam_signal, robot_cam_signal):
        self.bridge = bridge
        self.robot_status_signal = robot_status_signal
        self.conveyor_status_signal = conveyor_status_signal
        self.global_cam_signal = global_cam_signal
        self.robot_cam_signal = robot_cam_signal
        

    def init_variables(self):
        self.robot_status = 0
        self.conveyor_status = False
        self.robot_image = None

    def update_variables(self):
        self.robot_status_signal.emit(self.robot_status)
        self.conveyor_status_signal.emit(self.conveyor_status)

    def robot_status_callback(self, msg:Int16):
        status = msg.data
        
        if status != self.robot_status:
            self.robot_status = status
            self.get_logger().info(f"Robot Status changed to {status}")

            self.robot_status_signal.emit(self.robot_status)
    
    def conveyor_status_callback(self, msg:Bool):
        status = msg.data

        if status != self.conveyor_status:
            self.conveyor_status = status
            self.get_logger().info(f"Conveyor Status changed to {status}")

            self.conveyor_status_signal.emit(self.conveyor_status)

    def global_image_callback(self, msg:CompressedImage):
        if self.bridge is None:
            return
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.global_cam_signal.emit(cv_image)
        self.last_global_image_received_time = time.time()
        
        self.mail_send_status = False

        #임시
        self.robot_image = cv_image

    def send_job(self, red_num, blue_num, goal_num):
        self.get_logger().info(f"Red: {red_num}, Blue: {blue_num}, Goal: {goal_num}")
        goal_msg = JobAction.Goal()
        goal_msg.red_num, goal_msg.blue_num, goal_msg.goal_num = red_num, blue_num, goal_num

        # 액션을 비동기적으로 호출하고 Future를 반환
        future = self.job_command_action.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)
        return future

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
        if self.robot_image is not None:
            self.robot_cam_signal.emit(self.robot_image)

    def send_conveyor_request(self, data):
        # SetBool 요청 메시지 생성
        request = SetBool.Request()
        request.data = data

        # 서비스 호출 및 응답 대기
        future = self.conveyor_client.call_async(request)
        future.add_done_callback(self.set_conveyor_response_callback)
        self.get_logger().info(f'Conveyor Request Status: {data}')

    def set_conveyor_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'Success: {response.message}')
            else:
                self.get_logger().info(f'Failed: {response.message}')
        except Exception as e:
            self.get_logger().info(f'Service call failed: {e}')

    def check_camera_connection_timer_callback(self):
        # 현재 시간과 마지막 메시지 수신 시간을 비교
        if time.time() - self.last_global_image_received_time > 5.0:  # 5초 동안 메시지를 수신하지 않으면
            self.get_logger().warn("No message received for 5 seconds!")
            if self.mail_send_status == False:
                send_email("tysong2000@naver.com", "[Error]카메라 작동이 중지됨", "카메라 작동이 중단되었습니다 확인바랍니다")
                self.mail_send_status = True



def main(args=None):
    rclpy.init(args=args)  # ROS 2 초기화
    node = SystemMonitoringNode()  # 퍼블리셔 노드 생성

    try:
        rclpy.spin(node)  # 노드 실행
    except KeyboardInterrupt:
        pass  # Ctrl+C로 종료
    finally:
        node.destroy_node()  # 노드 소멸
        rclpy.shutdown()  # ROS 2 종료

if __name__ == '__main__':
    main()
