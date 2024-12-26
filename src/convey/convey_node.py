import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from example_interfaces.srv import SetBool  # ROS 2 서비스 메시지 타입
import serial
import os
import timeclass ConveyorNode(Node):
    def __init__(self):
        super().__init__('conveyor_node')        # ROS 2 Publisher 설정 (상태 발행 및 알림)
        self.status_publisher = self.create_publisher(String, 'conveyor_status', 10)
        self.alert_publisher = self.create_publisher(String, 'conveyor_alert', 10)        # ROS 2 Service 설정 (메인 노드에서 명령 수신)
        self.command_service = self.create_service(
            SetBool, 'conveyor_command', self.handle_command)        # 상태 변수
        self.current_status = "Idle"  # 현재 상태
        self.previous_status = None  # 이전 상태 (상태 변화 감지용)        # 타이머 설정
        self.timer = self.create_timer(0.1, self.publish_status)  # 상태 발행 주기
        self.usb_check_timer = self.create_timer(1.0, self.check_usb_connection)  # USB 연결 상태 확인 주기        # 시리얼 포트 설정 (아두이노 연결)
        self.serial_port = self.initialize_serial('/dev/ttyACM0', 115200)    def initialize_serial(self, port, baudrate):
        """시리얼 포트를 초기화하고 연결 실패 시 재시도."""
        while True:
            try:
                self.get_logger().info(f"Trying to connect to {port}...")
                serial_port = serial.Serial(port, baudrate, timeout=1)
                self.get_logger().info(f"Connected to {port}")
                return serial_port
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to connect to serial port: {e}")
                self.get_logger().info("Retrying in 5 seconds...")
                time.sleep(5)    def check_usb_connection(self):
        """USB 연결 상태를 주기적으로 확인."""
        port = '/dev/ttyACM0'
        if not os.path.exists(port):  # 포트가 존재하지 않으면 연결 끊김
            self.get_logger().error("USB port disconnected or Arduino powered off.")
            self.current_status = "Error: USB Port Disconnected"
            self.send_alert(emergency=True)
            self.serial_port = None  # 시리얼 포트 해제
        else:
            if self.serial_port is None or not self.serial_port.is_open:
                self.get_logger().info("Reconnecting to USB port...")
                self.serial_port = self.initialize_serial(port, 115200)    def handle_command(self, request, response):
        """
        메인 노드에서 명령을 수신하고 시리얼로 전송.
        - True: '움직임' (Start)
        - False: '멈춤' (Stop)
        """
        try:
            if self.serial_port is None:
                raise serial.SerialException("Serial port is not connected.")            command = "Start" if request.data else "Stop"
            self.serial_port.write(f"{command}\n".encode())  # 명령 전송
            self.get_logger().info(f"Received command: {command}")            # 상태 업데이트
            self.current_status = "Running" if request.data else "Stopped"            # 서비스 응답
            response.success = True
            response.message = f"Conveyor command '{command}' executed."
            return response
        except serial.SerialException as e:
            self.get_logger().error(f"Error sending command: {e}")
            self.current_status = "Error: Serial Disconnected"
            self.send_alert(emergency=True)
            response.success = False
            response.message = "Failed to execute conveyor command due to serial error."
            return response    def publish_status(self):
        """현재 상태를 ROS 2 토픽으로 발행."""
        try:
            msg = String()
            msg.data = self.current_status
            self.status_publisher.publish(msg)            # 상태 변화 감지 및 알림 발행
            if self.current_status != self.previous_status:
                self.previous_status = self.current_status
                self.send_alert()            # 상태 로그 출력
            self.get_logger().info(f"Conveyor status: {self.current_status}")
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")    def send_alert(self, emergency=False):
        """상태 변화가 감지되었거나 긴급상황일 때 알림 발행."""
        try:
            alert_msg = String()
            if emergency:
                alert_msg.data = f"EMERGENCY: {self.current_status}"
            else:
                alert_msg.data = f"State changed to: {self.current_status}"
            self.alert_publisher.publish(alert_msg)            # 알림 로그 출력
            self.get_logger().info(f"Alert sent: {alert_msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error sending alert: {e}")def main(args=None):
    rclpy.init(args=args)
    node = ConveyorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()if __name__ == '__main__':
    main()
