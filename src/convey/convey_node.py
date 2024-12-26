import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from example_interfaces.srv import SetBool  # ROS 2 서비스 메시지 타입
import serial
import os
import time

class ConveyorNode(Node):
    def __init__(self):
        super().__init__('conveyor_node')        # ROS 2 Publisher 설정 (상태 발행 및 알림)
        self.status_publisher = self.create_publisher(Int16, 'conveyor_status', 10)
        self.command_service = self.create_service(
            SetBool, 
            '/conveyor_move', 
            self.handle_command
            )        # 상태 변수
        self.current_status = "Idle"  # 현재 상태
        self.previous_status = None  # 이전 상태 (상태 변화 감지용)        # 타이머 설정
        self.timer = self.create_timer(0.1, self.publish_status)  # 상태 발행 주기
        self.usb_check_timer = self.create_timer(1.0, self.check_usb_connection)  # USB 연결 상태 확인 주기        # 시리얼 포트 설정 (아두이노 연결)
        self.serial_port = self.initialize_serial('/dev/ttyACM0', 115200)    
        
    def initialize_serial(self, port, baudrate):
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
                time.sleep(5)    
                
    def check_usb_connection(self):
        """USB 연결 상태를 주기적으로 확인."""
        port = '/dev/ttyACM0'
        if not os.path.exists(port):  # 포트가 존재하지 않으면 연결 끊김
            self.get_logger().error("USB port disconnected or Arduino powered off.")
            self.current_status = 2
            self.serial_port = None  # 시리얼 포트 해제
        else:
            if self.serial_port is None or not self.serial_port.is_open:
                self.get_logger().info("Reconnecting to USB port...")
                self.serial_port = self.initialize_serial(port, 115200)    
                
    def handle_command(self, request, response):
        try:
            if self.serial_port is None:
                raise serial.SerialException("Serial port is not connected.")            
            command = "10000" if request.data else "1"
            self.serial_port.write(f"{command}\n".encode())  # 명령 전송
            self.get_logger().info(f"Received command: {command}")            # 상태 업데이트
            self.current_status = 1 if request.data else 0            # 서비스 응답
            response.success = True
            response.message = f"Conveyor command '{command}' executed."
            return response
        
        except serial.SerialException as e:
            self.get_logger().error(f"Error sending command: {e}")
            self.current_status = 2
            response.success = False
            response.message = "Failed to execute conveyor command due to serial error."
            return response    
        
    def publish_status(self):
        """현재 상태를 ROS 2 토픽으로 발행."""
        try:
            msg = Int16()
            msg.data = self.current_status
            self.status_publisher.publish(msg)            # 상태 변화 감지 및 알림 발행
            if self.current_status != self.previous_status:
                self.previous_status = self.current_status
        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")    
            
            
def main(args=None):
    rclpy.init(args=args)
    node = ConveyorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
