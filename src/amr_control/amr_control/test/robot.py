from geometry_msgs.msg import Twist
import rclpy
from velocity_calculator import Calculator
from geometry_msgs.msg import Vector3


class Robot():
    def __init__(self,node):
        self.cmd_vel_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        self.init_aligned = False
        self.angular_now = Vector3(x=1.0, y=0.0, z=0.0)
        self.calculator = Calculator()
        self.node = node

    def update_move_status(self):
        self.init_aligned = not self.init_aligned
    
    def update_now_rotation_matrix(self, data):
        self.angular_now.x = data.orientation.x
        self.angular_now.y = data.orientation.y

    def calculate_angle(self, data):
        return self.calculator.calculate_angle(self.angular_now.x, self.angular_now.y,data.orientation.x, data.orientation.y)
    
    def get_velocity(self, data):
        if not self.init_aligned:
            return self.calculator.get_init_rotate(data)
        else:
            return self.calculator.get_target_move(data)
        
    def stop(self):
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        print("멈춥니다.")

    def rotation(self, radians, data):
        cmd_vel = self.calculator.get_cmd_vel_rotation()
        try:
            if cmd_vel.angular.z == 0:
                raise ZeroDivisionError("Angular velocity cannot be zero.")
            self.rotation_pub(radians, cmd_vel, data)
        except ZeroDivisionError as e:
            self.get_logger().error(f"ZeroDivisionError: {str(e)}")

    def rotation_pub(self, radians, cmd_vel, data):
        duration = (abs(radians) / cmd_vel.angular.z) * 1e9
        start_time = self.node.get_clock().now()
        while (self.node.get_clock().now() - start_time).nanoseconds < duration:
            self.cmd_vel_pub.publish(cmd_vel)
            print("각속도를 발행합니다.")
            rclpy.spin_once(self.node, timeout_sec=0.1)  # 0.1초 대기 (ROS2의 이벤트 큐를 처리)
        self.stop()
        self.update_move_status()
        self.update_now_rotation_matrix(data)

    def move(self, distance):
        cmd_vel = self.calculator.get_cmd_vel_move()
        try:
            if cmd_vel.linear.x == 0:
                raise ZeroDivisionError("Linear velocity cannot be zero.")
            self.move_pub(distance, cmd_vel)
        except ZeroDivisionError as e:
            self.get_logger().error(f"ZeroDivisionError: {str(e)}")

    def move_pub(self, distance, cmd_vel):
        duration = (distance / cmd_vel.linear.x) * 1e9
        start_time = self.node.get_clock().now()

        while (self.node.get_clock().now() - start_time).nanoseconds < duration:
            self.cmd_vel_pub.publish(cmd_vel)
            print("선속도를 발행합니다.")
            rclpy.spin_once(self.node, timeout_sec=0.1)  # 0.1초 대기 (ROS2의 이벤트 큐를 처리)
        self.stop()
        self.update_move_status()