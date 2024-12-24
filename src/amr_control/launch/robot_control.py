# turtlebot_controller.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np
from pid import PID  # PID 모듈을 임포트합니다.

class TurtleBotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        # create ROS communication
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Float32MultiArray, 'aruco_relative_position', self.position_callback, 10)
        
        # PID controllers for linear and angular velocity
        self.linear_pid = PID(1.0, 0.1, 0.05)
        self.angular_pid = PID(1.0, 0.1, 0.05)
        
        self.target_distance = 0.4   # Target distance from the marker
        self.target_angle = 0.0      # Target angle to face the marker

        self.target_reached = False  # Flag to check if target position is reached
        self.angle_aligned = False   # Flag to check if alignment is complete

    def position_callback(self, msg):
        dx, dy = msg.data
        distance = np.sqrt(dx**2 + dy**2)
        angle = np.arctan2(dy, dx)

        if not self.target_reached:
            self.move_to_position(distance)
        elif not self.angle_aligned:
            self.align_to_marker(angle)
        else:
            # manifulator
            pass

    def move_to_position(self, distance):
        self.linear_pid.setpoint = self.target_distance
        linear_velocity = self.linear_pid.update(distance)

        cmd_vel = Twist()
        cmd_vel.linear.x = linear_velocity
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)

        if abs(distance - self.target_distance) < 0.01:
            self.target_reached = True
            self.stop_robot()

    def align_to_marker(self, angle):
        self.angular_pid.setpoint = self.target_angle
        angular_velocity = self.angular_pid.update(angle)

        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = angular_velocity
        self.cmd_vel_pub.publish(cmd_vel)

        if abs(angle - self.target_angle) < 0.01:
            self.angle_aligned = True
            self.stop_robot()

    def stop_robot(self):
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBotController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
