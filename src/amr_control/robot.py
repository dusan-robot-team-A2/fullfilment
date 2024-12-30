from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node

class Robot(Node):
    def __init__(self, cmd_vel_pub):
        self.cmd_vel_pub = cmd_vel_pub

    def stop(self):
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
    
    def rotation(self, radians):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.1 # radians per second not zero
        try:
            if cmd_vel.angular.z == 0:
                raise ZeroDivisionError("Angular velocity cannot be zero.")
            self.rotation_pub(self, radians, cmd_vel)
        except ZeroDivisionError as e:
            self.cmd_vel_pub.get_logger().error(f"ZeroDivisionError: {str(e)}")

    def rotation_pub(self, radians, cmd_vel):
        duration = abs(radians) / cmd_vel.angular.z
        start_time = self.cmd_vel_pub.get_clock().now()
        while (self.cmd_vel_pub.get_clock().now() - start_time).seconds < duration:
            self.cmd_vel_pub.publish(cmd_vel)
            rclpy.spin_once(self, timeout_sec=0.1)  # 0.1초 대기 (ROS2의 이벤트 큐를 처리)
        self.stop()

    def move(self, distance):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1 # meter per second
        cmd_vel.angular.z = 0.0
        try:
            if cmd_vel.linear.x == 0:
                raise ZeroDivisionError("Linear velocity cannot be zero.")
            self.move_pub(distance, cmd_vel)
        except ZeroDivisionError as e:
            self.cmd_vel_pub.get_logger().error(f"ZeroDivisionError: {str(e)}")

    def move_pub(self, distance, cmd_vel):
        duration = distance / cmd_vel.linear.x
        start_time = self.get_clock().now()

        while self.get_clock().now() - start_time < duration:
            self.cmd_vel_pub.publish(cmd_vel)
            rclpy.spin_once(self, timeout_sec=0.1)  # 0.1초 대기 (ROS2의 이벤트 큐를 처리)
        self.stop() 