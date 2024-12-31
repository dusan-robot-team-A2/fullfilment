import math
import numpy as np
from geometry_msgs.msg import Twist

class Calculator:
    def __init__(self):
        self.pick_status = False

    def calculate_angle(self, a, b, c, d):
        dot_product = a * c + b * d
        magnitude_a = math.sqrt(a**2 + b**2)
        magnitude_b = math.sqrt(c**2 + d**2)
        try:
            cos_theta = dot_product / (magnitude_a * magnitude_b)
            angle_radians = math.acos(cos_theta)
            print("상대각도를 구했습니다.")
            return angle_radians
        except ZeroDivisionError as e:
            print(f"ZeroDivisionError: {str(e)}")
            return 
    
    def calculate_distance(self, data):
        data = data.position
        distance = np.sqrt((data.x*0.01)**2 + (data.y*0.01)**2)
        print("거리를 구했습니다.")
        return distance

    def get_init_rotate(self, data):
        return self.calculate_angle(data.position.x, data.position.y , data.orientation.x, data.orientation.y)
        
    def get_target_move(self, data):
        return self.calculate_distance(data)
    
    def get_cmd_vel_rotation(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.1 # radians per second not zero
        return cmd_vel
    
    def get_cmd_vel_move(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1 # meter per second
        cmd_vel.angular.z = 0.0
        return cmd_vel

    def get_cmd_vel_back(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = -0.1 # meter per second
        cmd_vel.angular.z = 0.0
        return cmd_vel