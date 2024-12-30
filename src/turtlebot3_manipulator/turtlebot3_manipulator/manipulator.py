import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient, ActionServer
from control_msgs.action import GripperCommand
from ultralytics import YOLO
from sensor_msgs.msg import CompressedImage
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
from fullfilment_interfaces.action import MoveBoxes
from fullfilment_interfaces.srv import MoveBasket
from visual_kinematics.RobotSerial import *
from std_msgs.msg import Header
import time
import math
import cv2
from .imagesaver import ImageSaver
from .camera_undistortion import camera_undistortion
from std_srvs.srv import SetBool
from random import randint

class manipulator(Node):
    def __init__(self):
        super().__init__("manipulator")
        self.gripper_action_client = ActionClient(self, GripperCommand, 'gripper_controller/gripper_cmd')
        self.joint_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.get_image = self.create_subscription(CompressedImage, 'rgb_image/compressed_image', self.image_callback, 10)
        self.job_server = ActionServer(self, MoveBoxes, 'job_command2amr', self.moveboxes_callback)
        self. basket_service = self.create_service(MoveBasket, 'move_basket', self.basket_callback)
        # self.gripper_client = self.create_client(SetBool, '/gripper_control')

        self.image_saver = ImageSaver()
        self.cam_setting = camera_undistortion()
        self.label_dic = {0: "blue", 1: "purple", 2: "red"}
        self.move_manipulator(100, 0, 0)

    def basket_callback(self, request, response):
        if request.message == 'pick':
            self.pick_up_basket()
            response.success = True
            return response
        else:
            self.place_basket()
            response.success = True
            return response

    def image_callback(self, img):
        np_arr = np.frombuffer(img.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # Decode to color image
        self.image = image_np
        cv2.imshow('', self.image)
        cv2.waitKey(1)

    def save_image(self, cnt):
        for i in range(cnt):
            self.move_manipulator(randint(50, 100), randint(-50, 50), randint(50, 100))
            num = self.image_saver.save_imgae(self.image, i + num)

    def detect_object(self, img):
        image = self.cam_setting.undistortion(img)
        model = YOLO("/home/pgt/doosan/fullfilment/src/turtlebot3_manipulator/turtlebot3_manipulator/best.pt")
        result = model.track(image, persist=True)[0]
        data = result.boxes
        print(data)
        cv2.imshow('', image)
        return data

    def get_object_pos(self):
        data = self.detect_object(self.image)
        x_center = 640
        y_center = 90
        red_lst = []
        blue_lst = []
        # Get the boxes and track IDs
        class_ids = data.cls
        boxes = data.xywh.cpu()
        track_ids = data.id.int().cpu().tolist()
        confidences = data.conf.cpu().tolist()
        # Plot the tracks
        for box, class_id, track_id, confidence in zip(boxes, class_ids, track_ids, confidences):
            if confidence < 0.5:
                continue
            x, y, w, h = box
            if self.label_dic[int(class_id)] == 'red':
                red_lst.append([self.label_dic[int(class_id)], float((y_center - (y + (h / 2))) * 0.01), float((x_center - (x + (w / 2))) * 0.12)])
            elif self.label_dic[int(class_id)] == 'blue':
                blue_lst.append([self.label_dic[int(class_id)], float((y_center - (y + (h / 2))) * 0.01), float((x_center - (x + (w / 2))) * 0.12)])
            else:
                return [self.label_dic[class_id], float((y_center - (y + (h / 2))) * 0.01), float((x_center - (x + (w / 2))) * 0.12)]
        return red_lst, blue_lst
    
    def check_object(self):
        self.move_manipulator(100, 0, 120)
        return self.get_object_pos()
    
    def safty_pick(self, x, y, z):
        self.move_manipulator(x, y, z + 20)
        self.move_manipulator(x, y, z)
        self.grip()

    def pick(self, x, y, z):
        print(x, y, z)
        self.move_manipulator(x, y, z + 20)
        self.move_manipulator(x, y, z)
        self.grip()
        self.move_manipulator(x, y, z + 30)

    def place(self, x, y, z):
        self.move_manipulator(x, y, z + 20)
        self.move_manipulator(x, y, z)
        self.release()
        self.move_manipulator(x, y, z + 30)

    def moveboxes_callback(self, goal):
        self.release()
        feedback_msg = MoveBoxes.Feedback()
        red_lst, blue_lst = self.check_object()
        print(red_lst, blue_lst, end='\n')
        red = goal.request.red_num
        blue = goal.request.blue_num
        for i in range(red):
            self.pick(100 + red_lst[i][1], 5 + red_lst[i][2], -25)
            print(red_lst[i][1], red_lst[i][2])
            self.place(0, -200, 5)
            self.move_manipulator(100, 0, 0)
            feedback_msg.go = True
            goal.publish_feedback(feedback_msg)
        
        for i in range(blue):
            self.pick(103 + blue_lst[i][1], 0 + blue_lst[i][2], -25)
            self.place(0, -200, 5)
            self.move_manipulator(100, 0, 0)
            feedback_msg.go = True
            goal.publish_feedback(feedback_msg)
        result = MoveBoxes.Result()
        result.success = True
        return result
    
    def pick_up_basket(self):
        pos_lst = self.check_object()
        self.safty_pick(pos_lst[1], pos_lst[2], -20)
        self.move_manipulator(pos_lst[1], pos_lst[2] - 30, -20)
        self.move_manipulator(pos_lst[1], pos_lst[2] - 30, -20)
        self.move_manipulator(pos_lst[1], pos_lst[2] - 30, 10)

    def place_basket(self):
        self.place(100, 0, -5)

    def solv2(self, r1, r2, r3):
        d1 = (r3**2 - r2**2 + r1**2) / (2*r3)
        d2 = (r3**2 + r2**2 - r1**2) / (2*r3)

        s1 = math.acos(d1 / r1)
        s2 = math.acos(d2 / r2)

        return s1, s2
    
    # sr1 : angle between z-axis to J0->J1  sr2 : angle between J0->J1 to J1->J2  sr3 : angle between J1->J2 to J2->J3
    def solv_robot_arm2(self, x, y, z):
        j1_z_offset = 77
        r1 = 130                       # r1 : distance J0 to J1    
        r2 = 124                       # r2 : distance J1 to J2
        r3 = 150                       # r3 : distance J0 to J2

        z = z + r3 - j1_z_offset

        Rt = math.sqrt(x**2 + y**2 + z**2)
        Rxy = math.sqrt(x**2 + y**2)
        St = math.asin(z / Rt)
        #   Sxy = np.arccos(x / Rxy)
        Sxy = math.atan2(y, x)

        s1, s2 = self.solv2(r1, r2, Rt)

        sr1 = math.pi/2 - (s1 + St)
        sr2 = s1 + s2
        sr2_ = sr1 + sr2
        sr3 = math.pi - sr2_

        return Sxy, sr1, sr2, sr3, St, Rt

    def grip(self):
        self.send_gripper_goal(-0.015)
        time.sleep(1)
        # request = SetBool.Request()
        # request.data = True
        # future = self.gripper_client.call(request)

    def release(self):
        self.send_gripper_goal(0.005)
        time.sleep(1)
        # request = SetBool.Request()
        # request.data = False
        # future = self.gripper_client.call(request)

    def send_gripper_goal(self, position):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = -1.0
    
        if not self.gripper_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Gripper action server not available!")
            return
        
        self.gripper_action_client.send_goal_async(goal)

    def move_manipulator(self, x, y, z):
        th1_offset = - math.atan2(0.024, 0.128)
        th2_offset = - 0.5*math.pi - th1_offset

        Sxy, sr1, sr2, sr3, St, Rt = self.solv_robot_arm2(x, y, z)
        trajectory_msg = JointTrajectory()
        trajectory_msg.header = Header()
        trajectory_msg.header.frame_id = ''
        trajectory_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4']

        point = JointTrajectoryPoint()
        point.positions = [Sxy, sr1 + th1_offset, sr2 + th2_offset, sr3]
        point.velocities = [0.0] * 4
        point.accelerations = [0.0] * 4
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 500

        trajectory_msg.points = [point]

        self.joint_pub.publish(trajectory_msg)
        time.sleep(2)

def main(args = None):
    rclpy.init(args=args)
    node = manipulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__== "__main__": 
	main()