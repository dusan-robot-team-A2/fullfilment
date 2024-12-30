from .custom_coordinate import CustomCoordinateSystem
from util.transform import create_transformation_matrix, extract_rotation_translation, transformation_3d
import cv2
import numpy as np

class CoordsManager:
    def __init__(self):        
        self.global_coor:CustomCoordinateSystem = None

        self.goals_coor:list[CustomCoordinateSystem] = [None,None,None]
        self.base_coor:CustomCoordinateSystem = None
        # self.basket_coor = basket_coor
        self.robot_coor:CustomCoordinateSystem = None

        self.robot_offset = [0,0,0]
        self.goals_offset = [[0.4,0,0],[0.4,0,0],[0.4,0,0]]
        self.basket_offset = [0,0,0]
        self.base_offset = [0,-0.4,0]

        self.robot_pose = None
        self.base_pose = None
        self.goal_poses = [None,None,None]
        self.basket_pose = None
    
    def set_base_coor(self, base_coor):
        if self.global_coor is None:
            return
        self.base_coor = base_coor
        self.global_coor = self.set_global_coor(base_coor)
        # self.base_pose = self.global_coor.get_relative_coordinates(self.base_coor, self.base_offset)
    
    def set_goal_coor(self, goal_coor, num):
        if self.global_coor is None:
            return
        self.goals_coor[num] = goal_coor
        self.goal_poses[num] = self.global_coor.get_relative_coordinates(self.goals_coor[num], self.goals_offset[num])
    
    def set_robot_coor(self, robot_coor):
        if self.global_coor is None:
            return
        self.robot_coor = robot_coor
        self.robot_pose = self.global_coor.get_relative_coordinates(self.robot_coor, self.robot_offset)

    def update_base_coor(self, rvec, tvec):
        self.base_coor.update_vec(rvec=rvec, tvec=tvec)
        self.base_pose = self.global_coor.get_relative_coordinates(self.base_coor, self.base_offset)     
    
    def update_goal_coor(self, rvec, tvec, num):
        self.goals_coor[num].update_vec(rvec=rvec, tvec=tvec)
        self.goal_poses[num] = self.global_coor.get_relative_coordinates(self.goals_coor[num], self.goals_offset[num])
    
    def update_robot_coor(self, rvec, tvec):
        self.robot_coor.update_vec(rvec=rvec, tvec=tvec)
        self.robot_pose = self.global_coor.get_relative_coordinates(self.robot_coor, self.robot_offset)
    
    # def compute_global_coor(self, base_coor:CustomCoordinateSystem):
    #     t_1 = base_coor.T
    #     t_2 = transformation_3d(0,-np.pi/2,-np.pi/2,0,-0.4,-0.25)
    #     t = np.dot(t_1, t_2)
    #     global_r, global_tvec = extract_rotation_translation(t)
    #     global_rvec, _ = cv2.Rodrigues(global_r)
        
    #     return global_tvec, global_rvec

    def set_global_coor(self, global_coor):
        self.global_coor = global_coor
    
    def update_global_coor(self, rvec, tvec):
        self.global_coor.update_vec(rvec=rvec, tvec=tvec)

    def get_poses(self):
        return (*self.goal_poses, self.base_pose, self.robot_pose)

    def draw_axis_global(self,frame, mtx, dist):
        rvec, tvec = self.global_coor.rvec, self.global_coor.tvec
        cv2.drawFrameAxes(frame, mtx, dist, rvec, tvec, 0.1)
    
    def draw_dot_goal_poses(self, frame, mtx, dist):
        rvec, tvec = self.global_coor.rvec, self.global_coor.tvec
        for goal_pose in self.goal_poses:
            if goal_pose is None:
                continue
            goal_pose[2] = 0
            img_points, _ = cv2.projectPoints(goal_pose.reshape(1,3), rvec, tvec, mtx, dist)
            position = (int(img_points[0][0][0]), int(img_points[0][0][1]))  # 변환된 이미지 좌표
            cv2.circle(frame, position, 5, (0, 255, 0), -1)

    def draw_base_pose(self, frame, mtx,dist):
        if self.base_pose is None:
            return
        self.base_pose[2] = 0
        rvec, tvec = self.global_coor.rvec, self.global_coor.tvec
        img_points, _ = cv2.projectPoints(self.base_pose.reshape(1,3), rvec, tvec, mtx, dist)
        position = (int(img_points[0][0][0]), int(img_points[0][0][1]))  # 변환된 이미지 좌표
        cv2.circle(frame, position, 5, (0, 255, 0), -1)
    
    def draw_robot_pose(self, frame, mtx, dist):
        if self.robot_pose is None:
            return
        self.robot_pose[2] = 0
        rvec, tvec = self.global_coor.rvec, self.global_coor.tvec
        img_points, _ = cv2.projectPoints(self.robot_pose.reshape(1,3), rvec, tvec, mtx, dist)
        position = (int(img_points[0][0][0]), int(img_points[0][0][1]))  # 변환된 이미지 좌표
        cv2.circle(frame, position, 5, (0, 255, 0), -1)

    def draw_frame(self, frame,mtx, dist):
        if self.global_coor is None:
            return
        self.draw_axis_global(frame, mtx,dist)
        self.draw_dot_goal_poses(frame, mtx, dist)
        self.draw_base_pose(frame,mtx,dist)
        self.draw_robot_pose(frame,mtx,dist)

        