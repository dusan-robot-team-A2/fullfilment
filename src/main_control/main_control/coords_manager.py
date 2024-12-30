from .custom_coordinate import CustomCoordinateSystem
from util.transform import create_transformation_matrix, extract_rotation_translation, transformation_3d
import cv2
import numpy as np

class CoordsManager:
    def __init__(self, g1_coor:CustomCoordinateSystem, g2_coor:CustomCoordinateSystem, g3_coor:CustomCoordinateSystem, base_coor:CustomCoordinateSystem, basket_coor:CustomCoordinateSystem, robot_coor:CustomCoordinateSystem):        
        self.global_coor = self.set_global_coor(base_coor)

        self.goals_coor = [g1_coor,g2_coor,g3_coor]
        self.base_coor = base_coor
        self.basket_coor = basket_coor
        self.robot_coor = robot_coor

        self.robot_offset = [0,0,0]
        self.goals_offset = [[0,0,10],[0,0,10],[0,0,10]]
        self.basket_offset = [0,0,0]
        self.base_offset = [0,0,10]

        self.robot_pose = None
        self.base_pose = None
        self.goal_poses = None
        self.basket_pose = None
        self.update_poses()
    
    def compute_global_coor(self, base_coor:CustomCoordinateSystem):
        t_1 = base_coor.T
        t_2 = transformation_3d(0,-np.pi/2,-np.pi/2,0,-0.4,-0.25)
        t = np.dot(t_1, t_2)
        global_r, global_tvec = extract_rotation_translation(t)
        global_rvec, _ = cv2.Rodrigues(global_r)
        
        return global_tvec, global_rvec

    def set_global_coor(self, base_coor:CustomCoordinateSystem):
        global_tvec, global_rvec = self.compute_global_coor(base_coor)
        return CustomCoordinateSystem(global_tvec, global_rvec)
    
    def update_global_coor(self):
        global_tvec, global_rvec = self.compute_global_coor(self.base_coor)
        self.global_coor.update_vec(global_tvec, global_rvec)

    def update_poses(self):
        self.update_global_coor()
        
        #robot orientation 추가 필요
        self.robot_pose = self.global_coor.get_relative_coordinates(self.robot_coor, self.robot_offset)
        self.base_pose = self.global_coor.get_relative_coordinates(self.base_coor, self.base_offset)
        self.goal_poses = [self.global_coor.get_relative_coordinates(self.goals_coor[i],self.goals_offset[i]) for i in range(len(self.goals_coor))]
        self.basket_pose = self.global_coor.get_relative_coordinates(self.basket_coor, self.basket_offset)

    def get_poses(self):
        return self.goal_poses, self.base_pose, self.robot_pose, self.basket_pose


        