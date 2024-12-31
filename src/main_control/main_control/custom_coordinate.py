import cv2
import numpy as np
from util.transform import create_transformation_matrix, extract_rotation_translation, transformation_3d

class CustomCoordinateSystem:
    def __init__(self,tvec,rvec):
        self.tvec = tvec
        self.rvec = rvec
        self.R, _ = cv2.Rodrigues(rvec)
        self.T = create_transformation_matrix(self.R, self.tvec)
    
    def get_y_direction(self, other_system):
        homogeneous_coordinate = np.array([0,0.2,0,1])
        return np.linalg.multi_dot([np.linalg.inv(self.T), other_system.T, homogeneous_coordinate])[:3]
    
    def get_relative_coordinates(self, other_system:'CustomCoordinateSystem', coordinate):
        homogeneous_coordinate = np.array([0,0,0,1])

        # Ensure coordinate is a numpy array and reshape to (3,)
        coordinate = np.array(coordinate).flatten()

        # Translation
        translation = np.array([
            [1, 0, 0, coordinate[0]],
            [0, 1, 0, coordinate[1]],
            [0, 0, 1, coordinate[2]],
            [0, 0, 0, 1]
        ])

        # Return only the x, y, z part
        return np.linalg.multi_dot([translation, np.linalg.inv(self.T), other_system.T, homogeneous_coordinate])[:3]
    
    def update_vec(self, tvec, rvec):
        self.tvec = tvec
        self.rvec = rvec
        self.R, _ = cv2.Rodrigues(rvec)
        self.T = create_transformation_matrix(self.R, self.tvec)
    