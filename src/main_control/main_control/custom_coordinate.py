import cv2
import numpy as np
from util.transform import create_transformation_matrix, extract_rotation_translation, transformation_3d

class CustomCoordinateSystem:
    def __init__(self,tvec,rvec):
        self.tvec = tvec
        self.rvec = rvec
        self.R, _ = cv2.Rodrigues(rvec)
        self.T = create_transformation_matrix(self.R, self.tvec)
    
    def get_relative_coordinates(self, other_system:'CustomCoordinateSystem', coordinate):
        trans = np.dot(self.T.T, other_system.T)

        # Ensure coordinate is a numpy array and reshape to (3,)
        coordinate = np.array(coordinate).flatten()

        # Add the homogeneous coordinate
        homogeneous_coordinate = np.append(coordinate, 1)  # Convert [x, y, z] to [x, y, z, 1]

        # Compute the relative transformation matrix
        trans = np.dot(np.linalg.inv(self.T), other_system.T)

        # Apply the transformation to the homogeneous coordinate
        transformed_homogeneous = np.dot(trans, homogeneous_coordinate)

        # Return only the x, y, z part
        return transformed_homogeneous[:3]
    
    def update_vec(self, tvec, rvec):
        self.tvec = tvec
        self.rvec = rvec
        self.R, _ = cv2.Rodrigues(rvec)
        self.T = create_transformation_matrix(self.R, self.tvec)
    