# aruco_module.py
import cv2
import numpy as np

class Aruco:
    def __init__(self, camera_matrix, dist_coeffs):
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def detect_markers(self, image):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        return corners, ids

    def estimate_pose(self, corners):
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, self.camera_matrix, self.dist_coeffs)
        return rvecs, tvecs

    def calculate_relative_position(self, rvec1, tvec1, rvec2, tvec2):
        tvec_rel = tvec2 - tvec1
        tvec_rel = np.squeeze(tvec_rel)
        dx, dy = tvec_rel[0], tvec_rel[1]
        return dx, dy