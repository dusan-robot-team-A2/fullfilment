import numpy as np
import cv2 as cv
import glob
import os

class CameraUtil:
    def __init__(self):
        self.mtx = np.loadtxt('aruco/images/global_cam/camera_matrix.txt', delimiter=",", skiprows=1)
        self.dist = np.loadtxt('aruco/images/global_cam/dist_coeffs.txt', delimiter=",", skiprows=1)

    def return_undistort_image(self, img):
        img = cv.imread('./images/image.png')
        h,  w = img.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(self.mtx, self.dist, (w,h), 1, (w,h))


        # undistort
        mapx, mapy = cv.initUndistortRectifyMap(self.mtx, self.dist, None, newcameramtx, (w,h), 5)
        dst = cv.remap(img, mapx, mapy, cv.INTER_LINEAR)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        return dst

    def get_camera_mtx_dist(self):
        return self.mtx, self.dist

if __name__ == '__main__':
    camera_util = CameraUtil()
    img = cv.imread('./images/image.png')
    dst = camera_util.return_undistort_image(img)
    cv.imwrite('calibresult.png', dst)