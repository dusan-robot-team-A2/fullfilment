import cv2
import numpy as np

class camera_undistortion:
    def undistortion(img):
        mtx = np.array([
            1.411178332933327965e+03,0.000000000000000000e+00,5.657066461642494914e+02,
            0.000000000000000000e+00,1.404459077346332606e+03,3.404933382879391388e+02,
            0.000000000000000000e+00,0.000000000000000000e+00,1.000000000000000000e+00,
        ])

        dist = np.array([6.131983532253498098e-02,2.887928520306218272e-01,-7.355652692601365866e-03,-1.182111798112949440e-02,-1.211195517419898948e+00])

        # 보정된 이미지 크기 (원본 이미지 크기 사용)
        h, w = img.shape[:2]

        # 보정된 카메라 행렬과 왜곡 계수를 사용하여 새 카메라 행렬 계산
        new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

        # 왜곡 보정 수행
        dst = cv2.undistort(img, mtx, dist, None, new_mtx)
        return dst


def main():
    img = cv2.imread('src/turtlebot3_manipulator/turtlebot3_manipulator/capture_0.png')
    img = camera_undistortion.undistortion(img)
    cv2.imshow('Undistorted Image', img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()