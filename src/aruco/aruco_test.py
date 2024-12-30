from .aruco import Aruco
from .camera_util import CameraUtil
from util.transform import create_transformation_matrix, transformation_3d, extract_rotation_translation
import cv2
import numpy as np

# 카메라 열기 (0번은 기본 카메라를 의미, 다른 값은 추가 카메라를 의미)
# cap = cv2.VideoCapture(4)
# 카메라가 열리지 않으면 에러 메시지 출력
# if not cap.isOpened():
#     print("Error: Could not open video stream.")
#     exit()

camera_util = CameraUtil()
mtx, dist = camera_util.get_camera_mtx_dist()

aruco = Aruco(mtx, dist)

while True:
    # 프레임 읽기
    # ret, frame = cap.read()
    frame = cv2.imread('aruco/images/image.png')

    ids, rvecs, tvecs = aruco.get_matrix(frame)
    if ids is not None:
        for i in range(len(ids)):
            # 각 마커에 대해 축 그리기
            cv2.drawFrameAxes(frame, mtx, dist, rvecs[i], tvecs[i], 0.1)

            id_text = str(ids[i][0])  # ID는 2D 배열이므로 첫 번째 값을 가져옵니다

            # tvecs[i]는 카메라 좌표계에서의 위치, 이를 이미지 좌표계로 변환하기
            img_points, _ = cv2.projectPoints(np.array([[0, 0, 0]], dtype=np.float32), rvecs[i], tvecs[i], mtx, dist)
            position = (int(img_points[0][0][0]), int(img_points[0][0][1]))  # 변환된 이미지 좌표

            # ID를 텍스트로 표시
            cv2.putText(frame, id_text, position, cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            if ids[i][0] == 60:
                r, _ = cv2.Rodrigues(rvecs[i])
                t_1 = create_transformation_matrix(r, tvecs[i])
                t_2 = transformation_3d(0,-np.pi/2,-np.pi/2,0,-0.4,-0.25)
                t = np.dot(t_1, t_2)
                base_r, base_tvec = extract_rotation_translation(t)
                base_rvec, _ = cv2.Rodrigues(base_r)

                cv2.drawFrameAxes(frame, mtx, dist, base_rvec, base_tvec, 0.2)


    
    # 처리된 프레임을 윈도우에 표시
    cv2.imshow("Frame", frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 카메라 및 윈도우 해제
# cap.release()
cv2.destroyAllWindows()
