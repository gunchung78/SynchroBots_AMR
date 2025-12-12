# coding=utf8
import numpy as np
import math
import time
import cv2
import cv2.aruco as aruco

from std_msgs.msg import Int8
from geometry_msgs.msg import Twist

print("find video0")
cam_id = 0

cam = cv2.VideoCapture(cam_id)

dist_coeffs = np.array(
    [[3.41360787e-01, -2.52114260e+00, -1.28012469e-03, 6.70503562e-03, 2.57018000e+00]]
)
print(dist_coeffs)

font = cv2.FONT_HERSHEY_SIMPLEX
ret, frame = cam.read()
width = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
count = cam.get(cv2.CAP_PROP_FRAME_COUNT)
fps = cam.get(cv2.CAP_PROP_FPS)
size = frame.shape
focal_length = size[1]
center = (size[1] / 2, size[0] / 2)

camera_matrix = np.array(
    [[focal_length, 0, center[0]],
     [0, focal_length, center[1]],
     [0, 0, 1]],
    dtype="double",
)

print(camera_matrix, dist_coeffs)

cv2.namedWindow("show", 0)

marker_length = 0.032

R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

pose_data = [None, None, None, None, None, None]
_id = [0]
pose_data_dict = {}

x = 0
y = 0
theta = 0


def _is_rotation_matrix(R):
    Rt = np.transpose(R)
    should_be_identity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - should_be_identity)
    return n < 1e-6


def _rotation_matrix_to_euler_angles(R):
    assert _is_rotation_matrix(R)

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x_ = math.atan2(R[2, 1], R[2, 2])
        y_ = math.atan2(-R[2, 0], sy)
        z_ = math.atan2(R[1, 0], R[0, 0])
    else:
        x_ = math.atan2(-R[1, 2], R[1, 1])
        y_ = math.atan2(-R[2, 0], sy)
        z_ = 0.0

    return np.array([x_, y_, z_])


def _detect(corners, ids, img_with_aruco):
    global pose_data

    if len(corners) > 0:
        pts = corners[0][0]

        x1 = (int(pts[0][0]), int(pts[0][1]))
        x2 = (int(pts[1][0]), int(pts[1][1]))
        x3 = (int(pts[2][0]), int(pts[2][1]))
        x4 = (int(pts[3][0]), int(pts[3][1]))

        cv2.line(img_with_aruco, x1, x2, (255, 0, 0), 1)
        cv2.line(img_with_aruco, x2, x3, (255, 0, 0), 1)
        cv2.line(img_with_aruco, x3, x4, (255, 0, 0), 1)
        cv2.line(img_with_aruco, x4, x1, (255, 0, 0), 1)

        font_local = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img_with_aruco, "C1", x1, font_local, 1, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img_with_aruco, "C2", x2, font_local, 1, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img_with_aruco, "C3", x3, font_local, 1, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img_with_aruco, "C4", x4, font_local, 1, (255, 255, 255), 1, cv2.LINE_AA)

        if ids is not None:
            img_pts = pts.astype(np.float32)

            half = marker_length / 2.0
            obj_pts = np.array(
                [
                    [-half,  half, 0.0],
                    [ half,  half, 0.0],
                    [ half, -half, 0.0],
                    [-half, -half, 0.0],
                ],
                dtype=np.float32,
            )

            ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_matrix, dist_coeffs)
            if not ok:
                pose_data[0] = None
                pose_data[1] = None
                pose_data[2] = None
                pose_data[3] = None
                pose_data_dict[ids] = pose_data
                return None

            corner_mid = (
                int((x1[0] + x2[0] + x3[0] + x4[0]) / 4),
                int((x1[1] + x2[1] + x3[1] + x4[1]) / 4),
            )

            cv2.putText(
                img_with_aruco,
                "id=" + str(ids),
                corner_mid,
                font_local,
                1,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

            R_ct, _ = cv2.Rodrigues(rvec)
            R_tc = np.matrix(R_ct).T

            roll_marker, pitch_marker, yaw_marker = _rotation_matrix_to_euler_angles(
                R_flip * R_tc
            )

            t = tvec.reshape(3)

            pose_data[0] = t[0] * 100.0
            pose_data[1] = t[1] * 100.0
            pose_data[2] = t[2] * 100.0
            pose_data[3] = math.degrees(roll_marker)
            pose_data[4] = math.degrees(pitch_marker)
            pose_data[5] = math.degrees(yaw_marker)

            pose_data_dict[ids] = pose_data

            roll_deg = pose_data[3]
            pitch_deg = pose_data[4]
            yaw_deg = pose_data[5]

            if abs(yaw_deg) % 90.0 < 30.0:
                return [
                    pose_data[0],
                    pose_data[1],
                    pose_data[2],
                    roll_deg,
                    pitch_deg,
                    yaw_deg,
                    corner_mid,
                ]
            else:
                return None

    pose_data[0] = None
    pose_data[1] = None
    pose_data[2] = None
    pose_data[3] = None
    pose_data_dict[0] = pose_data
    return None


def displayFrame(frame_input):
    cv2.imshow("show", frame_input)
    cv2.waitKey(1)


def getArucoCode(display_mode=True):
    ret, frame_local = cam.read()
    if not ret:
        return None

    gray = cv2.cvtColor(frame_local, cv2.COLOR_BGR2GRAY)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = aruco.DetectorParameters()

    corners, ids, rejectedImgPoints = aruco.detectMarkers(
        gray, aruco_dict, parameters=parameters
    )
    frame_markers = aruco.drawDetectedMarkers(frame_local.copy(), corners, ids)

    if ids is not None:
        ids_len = len(ids)
        res = []
        i = 0
        if ids_len > 0:
            for _ in range(ids_len):
                aruco_res = _detect(corners[i:i + 1], ids[i][0], frame_local)
                if aruco_res is not None:
                    res.append(aruco_res)
                i += 1
                if display_mode:
                    displayFrame(frame_markers)
        return (res, ids)
    else:
        if display_mode:
            displayFrame(frame_markers)
        return None


def process_qr_data():
    data_ = getArucoCode(True)

    if data_ is not None:
        if data_[0] == []:
            return -1

        _z = data_[0][0][2]
        _ry = data_[0][0][4]
        _perc = data_[0][0][6][0] / 640.0
        return (_z, _ry, _perc)
    else:
        return -1
