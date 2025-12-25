# coding=utf8
import numpy as np
import math
import time
import cv2

try:
    import cv2.aruco as aruco
except Exception:
    raise ImportError("cv2.aruco not found. Install opencv-contrib-python 4.2.x.")

cam_id = 0
cam = cv2.VideoCapture(cam_id)
if not cam.isOpened():
    raise RuntimeError("Cannot open camera id=%d" % cam_id)

dist_coeffs = np.array(
    [[3.41360787e-01, -2.52114260e+00, -1.28012469e-03, 6.70503562e-03, 2.57018000e+00]],
    dtype=np.float64
)

font = cv2.FONT_HERSHEY_SIMPLEX

ret, frame0 = cam.read()
if not ret or frame0 is None:
    raise RuntimeError("Failed to read first frame from camera.")

width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cam.get(cv2.CAP_PROP_FPS)

size = frame0.shape
focal_length = float(size[1])
center = (size[1] / 2.0, size[0] / 2.0)

camera_matrix = np.array(
    [[focal_length, 0, center[0]],
     [0, focal_length, center[1]],
     [0, 0, 1]],
    dtype=np.float64
)

cv2.namedWindow("show", 0)

marker_length = 0.032  # meters

R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

pose_data_dict = {}

def _get_aruco_dict():
    if hasattr(aruco, "Dictionary_get"):
        return aruco.Dictionary_get(aruco.DICT_6X6_250)
    return aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

def _get_detector_params():
    if hasattr(aruco, "DetectorParameters_create"):
        return aruco.DetectorParameters_create()
    return aruco.DetectorParameters()

def _is_rotation_matrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def _rotation_matrix_to_euler_angles(R):
    assert _is_rotation_matrix(R)
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

def _draw_axes(img, rvec, tvec, length):
    if hasattr(aruco, "drawAxis"):
        aruco.drawAxis(img, camera_matrix, dist_coeffs, rvec, tvec, length)
    else:
        cv2.drawFrameAxes(img, camera_matrix, dist_coeffs, rvec, tvec, length)

def _detect_one(corners, marker_id, img):
    if len(corners) == 0:
        return None

    x1 = (int(corners[0][0][0][0]), int(corners[0][0][0][1]))
    x2 = (int(corners[0][0][1][0]), int(corners[0][0][1][1]))
    x3 = (int(corners[0][0][2][0]), int(corners[0][0][2][1]))
    x4 = (int(corners[0][0][3][0]), int(corners[0][0][3][1]))

    cv2.line(img, x1, x2, (255, 0, 0), 1)
    cv2.line(img, x2, x3, (255, 0, 0), 1)
    cv2.line(img, x3, x4, (255, 0, 0), 1)
    cv2.line(img, x4, x1, (255, 0, 0), 1)

    cv2.putText(img, "C1", x1, font, 1, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, "C2", x2, font, 1, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, "C3", x3, font, 1, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(img, "C4", x4, font, 1, (255, 255, 255), 1, cv2.LINE_AA)

    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)
    rvec = rvecs[0][0]
    tvec = tvecs[0][0]

    _draw_axes(img, rvec, tvec, marker_length)

    cornerMid = (int((x1[0] + x2[0] + x3[0] + x4[0]) / 4),
                 int((x1[1] + x2[1] + x3[1] + x4[1]) / 4))
    cv2.putText(img, "id=" + str(marker_id), cornerMid, font, 1, (255, 255, 255), 1, cv2.LINE_AA)

    R_ct = np.matrix(cv2.Rodrigues(rvec)[0])
    R_tc = R_ct.T

    roll_marker, pitch_marker, yaw_marker = _rotation_matrix_to_euler_angles(R_flip * R_tc)
    roll_deg = float(math.degrees(roll_marker))
    pitch_deg = float(math.degrees(pitch_marker))
    yaw_deg = float(math.degrees(yaw_marker))

    pose_data = [
        float(tvec[0] * 100.0),
        float(tvec[1] * 100.0),
        float(tvec[2] * 100.0),
        roll_deg,
        pitch_deg,
        yaw_deg
    ]
    pose_data_dict[int(marker_id)] = pose_data[:]

    if abs(yaw_deg) % 90.0 < 30:
        return [pose_data[0], pose_data[1], pose_data[2], roll_deg, pitch_deg, yaw_deg, cornerMid]
    return None

def displayFrame(frame_input):
    cv2.imshow("show", frame_input)
    cv2.waitKey(1)

def getArucoCode(display_mode=True):
    ret, frame = cam.read()
    if not ret or frame is None:
        return None

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    aruco_dict = _get_aruco_dict()
    parameters = _get_detector_params()

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    frame_markers = frame.copy()
    if len(corners) > 0:
        if ids is not None:
            frame_markers = aruco.drawDetectedMarkers(frame_markers, corners, ids)
        else:
            frame_markers = aruco.drawDetectedMarkers(frame_markers, corners)

    if ids is not None and len(ids) > 0:
        res = []
        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            one = _detect_one(corners[i:i+1], marker_id, frame_markers)
            if one is not None:
                res.append(one)

        if display_mode:
            displayFrame(frame_markers)

        return (res, ids)

    if display_mode:
        displayFrame(frame_markers)
    return None

def process_qr_data():
    data_ = getArucoCode(True)
    if data_ is None or data_[0] == []:
        return -1

    z_cm = data_[0][0][2]
    pitch_deg = data_[0][0][4]
    perc_x = float(data_[0][0][6][0]) / float(max(width, 1))
    return (z_cm, pitch_deg, perc_x)
