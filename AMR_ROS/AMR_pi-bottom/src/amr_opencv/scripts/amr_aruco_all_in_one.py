#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import numpy as np
import cv2
import cv2.aruco as aruco

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse

# ========== Global camera and ArUco variables ==========

cam = None
camera_matrix = None
dist_coeffs = np.array(
    [[3.41360787e-01, -2.52114260e+00, -1.28012469e-03,
      6.70503562e-03, 2.57018000e+00]]
)

marker_length = 0.032  # meter

R_flip = np.zeros((3, 3), dtype=np.float32)
R_flip[0, 0] = 1.0
R_flip[1, 1] = -1.0
R_flip[2, 2] = -1.0

pose_data = [None, None, None, None, None, None]
pose_data_dict = {}

ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
ARUCO_PARAMS = aruco.DetectorParameters()
ARUCO_DETECTOR = aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

pub = None
rate = None


def init_camera():
    """Open camera and compute camera matrix only when needed (service call)."""
    global cam, camera_matrix
    if cam is not None:
        return True

    cam_id = 0
    cam = cv2.VideoCapture(cam_id)
    if not cam.isOpened():
        rospy.logerr("[amr_aruco_all_in_one] Cannot open camera id %d", cam_id)
        cam = None
        return False

    ret, frame = cam.read()
    if not ret:
        rospy.logerr("[amr_aruco_all_in_one] Cannot read initial frame from camera")
        cam.release()
        cam = None
        return False

    h, w = frame.shape[:2]
    focal_length = w
    center = (w / 2.0, h / 2.0)

    camera_matrix = np.array(
        [
            [focal_length, 0.0, center[0]],
            [0.0, focal_length, center[1]],
            [0.0, 0.0, 1.0],
        ],
        dtype="double",
    )

    cv2.namedWindow("show", 0)
    rospy.loginfo("[amr_aruco_all_in_one] Camera initialized: %dx%d", w, h)
    return True


# [ADDED] graceful camera shutdown so we can reopen on next service call
def shutdown_camera():
    global cam
    if cam is not None:
        rospy.loginfo("[amr_aruco_all_in_one] Releasing camera")
        cam.release()
        cam = None
    # Safe even if window does not exist
    cv2.destroyAllWindows()


# ========== Math helpers ==========

def is_rotation_matrix(R):
    Rt = np.transpose(R)
    should_be_identity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - should_be_identity)
    return n < 1e-6


def rotation_matrix_to_euler_angles(R):
    assert is_rotation_matrix(R)

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


# ========== ArUco detection ==========

def detect_single_marker(corners, ids_val, img_with_aruco):
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
        cv2.putText(img_with_aruco, "C1", x1, font_local, 1,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img_with_aruco, "C2", x2, font_local, 1,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img_with_aruco, "C3", x3, font_local, 1,
                    (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(img_with_aruco, "C4", x4, font_local, 1,
                    (255, 255, 255), 1, cv2.LINE_AA)

        if ids_val is not None:
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

            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts, camera_matrix, dist_coeffs
            )
            if not ok:
                pose_data[0] = None
                pose_data[1] = None
                pose_data[2] = None
                pose_data[3] = None
                pose_data_dict[ids_val] = pose_data
                return None

            corner_mid = (
                int((x1[0] + x2[0] + x3[0] + x4[0]) / 4),
                int((x1[1] + x2[1] + x3[1] + x4[1]) / 4),
            )

            cv2.putText(
                img_with_aruco,
                "id=" + str(ids_val),
                corner_mid,
                font_local,
                1,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

            R_ct, _ = cv2.Rodrigues(rvec)
            R_tc = np.matrix(R_ct).T

            roll_marker, pitch_marker, yaw_marker = \
                rotation_matrix_to_euler_angles(R_flip * R_tc)

            t = tvec.reshape(3)

            pose_data[0] = t[0] * 100.0
            pose_data[1] = t[1] * 100.0
            pose_data[2] = t[2] * 100.0
            pose_data[3] = math.degrees(roll_marker)
            pose_data[4] = math.degrees(pitch_marker)
            pose_data[5] = math.degrees(yaw_marker)

            pose_data_dict[ids_val] = pose_data

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


def display_frame(frame_input):
    cv2.imshow("show", frame_input)
    cv2.waitKey(1)


def get_aruco_code(display_mode=True):
    if cam is None:
        if not init_camera():
            return None

    ret, frame_local = cam.read()
    if not ret:
        rospy.logwarn("[amr_aruco_all_in_one] Failed to grab frame from camera")
        return None

    gray = cv2.cvtColor(frame_local, cv2.COLOR_BGR2GRAY)

    corners, ids_local, rejected = ARUCO_DETECTOR.detectMarkers(gray)

    frame_markers = aruco.drawDetectedMarkers(
        frame_local.copy(), corners, ids_local
    )

    if ids_local is not None and len(ids_local) > 0:
        res = []
        for i in range(len(ids_local)):
            marker_info = detect_single_marker(
                corners[i:i + 1], ids_local[i][0], frame_local
            )
            if marker_info is not None:
                res.append(marker_info)

        if display_mode:
            display_frame(frame_markers)

        return (res, ids_local)
    else:
        if display_mode:
            display_frame(frame_markers)
        return None


def process_qr_data():
    data_ = get_aruco_code(True)

    if data_ is not None:
        if data_[0] == []:
            return -1

        z_val = data_[0][0][2]
        ry_val = data_[0][0][4]
        perc = data_[0][0][6][0] / 640.0
        return (z_val, ry_val, perc)
    else:
        return -1


# ========== ROS velocity helpers ==========

def pub_vel(x, y, theta):
    twist = Twist()
    twist.linear.x = x
    twist.linear.y = y
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = theta
    pub.publish(twist)


def stop():
    pub_vel(0.0, 0.0, 0.0)


# ========== State machine movement ==========

def rot_once(dir_val=1, time_gap_input=0.5, sp=0.9, not_ignore_qr=True):
    pub_vel(0.0, 0.0, sp * dir_val)
    time_ini = time.time()
    while True:
        time_gap = time.time() - time_ini
        res = process_qr_data()
        if res != -1 and not_ignore_qr:
            stop()
            return 1
        if time_gap > time_gap_input:
            stop()
            return 0


def stage_quick_rot(fir_dir=1, first_rot_times=3, second_rot_times=6):
    time_wait = 1.0
    print("start stage_quick_rot")

    def rot_dir_times(dir_val, times_val):
        for _ in range(times_val):
            res = process_qr_data()
            if res != -1:
                return 1

            if rot_once(dir_val):
                return 1

            rot_once(1, time_wait, 0.0, 0)
        print("nothing in this round")
        return 0

    if rot_dir_times(fir_dir, first_rot_times) == 1:
        print("counter clock found")
        return 1
    if rot_dir_times(-fir_dir, second_rot_times) == 1:
        print("clock found")
        return 1
    print("nothing found in stage_quick_rot")
    return 0


# [CHANGED] make slow rotation more fine (smaller speed, smaller window)
def stage_slow_rot(slow_rot_times=6):
    # smaller speed and shorter step for fine alignment
    dir_val = 1
    sp = 0.3             # was 0.5
    time_gap = 0.20      # was 0.40
    center_low = 0.48    # was 0.40
    center_high = 0.52   # was 0.60

    rot_once(1, 1.0, 0.0, 1)

    for _ in range(slow_rot_times):
        res = process_qr_data()

        if res != -1:
            perc = res[2]

            if perc < center_low:
                dir_val = 1
            elif perc > center_high:
                dir_val = -1
            else:
                print("slow move success (fine)")
                stop()
                return 1

            rot_once(dir_val, time_gap, sp, not_ignore_qr=False)
        else:
            if rot_once(1, 1.0, 0.0) != 1:
                print("miss target in slow_rot")
                return -1

        rot_once(1, 1.0, 0.0, 0)

    print("slow focus fail")
    return 0


def front_once(time_gap=0.5, sp=0.32):
    pub_vel(sp, 0.0, 0.0)

    time_ini = time.time()
    while True:
        time_gap_cur = time.time() - time_ini
        _ = process_qr_data()
        if time_gap_cur > time_gap:
            stop()
            return 0


def stages_rot(dir_val=1, first_dir_times=3, second_dir_times=6):
    if stage_quick_rot(dir_val, first_dir_times, second_dir_times):
        if stage_slow_rot(9):
            return 1
    return 0


def main_process(first_dir=1):
    rot_once(1, 3.0, 0.0, 0)

    print("Step 1")
    if stages_rot(first_dir, 2, 5) == 0:
        print("initial found failed")
        return 0

    print("Step 2")
    while True:
        res = process_qr_data()
        if res != -1:
            l_val = res[0]
            ag_val = res[1]
            print("l:", l_val, "angle:", ag_val)

            if l_val > 60.0:
                if stage_slow_rot(6):
                    front_once(1.0, 0.01)
                    continue
                else:
                    stages_rot(1, 2, 4)

            elif 30.0 < l_val < 60.0:
                if stage_slow_rot(6):
                    front_once(0.15, 0.01)
                    continue
                else:
                    stages_rot(1, 2, 4)

            elif l_val < 35.0:
                front_once(0.1, sp=0.01)
                print("Finish doing")
                break
        else:
            print("cannot detect aruco")
            break

    rot_once(1, 1.0, 0.0, 0)
    return 1


# ========== Service callback ==========

def handle_go_aruco(req):
    rospy.loginfo("[amr_aruco_all_in_one] /go_aruco request: %s", str(req.data))

    if not req.data:
        return SetBoolResponse(
            success=True,
            message="Request data is False. No action."
        )

    if not init_camera():
        shutdown_camera()
        return SetBoolResponse(
            success=False,
            message="Camera initialization failed."
        )

    result = main_process(first_dir=-1)

    # [ADDED] close camera after run so we wait idle until next service call
    shutdown_camera()

    if result == 1:
        return SetBoolResponse(
            success=True,
            message="ArUco navigation finished successfully."
        )
    else:
        return SetBoolResponse(
            success=False,
            message="ArUco navigation failed or aborted."
        )


# ========== Node entry ==========

def main():
    global pub, rate

    rospy.init_node("amr_aruco_all_in_one", anonymous=False)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    rate = rospy.Rate(30)

    rospy.Service("/go_aruco", SetBool, handle_go_aruco)
    rospy.loginfo("[amr_aruco_all_in_one] Service /go_aruco is ready. Waiting for requests...")

    try:
        rospy.spin()
    finally:
        shutdown_camera()


if __name__ == "__main__":
    main()
