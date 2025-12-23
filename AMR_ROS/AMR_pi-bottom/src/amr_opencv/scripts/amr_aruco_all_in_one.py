#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import time
import threading

import numpy as np
import cv2
import cv2.aruco as aruco

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool, SetBoolResponse


# ----------------------------
# Defaults (override by ROS params)
# ----------------------------
CAM_ID = 0
CMD_VEL_TOPIC = "/cmd_vel"

DISPLAY = False
WINDOW_NAME = "aruco_view"

MARKER_LENGTH_M = 0.064

FAR_CM = 60.0
MID_CM_LOW = 30.0
MID_CM_HIGH = 60.0
NEAR_CM = 30.0

CENTER_LOW = 0.48
CENTER_HIGH = 0.56

MAX_RUNTIME_SEC = 120.0

SEARCH_STEP_SEC = 0.50
SEARCH_SPEED = 0.50
SEARCH_MAX_SEC = 20.0

TURN_INVERT = False

# Arrival distance per marker id (cm)
ARRIVAL_CM_BY_ID = {
    1: 40.0,
    2: 35.0,
    5: 30.0,
    6: 30.0,
}

EXTRA_FORWARD_CM_BY_ID = {
    5: 20.0,
    6: 20.0,
}

EXTRA_FORWARD_SPEED_MPS = 0.05
EXTRA_FORWARD_MAX_SEC = 15.0

# Camera thread params
CAM_THREAD_HZ = 30.0
DETECTION_MAX_AGE_SEC = 0.2

camera_matrix_override = None
dist_coeffs_override = None

cam = None
camera_matrix = None
dist_coeffs = None
frame_w = None
frame_h = None

camera_worker = None

pub = None
rate = None

RUN_LOCK = threading.Lock()
RUNNING = False

R_FLIP = np.array([[1.0, 0.0, 0.0],
                   [0.0, -1.0, 0.0],
                   [0.0, 0.0, -1.0]], dtype=np.float64)


def get_arrival_cm(marker_id):
    try:
        mid = int(marker_id)
    except Exception:
        return float(NEAR_CM)
    if mid in ARRIVAL_CM_BY_ID:
        return float(ARRIVAL_CM_BY_ID[mid])
    return float(NEAR_CM)


def get_extra_forward_cm(marker_id):
    try:
        mid = int(marker_id)
    except Exception:
        return 0.0
    return float(EXTRA_FORWARD_CM_BY_ID.get(mid, 0.0))


def drive_forward_cm(distance_cm, speed_mps):
    if distance_cm <= 0.0:
        return 0
    if speed_mps <= 0.0:
        return 0

    duration = (distance_cm / 100.0) / float(speed_mps)
    if duration <= 0.0:
        return 0

    if duration > EXTRA_FORWARD_MAX_SEC:
        duration = EXTRA_FORWARD_MAX_SEC

    return front_once(duration, speed_mps)


# ----------------------------
# ArUco compatibility
# ----------------------------
def build_aruco_detector():
    try:
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    except AttributeError:
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

    try:
        params = aruco.DetectorParameters()
        params.minMarkerPerimeterRate = 0.03
        detector = aruco.ArucoDetector(aruco_dict, params)

        def detect(gray_img):
            return detector.detectMarkers(gray_img)

        return aruco_dict, detect
    except AttributeError:
        params = aruco.DetectorParameters_create()

        def detect(gray_img):
            return aruco.detectMarkers(gray_img, aruco_dict, parameters=params)

        return aruco_dict, detect


ARUCO_DICT, DETECT_MARKERS = build_aruco_detector()


# ----------------------------
# cmd_vel helpers
# ----------------------------
def pub_vel(x, y, theta):
    if pub is None:
        return
    twist = Twist()
    twist.linear.x = float(x)
    twist.linear.y = float(y)
    twist.angular.z = float(theta)
    pub.publish(twist)


def stop():
    pub_vel(0.0, 0.0, 0.0)


# ----------------------------
# Math helpers
# ----------------------------
def is_rotation_matrix(R):
    Rt = R.T
    should_be_identity = Rt @ R
    I = np.identity(3, dtype=R.dtype)
    return np.linalg.norm(I - should_be_identity) < 1e-6


def rotation_matrix_to_euler_angles(R):
    if not is_rotation_matrix(R):
        return None

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

    return np.array([x_, y_, z_], dtype=np.float64)


# ----------------------------
# ArUco detection (single marker)
# ----------------------------
def detect_single_marker(corners_1, marker_id, draw_img=None):
    if corners_1 is None or len(corners_1) == 0:
        return None

    pts = corners_1[0][0].astype(np.float32)

    cx = int(pts[:, 0].mean())
    cy = int(pts[:, 1].mean())

    if draw_img is not None:
        p = pts.astype(int)
        cv2.polylines(draw_img, [p.reshape(-1, 1, 2)], True, (255, 0, 0), 2)
        cv2.putText(draw_img, "id=%d" % int(marker_id), (cx, cy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)

    half = MARKER_LENGTH_M / 2.0
    obj_pts = np.array([[-half,  half, 0.0],
                        [ half,  half, 0.0],
                        [ half, -half, 0.0],
                        [-half, -half, 0.0]], dtype=np.float32)

    ok, rvec, tvec = cv2.solvePnP(obj_pts, pts, camera_matrix, dist_coeffs)
    if not ok:
        return None

    R_ct, _ = cv2.Rodrigues(rvec)
    R_tc = R_ct.T
    euler = rotation_matrix_to_euler_angles(R_FLIP @ R_tc)
    if euler is None:
        return None

    roll_deg, pitch_deg, yaw_deg = map(math.degrees, euler.tolist())

    t = tvec.reshape(3)
    x_cm = float(t[0] * 100.0)
    y_cm = float(t[1] * 100.0)
    z_cm = float(t[2] * 100.0)

    if (abs(yaw_deg) % 90.0) >= 30.0:
        return None

    return [x_cm, y_cm, z_cm, roll_deg, pitch_deg, yaw_deg, (cx, cy), int(marker_id)]


# ----------------------------
# Camera worker thread
# ----------------------------
class CameraWorker(threading.Thread):
    def __init__(self, cap, display, window_name, hz):
        super(CameraWorker, self).__init__()
        self.daemon = True
        self.cap = cap
        self.display = bool(display)
        self.window_name = str(window_name)
        self.period = 1.0 / float(hz) if hz > 1.0 else 0.03

        self._stop_evt = threading.Event()
        self._lock = threading.Lock()

        self._latest = None
        self._latest_ts = 0.0
        self._latest_vis = None

        self._last_valid = None
        self._last_valid_ts = 0.0
        self._latest_frame_vis = None

    def stop(self):
        self._stop_evt.set()

    def get_latest(self, max_age_sec=0.45):
        now = time.time()
        with self._lock:
            if self._last_valid is None:
                return None
            if (now - self._last_valid_ts) > float(max_age_sec):
                return None
            return dict(self._last_valid)

    def _update(self, det, det_ts, vis):
        with self._lock:
            self._latest = det
            self._latest_ts = det_ts
            self._latest_vis = vis
            if det is not None:
                self._last_valid = det
                self._last_valid_ts = det_ts

    def run(self):
            if self.display:
                cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
    
            while not rospy.is_shutdown() and not self._stop_evt.is_set():
                t0 = time.time()
                ret, frame_bgr = self.cap.read()
                if not ret:
                    time.sleep(0.01)
                    continue
    
                if frame_w is None:
                    time.sleep(0.01)
                    continue
    
                det, frame_vis = self._detect(frame_bgr)
                
                with self._lock:
                    if det is not None:
                        self._last_valid = det
                        self._last_valid_ts = time.time()
                    self._latest_frame_vis = frame_vis
    
                if self.display and frame_vis is not None:
                    cv2.imshow(self.window_name, frame_vis)
                    cv2.waitKey(1)
    
                dt = time.time() - t0
                sleep_t = self.period - dt
                if sleep_t > 0:
                    time.sleep(sleep_t)
            if self.display:
                cv2.destroyWindow(self.window_name)

    def _detect(self, frame_bgr):
        frame_vis = frame_bgr.copy()
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _rej = DETECT_MARKERS(gray)

        if ids is not None and len(ids) > 0:
            try:
                frame_vis = aruco.drawDetectedMarkers(frame_vis, corners, ids)
            except Exception:
                pass

        if ids is None or len(ids) == 0:
            cv2.putText(frame_vis, "No marker", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
            return None, frame_vis

        valid = []
        for i in range(len(ids)):
            marker_id = int(ids[i][0])
            info = detect_single_marker(corners[i:i + 1], marker_id, draw_img=frame_vis)
            if info is not None:
                valid.append(info)

        if len(valid) == 0:
            return None, frame_vis

        valid.sort(key=lambda it: float(it[2]))
        z_cm = float(valid[0][2])
        pitch_deg = float(valid[0][4])
        cx = int(valid[0][6][0])
        mid = int(valid[0][7])

        if frame_w is None or frame_w <= 1:
            return None, frame_vis

        perc = float(cx) / float(frame_w)
        target_cm = get_arrival_cm(mid)

        cv2.putText(frame_vis, "id=%d dist=%.1fcm target=%.1fcm" % (mid, z_cm, target_cm),
                    (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame_vis, "perc=%.3f" % perc,
                    (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)

        det = {
            "z_cm": z_cm,
            "pitch_deg": pitch_deg,
            "perc": perc,
            "marker_id": mid,
        }
        return det, frame_vis


# ----------------------------
# Camera init/shutdown
# ----------------------------
def init_camera():
    global cam, camera_matrix, dist_coeffs, frame_w, frame_h, camera_worker

    if cam is not None and camera_worker is not None:
        return True

    cam = cv2.VideoCapture(CAM_ID)
    if not cam.isOpened():
        rospy.logerr("[aruco_ctrl] Cannot open camera id %d", CAM_ID)
        cam = None
        return False

    ret, frame = cam.read()
    if not ret:
        rospy.logerr("[aruco_ctrl] Cannot read initial frame")
        cam.release()
        cam = None
        return False

    frame_h, frame_w = frame.shape[:2]

    if camera_matrix_override is not None:
        camera_matrix = camera_matrix_override
    else:
        focal_length = float(frame_w)
        cx = frame_w / 2.0
        cy = frame_h / 2.0
        camera_matrix = np.array([[focal_length, 0.0, cx],
                                  [0.0, focal_length, cy],
                                  [0.0, 0.0, 1.0]], dtype=np.float64)

    if dist_coeffs_override is not None:
        dist_coeffs = dist_coeffs_override
    else:
        dist_coeffs = np.zeros((1, 5), dtype=np.float64)

    camera_worker = CameraWorker(cam, DISPLAY, WINDOW_NAME, CAM_THREAD_HZ)
    camera_worker.start()

    rospy.loginfo("[aruco_ctrl] Camera initialized: %dx%d", frame_w, frame_h)
    return True

def shutdown_camera():
    global cam, camera_worker
    rospy.loginfo("[aruco_ctrl] Shutting down camera...")
    if camera_worker is not None:
        camera_worker.stop()
        camera_worker.join(timeout=0.2)
        camera_worker = None
    if cam is not None:
        cam.release()
        cam = None
    rospy.loginfo("[aruco_ctrl] Camera shutdown complete.")


# ----------------------------
# Detection accessor (now uses thread cache)
# ----------------------------
def process_aruco_data():
    if camera_worker is None:
        return -1

    det = camera_worker.get_latest(max_age_sec=DETECTION_MAX_AGE_SEC)
    if det is None:
        return -1

    z_cm = det["z_cm"]
    pitch_deg = det["pitch_deg"]
    perc = det["perc"]
    marker_id = det["marker_id"]
    return (z_cm, pitch_deg, perc, marker_id)


# ----------------------------
# Motion primitives
# ----------------------------
def front_once(time_gap=0.5, sp=0.32, hz=20.0):
    if time_gap <= 0.0:
        stop()
        return 0

    period = 1.0 / float(hz) if hz and hz > 0.0 else 0.05
    t_end = time.time() + float(time_gap)

    while not rospy.is_shutdown() and time.time() < t_end:
        pub_vel(sp, 0.0, 0.0)   # keep publishing (watchdog-safe)
        time.sleep(period)

    stop()
    return 0 if not rospy.is_shutdown() else -1



def rotate_with_checks(dir_val,
                       total_sec,
                       sp,
                       stop_on_seen=False,
                       stop_on_center=False,
                       center_low=0.48,
                       center_high=0.56,
                       dt=0.03):
    t0 = time.time()
    pub_vel(0.0, 0.0, sp * dir_val)

    while not rospy.is_shutdown():
        if (time.time() - t0) >= total_sec:
            break

        res = process_aruco_data()
        if res != -1:
            perc = res[2]
            if stop_on_center and (center_low <= perc <= center_high):
                stop()
                return 1
            if stop_on_seen and not stop_on_center:
                stop()
                return 1

        time.sleep(dt)

    stop()
    return 0


# ----------------------------
# Search behavior
# ----------------------------
def search_scan(max_sec=10.0,
                first_dir=1,
                step_sec=0.4,
                sp=0.5,
                steps_each_side=5,
                settle_sec=0.50,
                check_frames=10,
                check_interval=0.05):
    rospy.loginfo("[aruco_ctrl] search_scan start")
    t0 = time.time()

    def timed_out():
        return (time.time() - t0) > max_sec

    def check_marker_multi():
        for _ in range(check_frames):
            if rospy.is_shutdown():
                stop()
                return -1
            if timed_out():
                stop()
                rospy.logwarn("[aruco_ctrl] search_scan timeout")
                return 0

            res = process_aruco_data()
            if res != -1:
                stop()
                return 1

            time.sleep(check_interval)
        return 2

    def step_rotate_and_check(dir_val):
        if rospy.is_shutdown():
            stop()
            return -1
        if timed_out():
            stop()
            rospy.logwarn("[aruco_ctrl] search_scan timeout")
            return 0

        # rotate but stop immediately if marker is seen during the motion
        rotate_with_checks(dir_val, step_sec, sp, stop_on_seen=True, dt=0.03)
        time.sleep(settle_sec)
        return check_marker_multi()

    def return_to_center(dir_val, steps_to_return):
        back_dir = -dir_val
        for _ in range(steps_to_return):
            if rospy.is_shutdown():
                stop()
                return -1
            if timed_out():
                stop()
                rospy.logwarn("[aruco_ctrl] search_scan timeout")
                return 0

            rotate_with_checks(back_dir, step_sec, sp, stop_on_seen=False, dt=0.03)
            time.sleep(settle_sec)

        stop()
        return 2

    base_dir = first_dir

    time.sleep(settle_sec)
    r0 = check_marker_multi()
    if r0 in (1, 0, -1):
        return r0

    while not rospy.is_shutdown():
        moved = 0
        for _ in range(steps_each_side):
            r = step_rotate_and_check(base_dir)
            if r in (1, 0, -1):
                return r
            moved += 1

        r = return_to_center(base_dir, moved)
        if r in (0, -1):
            return r

        moved = 0
        for _ in range(steps_each_side):
            r = step_rotate_and_check(-base_dir)
            if r in (1, 0, -1):
                return r
            moved += 1

        r = return_to_center(-base_dir, moved)
        if r in (0, -1):
            return r

        time.sleep(settle_sec)
        rc = check_marker_multi()
        if rc in (1, 0, -1):
            return rc

    stop()
    return -1


# ----------------------------
# Align stage (feedback-based stop)
# ----------------------------
def stage_slow_align(steps=10,
                     center_low=0.48, center_high=0.56,
                     sp=1.2, time_gap=1.2,
                     settle_sec=0.20,
                     auto_flip=True):
    global TURN_INVERT

    for _ in range(steps):
        if rospy.is_shutdown():
            stop()
            return -1

        res = process_aruco_data()
        if res == -1:
            return -1

        perc = res[2]
        if center_low <= perc <= center_high:
            stop()
            return 1

        dir_val = 1 if perc < center_low else -1
        if TURN_INVERT:
            dir_val *= -1

        err_before = abs(perc - 0.5)

        # rotate and stop early when center is reached
        rotate_with_checks(dir_val,
                           time_gap,
                           sp,
                           stop_on_center=True,
                           center_low=center_low,
                           center_high=center_high,
                           dt=0.03)

        time.sleep(settle_sec)

        if auto_flip:
            res2 = process_aruco_data()
            if res2 != -1:
                err_after = abs(res2[2] - 0.5)
                if err_after > err_before + 0.02:
                    TURN_INVERT = not TURN_INVERT

    return 0


# ----------------------------
# Main control logic
# ----------------------------
def main_process(first_dir=1):
    t_start = time.time()
    recenter_fail = 0

    if process_aruco_data() == -1:
        found = search_scan(max_sec=SEARCH_MAX_SEC,
                            first_dir=first_dir,
                            step_sec=SEARCH_STEP_SEC,
                            sp=SEARCH_SPEED,
                            steps_each_side=3)
        if found != 1:
            stop()
            return 0

    a0 = stage_slow_align(steps=12, center_low=CENTER_LOW, center_high=CENTER_HIGH, sp=1.2, time_gap=SEARCH_STEP_SEC)
    if a0 != 1:
        stage_slow_align(steps=16, center_low=0.46, center_high=0.58, sp=1.0, time_gap=SEARCH_STEP_SEC)

    while not rospy.is_shutdown():
        if (time.time() - t_start) > MAX_RUNTIME_SEC:
            rospy.logwarn("[aruco_ctrl] timeout")
            stop()
            return 0

        res = process_aruco_data()
        if res == -1:
            found = search_scan(max_sec=SEARCH_MAX_SEC,
                                first_dir=1,
                                step_sec=SEARCH_STEP_SEC,
                                sp=SEARCH_SPEED)
            if found != 1:
                stop()
                return 0
            recenter_fail = 0
            continue

        z_cm = res[0]
        marker_id = res[3]
        target_cm = get_arrival_cm(marker_id)

        if z_cm < target_cm:
            extra_cm = get_extra_forward_cm(marker_id)
            if extra_cm > 0.0:
                rospy.loginfo("[aruco_ctrl] extra forward id=%d: %.1f cm", int(marker_id), float(extra_cm))
                drive_forward_cm(extra_cm, EXTRA_FORWARD_SPEED_MPS)

            #front_once(0.1, 0.01)
            stop()
            return 1

        if z_cm > FAR_CM:
            forward_t = 1.0
        elif MID_CM_LOW < z_cm < MID_CM_HIGH:
            forward_t = 0.15
        else:
            forward_t = 0.3

        st = stage_slow_align(steps=8, center_low=CENTER_LOW, center_high=CENTER_HIGH, sp=1.2, time_gap=SEARCH_STEP_SEC)

        if st == 1:
            recenter_fail = 0
            if forward_t > 0.0:
                front_once(forward_t, 0.01)
            continue

        if st == -1:
            found = search_scan(max_sec=SEARCH_MAX_SEC,
                                first_dir=1,
                                step_sec=SEARCH_STEP_SEC,
                                sp=SEARCH_SPEED)
            if found != 1:
                stop()
                return 0
            recenter_fail = 0
            continue

        recenter_fail += 1
        if recenter_fail >= 3:
            stage_slow_align(steps=14, center_low=0.46, center_high=0.58, sp=1.0, time_gap=SEARCH_STEP_SEC)
            recenter_fail = 0

    stop()
    return 0


# ----------------------------
# Service callback
# ----------------------------
def handle_go_aruco(req):
    global RUNNING
    rospy.loginfo("[aruco_ctrl] Service called: %s", str(req.data))

    if not req.data:
        stop()
        return SetBoolResponse(success=True, message="Stopped by request.")

    with RUN_LOCK:
        if RUNNING:
            return SetBoolResponse(success=False, message="Already in progress.")
        RUNNING = True

    try:
        if not init_camera():
            return SetBoolResponse(success=False, message="Camera open failed.")

        result = main_process(first_dir=-1)

        stop()
        shutdown_camera()

        if result == 1:
            return SetBoolResponse(success=True, message="Success: Target reached + Extra drive.")
        else:
            return SetBoolResponse(success=False, message="Failed: Timeout or marker lost.")

    except Exception as e:
        stop()
        shutdown_camera()
        rospy.logerr(f"Error: {e}")
        return SetBoolResponse(success=False, message=f"Error: {e}")
    finally:
        with RUN_LOCK:
            RUNNING = False


# ----------------------------
# Param parsing
# ----------------------------
def read_matrix_param(name, expect_len):
    val = rospy.get_param(name, None)
    if val is None:
        return None
    if not isinstance(val, (list, tuple)) or len(val) != expect_len:
        rospy.logwarn("[aruco_ctrl] Param %s must be a list length %d", name, expect_len)
        return None
    return np.array(val, dtype=np.float64)


def main():
    global pub, rate
    global CAM_ID, CMD_VEL_TOPIC
    global DISPLAY, WINDOW_NAME
    global MARKER_LENGTH_M
    global FAR_CM, MID_CM_LOW, MID_CM_HIGH, NEAR_CM
    global CENTER_LOW, CENTER_HIGH
    global MAX_RUNTIME_SEC
    global SEARCH_STEP_SEC, SEARCH_SPEED, SEARCH_MAX_SEC
    global TURN_INVERT
    global camera_matrix_override, dist_coeffs_override
    global ARRIVAL_CM_BY_ID
    global CAM_THREAD_HZ, DETECTION_MAX_AGE_SEC

    rospy.init_node("aruco_ctrl_node", anonymous=False)

    CAM_ID = int(rospy.get_param("~camera_id", CAM_ID))
    CMD_VEL_TOPIC = str(rospy.get_param("~cmd_vel_topic", CMD_VEL_TOPIC))

    DISPLAY = bool(rospy.get_param("~display", rospy.get_param("display", DISPLAY)))
    WINDOW_NAME = str(rospy.get_param("~window_name", WINDOW_NAME))

    MARKER_LENGTH_M = float(rospy.get_param("~marker_length_m", MARKER_LENGTH_M))

    FAR_CM = float(rospy.get_param("~far_cm", FAR_CM))
    MID_CM_LOW = float(rospy.get_param("~mid_cm_low_cm", MID_CM_LOW))
    MID_CM_HIGH = float(rospy.get_param("~mid_cm_high_cm", MID_CM_HIGH))
    NEAR_CM = float(rospy.get_param("~near_cm", NEAR_CM))

    CENTER_LOW = float(rospy.get_param("~center_low", CENTER_LOW))
    CENTER_HIGH = float(rospy.get_param("~center_high", CENTER_HIGH))

    MAX_RUNTIME_SEC = float(rospy.get_param("~max_runtime_sec", MAX_RUNTIME_SEC))

    SEARCH_STEP_SEC = float(rospy.get_param("~search_step_sec", SEARCH_STEP_SEC))
    SEARCH_SPEED = float(rospy.get_param("~search_speed", SEARCH_SPEED))
    SEARCH_MAX_SEC = float(rospy.get_param("~search_max_sec", SEARCH_MAX_SEC))

    TURN_INVERT = bool(rospy.get_param("~invert_turn", TURN_INVERT))

    CAM_THREAD_HZ = float(rospy.get_param("~cam_thread_hz", CAM_THREAD_HZ))
    DETECTION_MAX_AGE_SEC = float(rospy.get_param("~detection_max_age_sec", DETECTION_MAX_AGE_SEC))

    arrival_map = rospy.get_param("~arrival_cm_by_id", None)
    if isinstance(arrival_map, dict):
        new_map = {}
        for k, v in arrival_map.items():
            try:
                new_map[int(k)] = float(v)
            except Exception:
                pass
        if len(new_map) > 0:
            ARRIVAL_CM_BY_ID = new_map

    cm = read_matrix_param("~camera_matrix", 9)
    if cm is not None:
        camera_matrix_override = cm.reshape(3, 3)

    dc = rospy.get_param("~dist_coeffs", None)
    if isinstance(dc, (list, tuple)) and len(dc) in (4, 5, 8):
        dist_coeffs_override = np.array(dc, dtype=np.float64).reshape(1, -1)

    pub = rospy.Publisher(CMD_VEL_TOPIC, Twist, queue_size=10)
    rate = rospy.Rate(30)

    rospy.on_shutdown(stop)

    rospy.Service("/go_aruco", SetBool, handle_go_aruco)
    rospy.loginfo("[aruco_ctrl] Service /go_aruco ready")

    rospy.spin()


if __name__ == "__main__":
    main()
