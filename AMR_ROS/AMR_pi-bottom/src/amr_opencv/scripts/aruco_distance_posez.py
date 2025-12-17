#!/usr/bin/env python3

import numpy as np
import time
import cv2
import cv2.aruco as aruco
from collections import deque

# load camera parameters
cv_file = cv2.FileStorage(
    "/home/er/SynchroBots_AMR/AMR_ROS/AMR_pi-bottom/src/amr_opencv/scripts/yuyan.yaml",
    cv2.FILE_STORAGE_READ,
)
camera_matrix = cv_file.getNode("camera_matrix").mat()
dist_matrix = cv_file.getNode("dist_coeff").mat()
cv_file.release()

# aruco dictionary and detector (new API style)
ARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
ARUCO_PARAMS = aruco.DetectorParameters()
ARUCO_DETECTOR = aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)

DEBUG = False
MARKER_LENGTH = 0.064  # marker side length in meters

cap = cv2.VideoCapture(0)
font = cv2.FONT_HERSHEY_SIMPLEX
distance_values = deque(maxlen=5)

while True:
    ret, frame = cap.read()
    if not ret:
        print("camera read failed")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # detect markers
    corners, ids, rejected = ARUCO_DETECTOR.detectMarkers(gray)

    if ids is not None and len(ids) > 0:
        # use first marker
        img_pts = corners[0].reshape(4, 2).astype(np.float32)

        half = MARKER_LENGTH / 2.0
        obj_pts = np.array(
            [
                [-half,  half, 0.0],
                [ half,  half, 0.0],
                [ half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float32,
        )

        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_matrix, dist_matrix)

        if ok:
            # draw marker and axes
            aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.drawFrameAxes(frame, camera_matrix, dist_matrix, rvec, tvec, 0.03)

            z_m = float(tvec[2])
            distance = z_m * 100.0 - 5.0  # cm, with small offset
            distance_values.append(distance)

            disp_distance = distance
            if DEBUG and len(distance_values) == distance_values.maxlen:
                disp_distance = float(np.mean(distance_values))

            cv2.putText(
                frame,
                "Id: {}".format(ids.flatten().tolist()),
                (0, 64),
                font,
                1,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                frame,
                "distance: {:.2f}cm".format(disp_distance),
                (0, 110),
                font,
                1,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
    else:
        cv2.putText(
            frame,
            "No Ids",
            (0, 64),
            font,
            1,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    cv2.imshow("frame", frame)

    key = cv2.waitKey(1) & 0xFF

    if key == 27:
        print("esc break")
        break

    if key == ord(" "):
        filename = str(int(time.time())) + ".jpg"
        cv2.imwrite(filename, frame)
        print("saved:", filename)

cap.release()
cv2.destroyAllWindows()
