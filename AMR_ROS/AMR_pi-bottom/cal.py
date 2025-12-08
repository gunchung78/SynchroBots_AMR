# -*- coding: utf-8 -*-

import cv2
import numpy as np
import os
import shutil

# ==========================================
# Checkerboard inner corner count (cols, rows)
# Example: (8, 7) means 8x7 internal corners
CHECKERBOARD = (8, 7)
CAMERA_INDEX = 0
SAVE_DIR = "calib_imgs"  # Folder to save calibration images
# ==========================================


def main():
    # 1. Prepare save folder (clean recreate)
    if os.path.exists(SAVE_DIR):
        shutil.rmtree(SAVE_DIR)
    os.makedirs(SAVE_DIR)
    print(f"[INFO] Images will be saved into '{SAVE_DIR}' folder.")

    # Prepare 3D object points
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane

    # 2. Open camera
    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"[WARN] Failed to open camera index {CAMERA_INDEX} -> trying 2")
        cap = cv2.VideoCapture(2)

    # ===================================
    # Try to keep original FOV (avoid digital zoom/crop)
    # ===================================
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    # Digital zoom / pan / tilt reset (ignored if not supported)
    try:
        cap.set(cv2.CAP_PROP_ZOOM, 1)
        cap.set(cv2.CAP_PROP_PAN, 0)
        cap.set(cv2.CAP_PROP_TILT, 0)
    except Exception:
        pass

    print("\n[INFO] Camera properties set:")
    print(" - Format: MJPG")
    print(" - Resolution: 1280 x 960")
    print(" - Digital zoom / crop disabled (if supported)")

    count = 0
    last_frame = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Failed to read frame from camera.")
            break

        last_frame = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret_corn, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

        # Draw visualization (we always save the original frame)
        vis_frame = frame.copy()
        if ret_corn:
            cv2.drawChessboardCorners(vis_frame, CHECKERBOARD, corners, ret_corn)
            cv2.putText(
                vis_frame,
                "READY (Press 'c')",
                (30, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2,
            )

        cv2.imshow("Calibration Shot", vis_frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('c'):
            if ret_corn:
                objpoints.append(objp)
                imgpoints.append(corners)
                count += 1
                filename = f"{SAVE_DIR}/img_{count:02d}.jpg"
                cv2.imwrite(filename, frame)
                print(f"[SAVE] Captured image {count} -> {filename}")
            else:
                print("[WARN] Checkerboard not detected. Adjust angle / distance.")
        elif key == ord('q'):
            print("[INFO] Quit key pressed.")
            break

    cap.release()
    cv2.destroyAllWindows()

    if count < 10:
        print("[WARN] Not enough images for stable calibration. (>= 10 recommended)")
        return

    # 3. Calibrate camera
    print("\n[INFO] Running camera calibration (this may take a short moment)...")

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    # Compute re-projection error
    mean_error = 0.0
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(
            objpoints[i], rvecs[i], tvecs[i], mtx, dist
        )
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error

    total_error = mean_error / len(objpoints)

    print("\n" + "=" * 50)
    print(f"[RESULT] Re-projection Error: {total_error:.4f}")
    print("  - 0.1 ~ 0.3 : Excellent")
    print("  - 0.3 ~ 0.7 : Good")
    print("  - >= 1.0   : Re-calibration recommended")
    print("=" * 50)

    print("\n[INFO] Calibration matrices (copy into your vision node):")
    print("-" * 30)
    print("CALIB_MATRIX_K = np.array(" + np.array2string(mtx, separator=', ') + ")")
    print("DIST_COEFF_D   = np.array(" + np.array2string(dist, separator=', ') + ")")
    print("-" * 30)

    # 4. Undistort test (visual check)
    print("\n[INFO] Showing undistortion test window (press any key to exit).")

    if last_frame is not None:
        h, w = last_frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
            mtx, dist, (w, h), 1, (w, h)
        )
        dst = cv2.undistort(last_frame, mtx, dist, None, newcameramtx)
        compare = np.hstack((last_frame, dst))
        cv2.imshow("Before (Left) vs After (Right)", compare)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
