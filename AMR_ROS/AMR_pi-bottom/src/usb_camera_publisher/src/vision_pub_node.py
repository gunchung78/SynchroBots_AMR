#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

CAM_INDEX = 0
WIDTH     = 640
HEIGHT    = 480
FPS       = 30
DEBUG     = True
PREVIEW   = False

_shutdown_flag = False

def _shutdown_cb(msg: Bool):
    global _shutdown_flag
    if msg.data:
        rospy.loginfo("[vision_pub_node] shutdown signal received.")
        _shutdown_flag = True

def main():
    rospy.init_node("vision_pub_node", anonymous=True)

    # subscribe to shutdown signal
    rospy.Subscriber("/shutdown_signal", Bool, _shutdown_cb, queue_size=1)

    pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)
    bridge = CvBridge()

    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        rospy.logerr(f"[cam={CAM_INDEX}] Failed to open camera")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    try:
        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        cap.set(cv2.CAP_PROP_FOURCC, fourcc)
    except Exception:
        pass
    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    rospy.loginfo(f"vision_pub_node started (cam={CAM_INDEX}, {WIDTH}x{HEIGHT}@{FPS}fps)")
    rate = rospy.Rate(FPS if FPS > 0 else 30)

    last_log = time.time()
    cnt = 0

    while not rospy.is_shutdown() and not _shutdown_flag:
        ok, frame = cap.read()
        if not ok:
            rospy.logwarn_throttle(5.0, f"[cam={CAM_INDEX}] Failed to read frame (throttled)")
            rate.sleep()
            continue

        pub.publish(bridge.cv2_to_imgmsg(frame, encoding="bgr8"))

        if PREVIEW:
            cv2.imshow("AGV Camera Preview (publisher)", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown("User quit preview")
                break

        if DEBUG:
            cnt += 1
            now = time.time()
            if now - last_log >= 1.0:
                rospy.loginfo(f"publishing... ~{cnt} fps")
                cnt = 0
                last_log = now

        rate.sleep()

    cap.release()
    if PREVIEW:
        cv2.destroyAllWindows()
    if _shutdown_flag:
        rospy.signal_shutdown("Stopped by /shutdown_signal")
    rospy.loginfo("vision_pub_node stopped.")

if __name__ == "__main__":
    main()
