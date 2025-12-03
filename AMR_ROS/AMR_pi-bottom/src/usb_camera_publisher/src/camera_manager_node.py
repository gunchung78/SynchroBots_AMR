#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import subprocess
import signal
from std_srvs.srv import Trigger, TriggerResponse

class CameraManager:
    def __init__(self):
        self.proc = None

        rospy.Service("/start_camera", Trigger, self.handle_start)
        rospy.Service("/stop_camera",  Trigger, self.handle_stop)

        rospy.loginfo("[camera_manager] ready. wait for /start_camera, /stop_camera")

    def handle_start(self, _req):

        if self.proc is not None and self.proc.poll() is None:
            return TriggerResponse(success=True, message="already running")

        cmd = (
            "source /opt/ros/noetic/setup.bash && "
            "source ~/myagv_ros/devel/setup.bash && "
            "rosrun usb_camera_publisher vision_pub_node.py"
        )

        rospy.loginfo("[camera_manager] starting vision_pub_node.py ...")
        self.proc = subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
        )
        return TriggerResponse(success=True, message="camera started")

    def handle_stop(self, _req):
        if self.proc is None or self.proc.poll() is not None:
            return TriggerResponse(success=True, message="not running")

        rospy.loginfo("[camera_manager] stopping vision_pub_node.py ...")

        self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=3.0)
        except Exception:
            self.proc.kill()

        self.proc = None
        return TriggerResponse(success=True, message="camera stopped")


def main():
    rospy.init_node("camera_manager_node", anonymous=False)
    CameraManager()
    rospy.spin()

if __name__ == "__main__":
    main()
