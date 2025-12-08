#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Vector3Stamped
import serial
import math

def parse_fast(line):
    """
    Fast parsing for data format:
    H:232.25,R:0.75,P:1.62
    """
    try:
        t = line.decode(errors="ignore").strip()

        h = float(t[t.find("H:")+2 : t.find(",R")])
        r = float(t[t.find("R:")+2 : t.find(",P")])
        p = float(t[t.find("P:")+2 : ])

        return h, r, p, t

    except:
        return None


def main():
    rospy.init_node("external_imu_rpy_publisher", anonymous=False)
    pub = rospy.Publisher("/external_imu_rpy", Vector3Stamped, queue_size=10)

    ser = serial.Serial('/dev/serial0', 115200, timeout=1)
    rospy.loginfo("[IMU UART] OPEN: %s", ser.is_open)
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():

        ser.reset_input_buffer()

        line = ser.readline()
        if not line:
            continue

        parsed = parse_fast(line)
        if not parsed:
            continue

        heading_deg, roll_deg, pitch_deg, raw_text = parsed

        roll  = roll_deg  * 0.01745329252
        pitch = pitch_deg * 0.01745329252

        yaw_deg_ros = 360.0 - heading_deg
        if yaw_deg_ros > 180.0:
            yaw_deg_ros -= 360.0
        elif yaw_deg_ros < -180.0:
            yaw_deg_ros += 360.0

        yaw = yaw_deg_ros * 0.01745329252

        # Publish
        msg = Vector3Stamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "external_imu_link"

        msg.vector.x = roll
        msg.vector.y = pitch
        msg.vector.z = yaw
        pub.publish(msg)

        print(f"roll={roll:.4f}, pitch={pitch:.4f}, yaw_rad={yaw:.4f}, yaw_deg={yaw_deg_ros:.2f}, RAW={raw_text}")

        rate.sleep()


if __name__ == "__main__":
    main()

