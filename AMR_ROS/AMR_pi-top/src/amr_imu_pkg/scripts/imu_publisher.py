#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler
import serial

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

    rpy_pub = rospy.Publisher("/external_imu_rpy", Vector3Stamped, queue_size=10)

    imu_pub = rospy.Publisher("/imu_data", Imu, queue_size=10)

    ser = serial.Serial('/dev/ttyAMA1', 115200, timeout=1)
    rospy.loginfo("[IMU UART] OPEN: %s", ser.is_open)
    rate = rospy.Rate(100)  

    DEG2RAD = 0.01745329252

    while not rospy.is_shutdown():

        ser.reset_input_buffer()

        line = ser.readline()
        if not line:
            continue

        parsed = parse_fast(line)
        if not parsed:
            continue

        heading_deg, roll_deg, pitch_deg, raw_text = parsed

        roll  = roll_deg  * DEG2RAD
        pitch = pitch_deg * DEG2RAD

        yaw_deg_ros = 360.0 - heading_deg
        if yaw_deg_ros > 180.0:
            yaw_deg_ros -= 360.0
        elif yaw_deg_ros < -180.0:
            yaw_deg_ros += 360.0

        yaw = yaw_deg_ros * DEG2RAD

        now = rospy.Time.now()

        rpy_msg = Vector3Stamped()
        rpy_msg.header.stamp = now
        rpy_msg.header.frame_id = "external_imu_link"
        rpy_msg.vector.x = roll
        rpy_msg.vector.y = pitch
        rpy_msg.vector.z = yaw
        rpy_pub.publish(rpy_msg)


        imu_msg = Imu()
        imu_msg.header.stamp = now

        imu_msg.header.frame_id = "external_imu_link"

        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw

        imu_msg.orientation_covariance[0] = 0.05  # roll
        imu_msg.orientation_covariance[4] = 0.05  # pitch
        imu_msg.orientation_covariance[8] = 0.05  # yaw

        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0

        imu_pub.publish(imu_msg)

        rate.sleep()


if __name__ == "__main__":
    main()
