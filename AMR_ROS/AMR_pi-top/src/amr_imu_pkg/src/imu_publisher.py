#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import serial
import time


def parse_imu_line(line):
    """
    Expected format:
    QW:0.99,QX:0.01,QY:0.02,QZ:0.03,
    GX:0.01,GY:-0.02,GZ:0.00,
    AX:0.12,AY:0.03,AZ:9.81
    """

    try:
        line = line.decode(errors="ignore").strip()
        parts = line.split(",")

        data = {}
        for p in parts:
            if ":" in p:
                key, val = p.split(":")
                data[key] = float(val)

        # 반드시 필요한 값들
        qw = data["QW"]
        qx = data["QX"]
        qy = data["QY"]
        qz = data["QZ"]

        gx = data["GX"]
        gy = data["GY"]
        gz = data["GZ"]

        ax = data["AX"]
        ay = data["AY"]
        az = data["AZ"]

        return qw, qx, qy, qz, gx, gy, gz, ax, ay, az

    except Exception as e:
        rospy.logwarn(f"Parse error: {e}, Raw: {line}")
        return None


def main():
    rospy.init_node("imu_publisher", anonymous=False)
    pub = rospy.Publisher("/imu/data", Imu, queue_size=10)

    ser = serial.Serial(
        port='/dev/serial0',
        baudrate=115200,
        timeout=1
    )
    rospy.loginfo(f"UART opened: {ser.is_open}")

    rate = rospy.Rate(100)  

    while not rospy.is_shutdown():
        line = ser.readline()

        if line:
            parsed = parse_imu_line(line)

            if parsed:
                qw, qx, qy, qz, gx, gy, gz, ax, ay, az = parsed

                msg = Imu()

                # ------------------------
                # Header
                # ------------------------
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "imu_link"

                # ------------------------
                # ORIENTATION (Quaternion)
                # ------------------------
                msg.orientation = Quaternion(qx, qy, qz, qw)
                msg.orientation_covariance[0] = -1  # unknown

                # ------------------------
                # GYRO (angular velocity)
                # ------------------------
                msg.angular_velocity = Vector3(gx, gy, gz)
                msg.angular_velocity_covariance[0] = -1

                # ------------------------
                # ACC (linear acceleration)
                # ------------------------
                msg.linear_acceleration = Vector3(ax, ay, az)
                msg.linear_acceleration_covariance[0] = -1

                # Publish IMU message
                pub.publish(msg)

                rospy.loginfo(
                    f"[IMU] Q=({qw:.3f},{qx:.3f},{qy:.3f},{qz:.3f}) "
                    f"G=({gx:.3f},{gy:.3f},{gz:.3f}) "
                    f"A=({ax:.3f},{ay:.3f},{az:.3f})"
                )

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
