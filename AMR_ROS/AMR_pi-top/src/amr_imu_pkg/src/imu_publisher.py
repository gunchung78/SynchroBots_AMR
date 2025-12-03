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

        # 가장 빠른 파싱 방법
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
    print("[IMU UART] OPEN:", ser.is_open)

    rate = rospy.Rate(100)   # 더 빠르게 읽어도 됨 (200Hz)

    while not rospy.is_shutdown():

        # ?? 주요 문제 해결: 오래된 데이터 제거
        ser.reset_input_buffer()

        line = ser.readline()
        if not line:
            continue

        parsed = parse_fast(line)
        if not parsed:
            continue

        heading_deg, roll_deg, pitch_deg, raw_text = parsed

        # ?? rad 변환 (최소 연산)
        roll  = roll_deg  * 0.01745329252   # math.radians보다 3배 빠름
        pitch = pitch_deg * 0.01745329252

        # IMU heading: 0~360도, 시계방향이 + 증가
        # ROS/AGV yaw: -180~+180도, 반시계(CCW)가 +
        # 변환: yaw_deg = 360 - heading_deg 후 [-180,180]로 정규화
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

        # ?? print가 loginfo보다 훨씬 빠름
        print(f"roll={roll:.4f}, pitch={pitch:.4f}, yaw_rad={yaw:.4f}, yaw_deg={yaw_deg_ros:.2f}, RAW={raw_text}")

        rate.sleep()


if __name__ == "__main__":
    main()


##!/usr/bin/env python3
## -*- coding: utf-8 -*-
#import rospy
#from geometry_msgs.msg import Vector3Stamped
#import serial
#import math
#
#
#def parse_imu_line(line):
#    """
#    Expected format:
#        H:232.25,R:0.75,P:1.62
#    """
#    try:
#        text = line.decode(errors="ignore").strip()
#
#        parts = text.split(",")
#        heading = float(parts[0].split(":")[1])
#        roll    = float(parts[1].split(":")[1])
#        pitch   = float(parts[2].split(":")[1])
#
#        return heading, roll, pitch, text  # text는 RAW 출력용
#
#    except Exception as e:
#        rospy.logwarn(f"[ParseError] {e} Raw={line}")
#        return None
#
#
#def main():
#    rospy.init_node("external_imu_rpy_publisher", anonymous=False)
#
#    pub = rospy.Publisher("/external_imu_rpy", Vector3Stamped, queue_size=10)
#
#    ser = serial.Serial(
#        port='/dev/serial0',
#        baudrate=115200,
#        timeout=1
#    )
#    rospy.loginfo(f"[IMU UART] OPEN: {ser.is_open}")
#
#    rate = rospy.Rate(100)  # 100Hz
#
#    while not rospy.is_shutdown():
#        line = ser.readline()
#
#        if not line:
#            rate.sleep()
#            continue
#
#        result = parse_imu_line(line)
#        if not result:
#            rate.sleep()
#            continue
#
#        heading_deg, roll_deg, pitch_deg, raw_text = result
#
#        # DEG → RAD
#        roll  = math.radians(roll_deg)
#        pitch = math.radians(pitch_deg)
#        yaw   = math.radians(heading_deg)
#
#        # -----------------------------
#        # ROS Publish
#        # -----------------------------
#        msg = Vector3Stamped()
#        msg.header.stamp = rospy.Time.now()
#        msg.header.frame_id = "external_imu_link"
#
#        msg.vector.x = roll
#        msg.vector.y = pitch
#        msg.vector.z = yaw
#
#        pub.publish(msg)
#
#        # -----------------------------
#        # Logging
#        # -----------------------------
#        rospy.loginfo(
#            f"roll={roll:.4f} rad, pitch={pitch:.4f} rad, yaw={yaw:.4f} rad, RAW: {raw_text}"
#        )
#
#        rate.sleep()
#
#
#if __name__ == "__main__":
#    try:
#        main()
#    except rospy.ROSInterruptException:
#        pass

##!/usr/bin/env python3
## -*- coding: utf-8 -*-
#import rospy
#from sensor_msgs.msg import Imu
#from geometry_msgs.msg import Quaternion, Vector3
#import serial
#import time
#
#
#def parse_imu_line(line):
#    """
#    Expected format:
#    QW:0.99,QX:0.01,QY:0.02,QZ:0.03,
#    GX:0.01,GY:-0.02,GZ:0.00,
#    AX:0.12,AY:0.03,AZ:9.81
#    """
#
#    try:
#        line = line.decode(errors="ignore").strip()
#        parts = line.split(",")
#
#        data = {}
#        for p in parts:
#            if ":" in p:
#                key, val = p.split(":")
#                data[key] = float(val)
#
#        # 반드시 필요한 값들
#        qw = data["QW"]
#        qx = data["QX"]
#        qy = data["QY"]
#        qz = data["QZ"]
#
#        gx = data["GX"]
#        gy = data["GY"]
#        gz = data["GZ"]
#
#        ax = data["AX"]
#        ay = data["AY"]
#        az = data["AZ"]
#
#        return qw, qx, qy, qz, gx, gy, gz, ax, ay, az
#
#    except Exception as e:
#        rospy.logwarn(f"Parse error: {e}, Raw: {line}")
#        return None
#
#
#def main():
#    rospy.init_node("imu_publisher", anonymous=False)
#    pub = rospy.Publisher("/external_imu", Imu, queue_size=10)
#
#    ser = serial.Serial(
#        port='/dev/serial0',
#        baudrate=115200,
#        timeout=1
#    )
#    rospy.loginfo(f"UART opened: {ser.is_open}")
#
#    rate = rospy.Rate(100)  
#
#    while not rospy.is_shutdown():
#        line = ser.readline()
#
#        if line:
#            parsed = parse_imu_line(line)
#
#            if parsed:
#                qw, qx, qy, qz, gx, gy, gz, ax, ay, az = parsed
#
#                msg = Imu()
#
#                # ------------------------
#                # Header
#                # ------------------------
#                msg.header.stamp = rospy.Time.now()
#                msg.header.frame_id = "imu_link"
#
#                # ------------------------
#                # ORIENTATION (Quaternion)
#                # ------------------------
#                msg.orientation = Quaternion(qx, qy, qz, qw)
#                msg.orientation_covariance[0] = -1  # unknown
#
#                # ------------------------
#                # GYRO (angular velocity)
#                # ------------------------
#                msg.angular_velocity = Vector3(gx, gy, gz)
#                msg.angular_velocity_covariance[0] = -1
#
#                # ------------------------
#                # ACC (linear acceleration)
#                # ------------------------
#                msg.linear_acceleration = Vector3(ax, ay, az)
#                msg.linear_acceleration_covariance[0] = -1
#
#                # Publish IMU message
#                pub.publish(msg)
#
#                rospy.loginfo(
#                    f"[IMU] Q=({qw:.3f},{qx:.3f},{qy:.3f},{qz:.3f}) "
#                    f"G=({gx:.3f},{gy:.3f},{gz:.3f}) "
#                    f"A=({ax:.3f},{ay:.3f},{az:.3f})"
#                )
#
#        rate.sleep()
#
#
#if __name__ == '__main__':
#    try:
#        main()
#    except rospy.ROSInterruptException:
#        pass

