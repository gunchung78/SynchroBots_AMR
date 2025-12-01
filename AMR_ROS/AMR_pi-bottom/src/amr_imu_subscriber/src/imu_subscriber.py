#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu

def imu_callback(msg):
    """
    msg.data = [heading, roll, pitch]
    """
    try:
        heading, roll, pitch = msg.data
        rospy.loginfo(f"[SUB] H={heading:.2f}, R={roll:.2f}, P={pitch:.2f}")
   
    except Exception as e:
        rospy.logwarn(f"[SUB] parse error: {e}, raw={msg.data}")

def main():
    rospy.init_node("imu_sub", anonymous=False)

    rospy.Subscriber("/imu/data", Imu, imu_callback, queue_size=50)

    rospy.loginfo("IMU Subscriber started, waiting for /imu_data ...")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
