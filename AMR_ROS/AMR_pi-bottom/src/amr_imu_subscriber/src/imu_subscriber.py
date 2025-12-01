#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class ImuOdomFusionNode(object):
    """
    Subscribe IMU and external_imu_odom, then
    replace odom pose.orientation with IMU orientation
    and publish fused odom.
    """

    def __init__(self):
        rospy.init_node("imu_sub", anonymous=False)

        imu_topic = rospy.get_param("~imu_topic", "/imu/data")
        odom_in_topic = rospy.get_param("~odom_in_topic", "/external_imu_odom")
        odom_out_topic = rospy.get_param("~odom_out_topic", "/odom")

        self.latest_imu_orientation = None

        self.sub_imu = rospy.Subscriber(
            imu_topic, Imu, self.imu_callback, queue_size=50
        )
        self.sub_odom = rospy.Subscriber(
            odom_in_topic, Odometry, self.odom_callback, queue_size=50
        )
        self.pub_odom = rospy.Publisher(
            odom_out_topic, Odometry, queue_size=50
        )

        rospy.loginfo("[ImuOdomFusion] IMU topic: %s", imu_topic)
        rospy.loginfo("[ImuOdomFusion] Input odom topic: %s", odom_in_topic)
        rospy.loginfo("[ImuOdomFusion] Output odom topic: %s", odom_out_topic)

    def imu_callback(self, msg):
        """Store the latest IMU orientation."""
        self.latest_imu_orientation = msg.orientation

    def odom_callback(self, odom_msg):
        """Replace odom orientation with IMU orientation and publish."""
        if self.latest_imu_orientation is None:
            rospy.logwarn_throttle(
                5.0,
                "[ImuOdomFusion] No IMU data yet. Skip fusion."
            )
            return

        fused = Odometry()
        fused.header.frame_id = "odom"
        fused.child_frame_id = odom_msg.child_frame_id

        fused.pose = odom_msg.pose
        fused.twist = odom_msg.twist

        fused.pose.pose.orientation = self.latest_imu_orientation

        self.pub_odom.publish(fused)

    def spin(self):
        rospy.loginfo("[ImuOdomFusion] Node started.")
        rospy.spin()


def main():
    node = ImuOdomFusionNode()
    node.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
