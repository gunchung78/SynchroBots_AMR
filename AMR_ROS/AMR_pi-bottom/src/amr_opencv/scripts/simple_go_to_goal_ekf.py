#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion


class SimpleGoToGoalEkf:
    def __init__(self):
        # Parameters
        self.goal_x = rospy.get_param("~goal_x", 1.0)
        self.goal_y = rospy.get_param("~goal_y", 0.0)
        self.dist_threshold = rospy.get_param("~dist_threshold", 0.05)
        self.forward_speed = rospy.get_param("~forward_speed", 0.1)

        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        # Use EKF fused odom (but its type is nav_msgs/Odometry in your system)
        self.odom_topic = rospy.get_param(
            "~odom_topic", "/robot_pose_ekf/odom_combined"
        )

        # Internal state
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.arrived = False

        # Pub/Sub
        self.cmd_pub = rospy.Publisher(
            self.cmd_vel_topic,
            Twist,
            queue_size=1
        )

        self.odom_sub = rospy.Subscriber(
            self.odom_topic,
            Odometry,
            self.odom_callback,
            queue_size=1,
        )

        rospy.loginfo(
            "[SimpleGoToGoalEkf] Using odom topic: %s (type=nav_msgs/Odometry)",
            self.odom_topic,
        )
        rospy.loginfo(
            "[SimpleGoToGoalEkf] Goal = (%.3f, %.3f), threshold = %.3f",
            self.goal_x,
            self.goal_y,
            self.dist_threshold,
        )

    def odom_callback(self, msg: Odometry):
        """Callback for fused odom from robot_pose_ekf."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion(
            [q.x, q.y, q.z, q.w]
        )
        self.current_yaw = yaw

    def compute_cmd(self) -> Twist:
        twist = Twist()

        # Already arrived -> keep publishing zero cmd_vel
        if self.arrived:
            return twist

        # No pose info yet
        if self.current_x is None or self.current_y is None:
            rospy.logwarn_throttle(
                2.0,
                "[SimpleGoToGoalEkf] No odom_combined data yet."
            )
            return twist

        # Compute distance to goal
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dist = math.sqrt(dx * dx + dy * dy)

        # Check if we are close enough
        if dist < self.dist_threshold:
            rospy.loginfo(
                "[SimpleGoToGoalEkf] Arrived at goal (dist=%.3f). Stopping.",
                dist,
            )
            self.arrived = True
            # twist is zero by default
            return twist

        # Move forward with constant speed
        twist.linear.x = self.forward_speed
        # Note: heading control (angular.z) can be added later if needed.

        return twist

    def spin(self):
        rate = rospy.Rate(20)
        rospy.loginfo("[SimpleGoToGoalEkf] Spin started.")
        while not rospy.is_shutdown():
            cmd = self.compute_cmd()
            self.cmd_pub.publish(cmd)
            rate.sleep()


def main():
    rospy.init_node("simple_go_to_goal_ekf")
    node = SimpleGoToGoalEkf()
    node.spin()


if __name__ == "__main__":
    main()
