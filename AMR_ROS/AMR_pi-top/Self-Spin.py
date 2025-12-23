#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import math

def rotate_for_amcl():
    rospy.init_node('amcl_init_spinner', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    
    vel_msg = Twist()

    speed = 0.6          
    target_angle = 360
    
    angular_speed = speed
    relative_angle = target_angle * (math.pi / 180)

    rospy.loginfo("AMCL")

    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    rate = rospy.Rate(10) # 10Hz
    while current_angle < relative_angle and not rospy.is_shutdown():
        vel_msg.linear.x = 0
        vel_msg.angular.z = angular_speed
        velocity_publisher.publish(vel_msg)
        
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed * (t1 - t0)
        rate.sleep()

    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    rospy.loginfo("fin.")

if __name__ == '__main__':
    try:
        rotate_for_amcl()
    except rospy.ROSInterruptException:
        pass