#include "myagv_odometry/myAGV.h"
#include "sensor_msgs/Imu.h"
#include <iostream>
#include <tf/tf.h>

double linearX = 0.0;
double linearY = 0.0;
double angularZ = 0.0;

void cmdCallback(const geometry_msgs::Twist& msg)
{
	linearX = msg.linear.x;
	linearY = msg.linear.y;
	angularZ = msg.angular.z;

}

void imuCallback(const sensor_msgs::Imu& msg)
{
  imu_received = true;
  imu_yaw = tf::getYaw(msg.orientation); // quaternion ¡æ yaw
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "myagv_odometry_node");
	ros::NodeHandle n;
	MyAGV myAGV;

	if (!myAGV.init())
		ROS_ERROR("myAGV initialized failed!");
	ROS_INFO("myAGV initialized successful!");

	ros::Subscriber sub = n.subscribe("cmd_vel", 50, cmdCallback);
	ros::Rate loop_rate(100);

	
	while (ros::ok())
	{
		ros::spinOnce();
		myAGV.execute(linearX, linearY, angularZ);
		loop_rate.sleep();
	}

	return 0;
}
