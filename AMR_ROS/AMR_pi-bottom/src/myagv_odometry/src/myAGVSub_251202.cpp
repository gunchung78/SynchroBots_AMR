#include "myagv_odometry/myAGV.h"
#include <iostream>

double linearX = 0.0;
double linearY = 0.0;
double angularZ = 0.0;

void cmdCallback(const geometry_msgs::Twist& msg)
{
	linearX = msg.linear.x;
	linearY = msg.linear.y;
	angularZ = msg.angular.z;
//	std:: "cmdCallback: " << msg.linear.x << ", linearX: " << linearX << ", linearY: " << linearY <<", angularZ: " << angularZ <<std::endl;
}

//void MyAGV::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
//{
//    // orientation (필요한 경우 사용)
//    imu_data.orientation = msg->orientation;
//
//    // angular velocity
//    imu_data.angular_velocity = msg->angular_velocity;
//
//    // linear acceleration
//    imu_data.linear_acceleration = msg->linear_acceleration;
//
//    // 만약 yaw만 쓰고 싶으면 (예: vtheta 대체)
//    // tf::Quaternion q(
//    //     msg->orientation.x,
//    //     msg->orientation.y,
//    //     msg->orientation.z,
//    //     msg->orientation.w
//    // );
//    // tf::Matrix3x3 m(q);
//    // double roll, pitch, yaw_tmp;
//    // m.getRPY(roll, pitch, yaw_tmp);
//    // this->theta = yaw_tmp;   // ★ 외부 IMU의 yaw 반영 (필요 시)
//}

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
