#ifndef MYAGV_H
#define MYAGV_H

#include <ros/ros.h>
#include <ros/time.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <boost/asio.hpp>
#include <sensor_msgs/Imu.h>


//#define sampleFreq	20.5f				// sample frequency in Hz
#define twoKpDef	1.0f				// (2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef	0.0f				// (2.0f * 0.0f)	// 2 * integral gain

#define OFFSET_COUNT 	200

class MyAGV
{
public:
	MyAGV();
	~MyAGV();
	bool init();
	float invSqrt(float number);
	void execute(double linearX, double linearY, double angularZ);
    void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
	void accelerometerOffset(float gx, float gy, float gz);
	void publisherInternalOdom();
  //void publisherExternalOdom();
	void publisherImuSensor();
	void publisherImuSensorRaw();

private:
	bool readSpeed();
	void writeSpeed(double movex, double movey, double rot);
	void restore();
	void restoreRun();
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
  
  ros::Subscriber sub_external_imu;  // 추가
  sensor_msgs::Imu external_imu_data;  // 추가
  bool external_imu_received;  // 추가
  
	ros::Time currentTime, lastTime;

	double x;
	double y;
	double theta;

	double vx;
	double vy;
	double vtheta;

	double ax;
	double ay;
	double az;

	double wx;
	double wy;
	double wz;
	
	double roll;
	double pitch;
	double yaw;
    
	float Gyroscope_Xdata_Offset;
	float Gyroscope_Ydata_Offset;
	float Gyroscope_Zdata_Offset;
	float sampleFreq;
	unsigned short Offest_Count;
    sensor_msgs::Imu imu_data;
	ros::NodeHandle n;
//  ros::Subscriber sub_imu;
	ros::Publisher pub_internal_imu_odom,pub_v,pub_imu,pub,pub_imu_raw; //, pub_external_imu_odom
	tf::TransformBroadcaster odomBroadcaster;
};


#endif // !MYAGV_H
