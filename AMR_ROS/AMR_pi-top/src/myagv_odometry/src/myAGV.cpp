#include <vector>
#include <iostream>
#include <iomanip>
#include <time.h>
#include <cstdint>  // uint8_t

#include "myagv_odometry/myAGV.h"
#include "std_msgs/Int8.h"
#include <sensor_msgs/Imu.h>  // <-- added for IMU message

// const unsigned char ender[2] = { 0x0d, 0x0a };
const unsigned char header[2] = { 0xfe, 0xfe };
// const int SPEED_INFO = 0xa55a;
// const int GET_SPEED = 0xaaaa;
// const double ROBOT_RADIUS = 105.00;
// const double ROBOT_LENGTH = 210.50;

boost::asio::io_service iosev;
// boost::asio::serial_port sp(iosev, "/dev/ttyUSB0");
boost::asio::serial_port sp(iosev, "/dev/ttyAMA2");

boost::array<double, 36> odom_pose_covariance = {
    {1e-9, 0, 0, 0, 0, 0,
     0, 1e-3, 1e-9, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e-9} };

boost::array<double, 36> odom_twist_covariance = {
    {1e-9, 0, 0, 0, 0, 0,
     0, 1e-3, 1e-9, 0, 0, 0,
     0, 0, 1e6, 0, 0, 0,
     0, 0, 0, 1e6, 0, 0,
     0, 0, 0, 0, 1e6, 0,
     0, 0, 0, 0, 0, 1e-9} };

void send()
{
    ;
}

void receive()
{
    ;
}

MyAGV::MyAGV()
{
    x = 0.0;
    y = 0.0;
    theta = 0.0;

    vx = 0.0;
    vy = 0.0;
    vtheta = 0.0;

    ax = 0.0;
    ay = 0.0;
    az = 0.0;

    wx = 0.0;
    wy = 0.0;
    wz = 0.0;
}

MyAGV::~MyAGV()
{
    ;
}

bool MyAGV::init()
{
    sp.set_option(boost::asio::serial_port::baud_rate(115200));
    sp.set_option(boost::asio::serial_port::flow_control(
        boost::asio::serial_port::flow_control::none));
    sp.set_option(boost::asio::serial_port::parity(
        boost::asio::serial_port::parity::none));
    sp.set_option(boost::asio::serial_port::stop_bits(
        boost::asio::serial_port::stop_bits::one));
    sp.set_option(boost::asio::serial_port::character_size(8));

    ros::Time::init();
    currentTime = ros::Time::now();
    lastTime = ros::Time::now();

    pub   = n.advertise<nav_msgs::Odometry>("odom", 50);
    pub_v = n.advertise<std_msgs::Int8>("Voltage", 1000);
    pub_imu = n.advertise<sensor_msgs::Imu>("imu_raw_data", 50);  // IMU publisher

    restore();  // first restore, abort current err, do not restore
    return true;
}

void MyAGV::restore()
{
    boost::asio::streambuf clear_buffer;
    boost::asio::read(sp, clear_buffer, boost::asio::transfer_at_least(1));

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    unsigned char cmd[6] = {0xfe, 0xfe, 0x01, 0x00, 0x01, 0x02};

    std::cout << "Sending data: ";
    for (int i = 0; i < 6; ++i) {
        std::cout << std::hex << std::setfill('0') << std::setw(2)
                  << (int)(cmd[i]) << " ";
    }
    std::cout << std::dec << std::endl;

    boost::asio::write(sp, boost::asio::buffer(cmd));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    return;
}

void MyAGV::restoreRun()
{
    int res = 0;
    std::cout << "if you want restore run, please input 1, then press enter"
              << std::endl;
    while (res != 1) {
        std::cin >> res;
        std::cout << "press enter" << std::endl;
        std::cout << res;
    }
    restore();
    std::cout << "restore finished" << std::endl;
    return;
}

bool MyAGV::readSpeed()
{
    int i, length = 0, count = 0;
    unsigned char checkSum;
    unsigned char buf_header[1] = {0};
    unsigned char buf[27] = {0};

    size_t ret;
    boost::system::error_code er2;
    bool header_found = false;
    while (!header_found) {
        ++count;
        ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
        if (ret != 1) {
            continue;
        }
        if (buf_header[0] != header[0]) {
            continue;
        }
        bool header_2_found = false;
        while (!header_2_found) {
            ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
            if (ret != 1) {
                continue;
            }
            if (buf_header[0] != header[0]) {
                continue;
            }
            header_2_found = true;
        }
        header_found = true;
    }

    ret = boost::asio::read(sp, boost::asio::buffer(buf), er2);
    std::cout << std::endl;
    if ((buf[0] + buf[1] + buf[2] + buf[3]) == buf[4]) {
        int wheel_num = 0;
        for (int i = 0; i < 4; ++i) {
            if (buf[i] == 1) {
                wheel_num = i + 1;
                ROS_ERROR("ERROR %d wheel current > 800", wheel_num);
            }
        }
        restoreRun();
        return false;
    }
    if (ret != 27) {
        ROS_ERROR("Read error %ld", ret);
        return false;
    }

    int index = 0;
    int check = 0;
    for (int j = 0; j < 26; ++j)
        check += buf[j];
    if (check % 256 != buf[index + 26]) {
        ROS_ERROR("error 3! %d -- %d", check, buf[index + 26]);
        return false;
    }

    vx = (static_cast<double>(buf[index]) - 128.0) * 0.01;
    vy = (static_cast<double>(buf[index + 1]) - 128.0) * 0.01;
    vtheta = (static_cast<double>(buf[index + 2]) - 128.0) * 0.01;

    ax = ((buf[index + 3] + buf[index + 4] * 256) - 10000) * 0.001;
    ay = ((buf[index + 5] + buf[index + 6] * 256) - 10000) * 0.001;
    az = ((buf[index + 7] + buf[index + 8] * 256) - 10000) * 0.001;

    wx = ((buf[index + 9] + buf[index + 10] * 256) - 10000) * 0.1;
    wy = ((buf[index + 11] + buf[index + 12] * 256) - 10000) * 0.1;
    wz = ((buf[index + 13] + buf[index + 14] * 256) - 10000) * 0.1;

    currentTime = ros::Time::now();

    // Publish IMU raw data
    // Note: static transform publisher uses /imu_raw as frame, so match it here.
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = currentTime;
    imu_msg.header.frame_id = "imu_raw";

    // Orientation is not computed here, so set identity and high covariance.
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;
    imu_msg.orientation_covariance[0] = 1e6;
    imu_msg.orientation_covariance[4] = 1e6;
    imu_msg.orientation_covariance[8] = 1e6;

    // Angular velocity: convert from deg/s (0.1 scale) to rad/s.
    const double deg2rad = 3.14159265358979323846 / 180.0;
    imu_msg.angular_velocity.x = wx * deg2rad;
    imu_msg.angular_velocity.y = wy * deg2rad;
    imu_msg.angular_velocity.z = wz * deg2rad;
    imu_msg.angular_velocity_covariance[0] = 1e-3;
    imu_msg.angular_velocity_covariance[4] = 1e-3;
    imu_msg.angular_velocity_covariance[8] = 1e-3;

    // Linear acceleration (already scaled from raw).
    imu_msg.linear_acceleration.x = ax;
    imu_msg.linear_acceleration.y = ay;
    imu_msg.linear_acceleration.z = az;
    imu_msg.linear_acceleration_covariance[0] = 1e-2;
    imu_msg.linear_acceleration_covariance[4] = 1e-2;
    imu_msg.linear_acceleration_covariance[8] = 1e-2;

    pub_imu.publish(imu_msg);

    double dt = (currentTime - lastTime).toSec();
    double delta_x = (vx * cos(theta) - vy * sin(theta)) * dt;
    double delta_y = (vx * sin(theta) + vy * cos(theta)) * dt;
    double delta_th = vtheta * dt;

    x += delta_x;
    y += delta_y;
    theta += delta_th;
    lastTime = currentTime;

    return true;
}

void MyAGV::writeSpeed(double movex, double movey, double rot)
{
    if (movex == 10 && movey == 10 && rot == 10) {
        uint8_t buf[7] = {0xfe, 0xfe, 0x01, 0x01, 0x01, 0x03};
        boost::asio::write(sp, boost::asio::buffer(buf));
        unsigned char buf_header[1] = {0};

        size_t ret;
        boost::system::error_code er2;
        bool header_found = false;
        time_t now_t = time(NULL);
        while (true) {
            ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);

            if (ret != 1) {
                continue;
            }
            if (buf_header[0] != header[0]) {
                continue;
            }
            bool header_2_found = false;
            while (!header_2_found) {
                ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
                if (ret != 1) {
                    continue;
                }
                if (buf_header[0] != header[0]) {
                    continue;
                }
                header_2_found = true;
            }
            header_found = true;
            ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
            if (buf_header[0] == 0x01) {
                ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
                if (buf_header[0] == 0x01) {
                    ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
                    std_msgs::Int8 msg;
                    msg.data = (int)buf_header[0] / 10;
                    ROS_INFO("Voltage: %d", msg.data);
                    pub_v.publish(msg);
                    break;
                }
            }
            if (time(NULL) - now_t > 3) {
                ROS_ERROR("Get Voltage timeout");
                break;
            }
        }
    } else {
        if (movex > 1.0) movex = 1.0;
        if (movex < -1.0) movex = -1.0;
        if (movey > 1.0) movey = 1.0;
        if (movey < -1.0) movey = -1.0;
        if (rot > 1.0) rot = 1.0;
        if (rot < -1.0) rot = -1.0;

        unsigned char x_send = static_cast<signed char>(movex * 100) + 128;
        unsigned char y_send = static_cast<signed char>(movey * 100) + 128;
        unsigned char rot_send = static_cast<signed char>(rot * 100) + 128;
        unsigned char check = x_send + y_send + rot_send;

        char buf2[8] = {0};
        buf2[0] = header[0];
        buf2[1] = header[1];
        buf2[2] = x_send;
        buf2[3] = y_send;
        buf2[4] = rot_send;
        buf2[5] = check;

        boost::asio::write(sp, boost::asio::buffer(buf2));
    }
}

bool MyAGV::execute(double linearX, double linearY, double angularZ)
{
    writeSpeed(linearX, linearY, angularZ);
    readSpeed();  // easy to report error

    geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(theta);  // THETA

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = currentTime;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odomBroadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry msgl;
    msgl.header.stamp = currentTime;
    msgl.header.frame_id = "odom";

    msgl.pose.pose.position.x = x;
    msgl.pose.pose.position.y = y;
    msgl.pose.pose.position.z = 0.0;
    msgl.pose.pose.orientation = odom_quat;
    msgl.pose.covariance = odom_pose_covariance;

    msgl.child_frame_id = "base_footprint";
    msgl.twist.twist.linear.x = vx;
    msgl.twist.twist.linear.y = vy;
    msgl.twist.twist.angular.z = vtheta;
    msgl.twist.covariance = odom_twist_covariance;

    pub.publish(msgl);
    lastTime = currentTime;

    return true;
}
