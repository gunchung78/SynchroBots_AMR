#include <vector>
#include <iostream>
#include <iomanip>
#include <time.h>
#include <cstdint>   // uint8_t

// ? 누락되기 쉬운 필수 헤더들
#include <thread>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <limits>

#include "myagv_odometry/myAGV.h"
#include "std_msgs/Int8.h"
#include <std_msgs/Float32.h>
#include <sensor_msgs/BatteryState.h>

#include <geometry_msgs/Vector3Stamped.h>  // RPY
#include <tf/transform_datatypes.h>        // tf::createQuaternionMsgFromYaw

const unsigned char header[2] = {0xfe, 0xfe};

boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/ttyAMA2");

boost::array<double, 36> odom_pose_covariance = {{
    1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9
}};

boost::array<double, 36> odom_twist_covariance = {{
    1e-9, 0, 0, 0, 0, 0,
    0, 1e-3, 1e-9, 0, 0, 0,
    0, 0, 1e6, 0, 0, 0,
    0, 0, 0, 1e6, 0, 0,
    0, 0, 0, 0, 1e6, 0,
    0, 0, 0, 0, 0, 1e-9
}};

// ============================================================================
// Battery voltage extract (CODE1 방식: readSpeed()에서 받은 buf[27]에서만 추출)
// - UART에 추가 명령을 보내지 않음 (주행 방해/오동작 방지)
// - 0x80(128) 같은 중립값 오인 방지
// - last_v(Backup_Battery_voltage) 근접값 우선 선택
// ============================================================================
static bool extractBatteryVoltageFromFrame(
    const unsigned char buf[27],
    double last_v,
    double &voltage_out,
    int &raw01v_out
)
{
    struct Cand { double v; int raw01v; int src; double score; };
    std::vector<Cand> cands;

    auto is_neutralish = [&](int raw) {
        return (raw >= 126 && raw <= 130) || raw == 0 || raw == 255;
    };

    auto add_raw01v = [&](int raw01v, int src) {
        if (is_neutralish(raw01v)) return;

        double v = raw01v / 10.0;

        // 3S 배터리 범위 (필요하면 조정)
        if (v < 8.0 || v > 14.5) return;

        double score = 0.0;
        if (last_v > 1.0) score += std::abs(v - last_v) * 10.0; // 이전값 근접 우선
        if (std::abs(v - 12.8) < 0.05) score += 5.0;            // 12.8 근처 고정 오인 페널티

        cands.push_back({v, raw01v, src, score});
    };

    // 1바이트 후보들 (buf[2]는 중립값 확률 높아서 제외)
    add_raw01v((int)buf[15], 15);
    add_raw01v((int)buf[16], 16);
    add_raw01v((int)buf[25], 25);

    // 2바이트 후보 (16,17)
    auto u16 = [&](int lo, int hi)->uint16_t {
        return (uint16_t)buf[lo] | ((uint16_t)buf[hi] << 8);
    };
    uint16_t w = u16(16, 17);

    // mV 가정: 11300 -> 11.3V
    {
        double v = (double)w / 1000.0;
        int raw = (int)std::lround(v * 10.0);
        add_raw01v(raw, 1617);
    }
    // 0.01V 가정: 1130 -> 11.30V
    {
        double v = (double)w / 100.0;
        int raw = (int)std::lround(v * 10.0);
        add_raw01v(raw, 1716);
    }

    if (cands.empty()) return false;

    auto best = *std::min_element(
        cands.begin(), cands.end(),
        [](const Cand& a, const Cand& b){ return a.score < b.score; }
    );

    // legacy /Voltage(Int8) 때문에 0~127 clamp
    int raw = std::max(0, std::min(127, best.raw01v));

    voltage_out = best.v;
    raw01v_out  = raw;
    return true;
}

// =========================
// MyAGV
// =========================
MyAGV::MyAGV()
{
    x = 0.0;
    y = 0.0;
    theta = 0.0;

    vx = 0.0;
    vy = 0.0;
    vtheta = 0.0;

    have_imu_  = false;
    imu_roll_  = 0.0;
    imu_pitch_ = 0.0;
    imu_yaw_   = 0.0;

    // battery init
    Battery_voltage = 0.0;
    Backup_Battery_voltage = 0.0;

    batt_v_full_   = 12.6;
    batt_v_empty_  = 10.5;
    batt_alpha_    = 0.1;
    batt_period_   = 1.0;
    last_batt_pub_ = ros::Time(0);
    batt_v_filt_   = std::numeric_limits<double>::quiet_NaN();
}

MyAGV::~MyAGV() {}

bool MyAGV::init()
{
    sp.set_option(boost::asio::serial_port::baud_rate(115200));
    sp.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
    sp.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
    sp.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
    sp.set_option(boost::asio::serial_port::character_size(8));

    ros::Time::init();
    currentTime = ros::Time::now();
    lastTime    = ros::Time::now();

    pub   = n.advertise<nav_msgs::Odometry>("odom", 50);
    pub_v = n.advertise<std_msgs::Int8>("Voltage", 1000);

    // battery publishers (CODE1 구조)
    pub_battery_state   = n.advertise<sensor_msgs::BatteryState>("battery_state", 10);
    pub_battery_voltage = n.advertise<std_msgs::Float32>("battery_voltage", 10);
    pub_battery_percent = n.advertise<std_msgs::Int8>("battery_percent", 10);

    // battery params
    n.param("battery_v_full",   batt_v_full_,  12.6);
    n.param("battery_v_empty",  batt_v_empty_, 10.5);
    n.param("battery_alpha",    batt_alpha_,   0.1);
    n.param("battery_period",   batt_period_,  1.0);

    imu_sub_ = n.subscribe("/external_imu_rpy", 50, &MyAGV::imuCallback, this);

    restore();
    return true;
}

void MyAGV::restore()
{
    boost::asio::streambuf clear_buffer;
    boost::asio::read(sp, clear_buffer, boost::asio::transfer_at_least(1));

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    unsigned char cmd[6] = {0xfe, 0xfe, 0x01, 0x00, 0x01, 0x02};
    boost::asio::write(sp, boost::asio::buffer(cmd, 6));

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

void MyAGV::restoreRun()
{
    int res = 0;
    std::cout << "if you want restore run, pls input 1, then press enter" << std::endl;
    while (res != 1) std::cin >> res;

    restore();
    std::cout << "restore finished" << std::endl;
}

bool MyAGV::readSpeed()
{
    unsigned char buf_header[1] = {0};
    unsigned char buf[27] = {0};

    size_t ret;
    boost::system::error_code er2;

    // ---- find header ----
    bool header_found = false;
    while (!header_found) {
        ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
        if (ret != 1) continue;
        if (buf_header[0] != header[0]) continue;

        ret = boost::asio::read(sp, boost::asio::buffer(buf_header), er2);
        if (ret != 1) continue;
        if (buf_header[0] != header[0]) continue;

        header_found = true;
    }

    // ---- read frame ----
    ret = boost::asio::read(sp, boost::asio::buffer(buf), er2);
    if (ret != 27) {
        ROS_ERROR("Read error %ld", ret);
        return false;
    }

    // ---- checksum ----
    int check = 0;
    for (int i = 0; i < 26; ++i) check += buf[i];
    if (check % 256 != buf[26]) {
        ROS_ERROR("Checksum error! %d -- %d", check, buf[26]);
        return false;
    }

    // ---- decode ----
    const int index = 0;

    vx     = (static_cast<double>(buf[index])     - 128.0) * 0.01;
    vy     = (static_cast<double>(buf[index + 1]) - 128.0) * 0.01;
    vtheta = (static_cast<double>(buf[index + 2]) - 128.0) * 0.01;

    ax = ((buf[index + 3] +  buf[index + 4] * 256)  - 10000) * 0.001;
    ay = ((buf[index + 5] +  buf[index + 6] * 256)  - 10000) * 0.001;
    az = ((buf[index + 7] +  buf[index + 8] * 256)  - 10000) * 0.001;

    wx = ((buf[index + 9]  + buf[index + 10] * 256) - 10000) * 0.1;
    wy = ((buf[index + 11] + buf[index + 12] * 256) - 10000) * 0.1;
    wz = ((buf[index + 13] + buf[index + 14] * 256) - 10000) * 0.1;

    // ---- battery parse from SAME frame (no extra UART command) ----
    {
        double v = 0.0;
        int raw01v = 0;
        if (extractBatteryVoltageFromFrame(buf, Backup_Battery_voltage, v, raw01v)) {
            Battery_voltage = v;
            Backup_Battery_voltage = v;
        } else {
            Battery_voltage = Backup_Battery_voltage;
        }
    }

    // ---- odom integrate ----
    currentTime = ros::Time::now();
    double dt = (currentTime - lastTime).toSec();

    double delta_x  = (vx * std::cos(theta) - vy * std::sin(theta)) * dt;
    double delta_y  = (vx * std::sin(theta) + vy * std::cos(theta)) * dt;
    double delta_th = vtheta * dt;

    x     += delta_x;
    y     += delta_y;
    theta += delta_th;

    lastTime = currentTime;
    return true;
}

void MyAGV::updateBatteryPublish()
{
    ros::Time now = ros::Time::now();
    if ((now - last_batt_pub_).toSec() < batt_period_) return;
    last_batt_pub_ = now;

    double v = Battery_voltage;
    if (v <= 0.1) {
        ROS_WARN_THROTTLE(5.0, "Battery_voltage not ready (v=%.3f).", v);
        return;
    }

    // low-pass filter (EMA)
    if (std::isnan(batt_v_filt_)) batt_v_filt_ = v;
    batt_v_filt_ = (1.0 - batt_alpha_) * batt_v_filt_ + batt_alpha_ * v;

    // percent (0~1)
    double denom = (batt_v_full_ - batt_v_empty_);
    double pct = (denom > 1e-6) ? ((batt_v_filt_ - batt_v_empty_) / denom) : 0.0;
    pct = std::max(0.0, std::min(1.0, pct));

    // /battery_state
    sensor_msgs::BatteryState bs;
    bs.header.stamp = now;
    bs.voltage = static_cast<float>(batt_v_filt_);
    bs.current = std::numeric_limits<float>::quiet_NaN();
    bs.percentage = static_cast<float>(pct);
    bs.present = true;
    pub_battery_state.publish(bs);

    // /battery_voltage
    std_msgs::Float32 vmsg;
    vmsg.data = static_cast<float>(batt_v_filt_);
    pub_battery_voltage.publish(vmsg);

    // /battery_percent (0~100)
    std_msgs::Int8 pmsg;
    pmsg.data = static_cast<int8_t>(std::lround(pct * 100.0));
    pub_battery_percent.publish(pmsg);

    // legacy /Voltage: 0.1V raw (Int8)
    int raw = static_cast<int>(std::lround(batt_v_filt_ * 10.0)); // 11.3V -> 113
    raw = std::max(0, std::min(127, raw));
    std_msgs::Int8 vraw;
    vraw.data = static_cast<int8_t>(raw);
    pub_v.publish(vraw);
}

void MyAGV::writeSpeed(double movex, double movey, double rot)
{
    // ? 배터리 분기(10,10,10) 제거: 주행 전송만
    if (movex > 1.0) movex = 1.0;
    if (movex < -1.0) movex = -1.0;
    if (movey > 1.0) movey = 1.0;
    if (movey < -1.0) movey = -1.0;
    if (rot > 1.0)   rot   = 1.0;
    if (rot < -1.0)  rot   = -1.0;

    unsigned char x_send   = static_cast<signed char>(movex * 100) + 128;
    unsigned char y_send   = static_cast<signed char>(movey * 100) + 128;
    unsigned char rot_send = static_cast<signed char>(rot   * 100) + 128;
    unsigned char check    = x_send + y_send + rot_send;

    // ? 6바이트만 전송 (스트림 꼬임 방지)
    unsigned char out[6] = {
        header[0], header[1],
        x_send, y_send, rot_send,
        check
    };

    boost::asio::write(sp, boost::asio::buffer(out, 6));
}

bool MyAGV::execute(double linearX, double linearY, double angularZ)
{
    // 주행 명령 송신
    writeSpeed(linearX, linearY, angularZ);

    // 프레임 수신/해석
    if (!readSpeed()) {
        ROS_WARN("readSpeed failed!");
        return false;
    }

    // ? battery publish (UART 추가 접근 없음)
    updateBatteryPublish();

    // IMU yaw 적용 (기존 코드2 유지)
    double yaw_for_odom = theta;
    if (have_imu_) {
        yaw_for_odom = imu_yaw_;
        theta        = imu_yaw_;
    }

    geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(yaw_for_odom);

    // publish odometry
    nav_msgs::Odometry msg;
    msg.header.stamp = currentTime;
    msg.header.frame_id = "odom";

    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation = odom_quat;
    msg.pose.covariance       = odom_pose_covariance;

    msg.child_frame_id        = "base_footprint";
    msg.twist.twist.linear.x  = vx;
    msg.twist.twist.linear.y  = vy;
    msg.twist.twist.angular.z = vtheta;
    msg.twist.covariance      = odom_twist_covariance;

    pub.publish(msg);
    return true;
}

void MyAGV::imuCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    imu_roll_  = msg->vector.x;
    imu_pitch_ = msg->vector.y;
    imu_yaw_   = msg->vector.z;
    have_imu_ = true;
}
