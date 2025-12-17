#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>
#include <cmath>
#include <mutex>

#include "amr_db_logger/amr_state_log_writer.hpp"
#include <amr_msg/SetStatus.h>   // ✅ bool enable 서비스로 바뀐 버전

class AmrStateDbLoggerNode {
public:
  AmrStateDbLoggerNode()
  : writer_(DBConfig(), "AMR01")
  {
    ros::NodeHandle pnh("~");

    // params
    pnh.param("topic_amcl", topic_amcl_, std::string("/amcl_pose"));
    pnh.param("topic_odom", topic_odom_, std::string("/odom"));
    pnh.param("write_hz", write_hz_, 2.0);

    // stale 경고용
    pnh.param("stale_warn_sec", stale_warn_sec_, 1.0);

    // start enabled?
    pnh.param("start_enabled", enabled_, false);

    // subs
    sub_amcl_ = nh_.subscribe(topic_amcl_, 10, &AmrStateDbLoggerNode::onAmclPose, this);
    sub_odom_ = nh_.subscribe(topic_odom_, 10, &AmrStateDbLoggerNode::onOdom, this);

    // ✅ service (bool enable)
    srv_ = nh_.advertiseService("set_status", &AmrStateDbLoggerNode::onSetStatus, this);

    // timer
    timer_ = nh_.createTimer(ros::Duration(1.0 / std::max(0.1, write_hz_)),
                             &AmrStateDbLoggerNode::onTimer, this);

    if (!writer_.connect()) {
      ROS_WARN_STREAM("[DB] connect failed: " << writer_.lastError());
    } else {
      ROS_INFO("[DB] connected");
    }

    ROS_INFO_STREAM("[DB] logger started. enabled=" << (enabled_ ? "true":"false")
                    << " service=/set_status (amr_msg/SetStatus{bool enable})");
  }

private:
  // ---- Service callback ----
  bool onSetStatus(amr_msg::SetStatus::Request& req,
                   amr_msg::SetStatus::Response& res)
  {
    std::lock_guard<std::mutex> lk(mtx_);

    enabled_ = req.enable;

    res.success = true;
    res.message = enabled_ ? "DB logging enabled" : "DB logging disabled";

    ROS_WARN_STREAM("[DB] set_status(enable) = " << (enabled_ ? "true" : "false"));
    return true;
  }

  // ---- Subscribers ----
  void onAmclPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    pos_x_ = msg->pose.pose.position.x;
    pos_y_ = msg->pose.pose.position.y;

    const auto& o = msg->pose.pose.orientation;
    tf2::Quaternion q(o.x, o.y, o.z, o.w);
    heading_ = tf2::getYaw(q);

    got_amcl_ = true;
    last_amcl_rx_ = ros::Time::now();  // 수신 시각 기준
  }

  void onOdom(const nav_msgs::Odometry::ConstPtr& msg) {
    std::lock_guard<std::mutex> lk(mtx_);
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    speed_ = std::sqrt(vx*vx + vy*vy);

    got_odom_ = true;
    last_odom_rx_ = ros::Time::now();
  }

  // ---- Timer (periodic DB insert) ----
  void onTimer(const ros::TimerEvent&) {
    AmrStateData d;

    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!enabled_) return;  // ✅ enable=false면 insert 중지

      const ros::Time now = ros::Time::now();
      if (got_amcl_ && (now - last_amcl_rx_).toSec() > stale_warn_sec_) {
        ROS_WARN_THROTTLE(5.0, "[DB] AMCL stale (%.2fs) but still logging",
                          (now - last_amcl_rx_).toSec());
      }
      if (got_odom_ && (now - last_odom_rx_).toSec() > stale_warn_sec_) {
        ROS_WARN_THROTTLE(5.0, "[DB] ODOM stale (%.2fs) but still logging",
                          (now - last_odom_rx_).toSec());
      }

      if (!std::isfinite(pos_x_) || !std::isfinite(pos_y_) ||
          !std::isfinite(heading_) || !std::isfinite(speed_)) {
        ROS_WARN_THROTTLE(5.0, "[DB] non-finite data -> forced to 0");
        if (!std::isfinite(pos_x_)) pos_x_ = 0;
        if (!std::isfinite(pos_y_)) pos_y_ = 0;
        if (!std::isfinite(heading_)) heading_ = 0;
        if (!std::isfinite(speed_)) speed_ = 0;
      }

      d.pos_x = pos_x_;
      d.pos_y = pos_y_;
      d.heading = heading_;
      d.battery_pct = battery_pct_; // 추후 배터리 토픽 연결
      d.speed = speed_;
    }

    if (!writer_.insert(d)) {
      ROS_WARN_THROTTLE(5.0, "[DB] insert failed: %s", writer_.lastError().c_str());
      writer_.disconnect();
      return;
    }
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_amcl_, sub_odom_;
  ros::ServiceServer srv_;
  ros::Timer timer_;

  AmrStateLogWriter writer_;
  std::mutex mtx_;

  // params/state
  std::string topic_amcl_, topic_odom_;
  double write_hz_ = 2.0;
  double stale_warn_sec_ = 1.0;
  bool enabled_ = true;

  // cached data
  double pos_x_ = 0, pos_y_ = 0, heading_ = 0;
  double speed_ = 0, battery_pct_ = 0;

  bool got_amcl_ = false, got_odom_ = false;
  ros::Time last_amcl_rx_, last_odom_rx_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "amr_state_db_logger_node");
  AmrStateDbLoggerNode node;
  ros::spin();
  return 0;
}
