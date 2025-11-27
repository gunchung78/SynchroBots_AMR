#include <ros/ros.h>
#include <std_msgs/Int32.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "vision_result_pub");
  ros::NodeHandle nh("~");

  int value = 0;        // 0 또는 1
  bool oneshot = true;  // true면 한 번만 발행하고 종료
  double rate_hz = 1.0;
  std::string topic = "/vision_result";

  nh.param("value", value, 0);
  nh.param("oneshot", oneshot, true);
  nh.param("rate_hz", rate_hz, 1.0);
  nh.param("topic", topic, std::string("/vision_result"));

  // latched publisher
  ros::Publisher pub = nh.advertise<std_msgs::Int32>(topic, 1, /*latch=*/true);

  std_msgs::Int32 msg;
  msg.data = (value != 0) ? 1 : 0;
  ROS_INFO("Publishing vision_result=%d on %s (latched, oneshot=%s)", msg.data, topic.c_str(), oneshot?"true":"false");

  if (oneshot) {
    pub.publish(msg);
    ros::spinOnce();
    ROS_INFO("Done (oneshot).");
    return 0;
  } else {
    ros::Rate r(rate_hz);
    while (ros::ok()) {
      pub.publish(msg);
      ros::spinOnce();
      r.sleep();
    }
  }
  return 0;
}
