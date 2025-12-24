#include <ros/ros.h>
#include <std_msgs/String.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <std_srvs/SetBool.h>
#include <amr_msg/SetMcuValue.h>
#include <amr_msg/SetStatus.h>
#include <amr_msg/SetMission.h>

#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <geometry_msgs/Twist.h>

/**
 * Type definition for MoveBase Action Client
 */
using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

class AmrGoMove
{
public:
  AmrGoMove()
      : ac_("move_base", true),
        last_move_command_(""),
        last_positions_command_(""),
        stop_requested_(false)
  {
    ros::NodeHandle private_nh("~");

    // --- Load Parameters: Logical destinations (go_move) ---
    private_nh.param("pick_up_zone_x", pick_up_zone_x_, 1.0);
    private_nh.param("pick_up_zone_y", pick_up_zone_y_, 0.0);
    private_nh.param("pick_up_zone_yaw_deg", pick_up_zone_yaw_deg_, 0.0);

    private_nh.param("go_standby_x", go_standby_x_, 1.0);
    private_nh.param("go_standby_y", go_standby_y_, 0.0);
    private_nh.param("go_standby_yaw_deg", go_standby_yaw_deg_, 0.0);
    
    private_nh.param("go_home_x", go_home_x_, 1.0);
    private_nh.param("go_home_y", go_home_y_, 0.0);
    private_nh.param("go_home_yaw_deg", go_home_yaw_deg_, 0.0);

    // --- Load Parameters: Object destinations (go_positions) ---
    private_nh.param("esp32_x", esp32_x_, 2.0);
    private_nh.param("esp32_y", esp32_y_, 1.0);
    private_nh.param("esp32_yaw_deg", esp32_yaw_deg_, 0.0);

    private_nh.param("motordriver_x", motordriver_x_, 2.0);
    private_nh.param("motordriver_y", motordriver_y_, 1.0);
    private_nh.param("motordriver_yaw_deg", motordriver_yaw_deg_, 0.0);

    private_nh.param("powersupply_x", powersupply_x_, 2.0);
    private_nh.param("powersupply_y", powersupply_y_, 1.0);
    private_nh.param("powersupply_yaw_deg", powersupply_yaw_deg_, 0.0);

    // --- Initialize Subscribers ---
    sub_move_ = nh_.subscribe(
        "/amr/opcua/go_move",
        10,
        &AmrGoMove::opcuaMoveCallback,
        this);

    sub_positions_ = nh_.subscribe(
        "/amr/opcua/go_positions",
        10,
        &AmrGoMove::opcuaPositionsCallback,
        this);

    // --- Initialize Publishers ---
    mission_pub_ = nh_.advertise<std_msgs::String>("amr_mission_state", 10);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // --- Initialize Service Clients ---
    go_aruco_client_ = nh_.serviceClient<std_srvs::SetBool>("/go_aruco");
    amr_mcu_client_ = nh_.serviceClient<amr_msg::SetMcuValue>("/amr_mcu");
    set_status_client_ = nh_.serviceClient<amr_msg::SetStatus>("/set_status");
    set_mission_client_ = nh_.serviceClient<amr_msg::SetMission>("/set_mission");

    // --- Initialize Service Server ---
    amr_mcu_service_server_ =
        nh_.advertiseService("/amr_mcu_service",
                             &AmrGoMove::amrMcuServiceCallback,
                             this);

    // Connect to move_base
    ROS_INFO("[amr_go_move] Waiting for move_base action server...");
    ac_.waitForServer();
    ROS_INFO("[amr_go_move] Connected to move_base action server.");
  }

private:
  /**
   * ALL Stop
   */
  bool stop_requested_;
  void stopRobot() {
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    cmd_vel_pub_.publish(stop_msg);
    ROS_INFO("[amr_go_move] Emergency Stop: cmd_vel 0 published.");
  }
  
  /**
   * Publishes velocity commands for a specific duration
   */
  void moveManual(double linear_x, double angular_z, double duration_sec)
  {
      geometry_msgs::Twist msg;
      msg.linear.x = linear_x;
      msg.angular.z = angular_z;
  
      ros::Time start_time = ros::Time::now();
      
      // Wait for a valid ROS clock
      while(ros::ok() && start_time.toSec() == 0) {
          ros::spinOnce();
          start_time = ros::Time::now();
          ros::Duration(0.1).sleep();
      }
  
      ros::Rate rate(10); // 10Hz
      double elapsed = 0.0;
  
      ROS_INFO("[amr_go_move] Manual move started: v=%.2f, w=%.2f for %.1f sec", 
               linear_x, angular_z, duration_sec);
  
      while (ros::ok())
      {
          elapsed = (ros::Time::now() - start_time).toSec();
          if (elapsed >= duration_sec) break; 
  
          cmd_vel_pub_.publish(msg);
          
          if (std::fmod(elapsed, 1.0) < 0.1) {
              ROS_INFO("[amr_go_move] Moving... elapsed: %.1f / %.1f", elapsed, duration_sec);
          }
  
          ros::spinOnce();
          rate.sleep();
      }
  
      // Stop the robot
      msg.linear.x = 0.0;
      msg.angular.z = 0.0; 
      for(int i=0; i<5; i++) {
          cmd_vel_pub_.publish(msg);
          ros::Duration(0.05).sleep();
      }
      
      ROS_INFO("[amr_go_move] Manual move completed.");
  }
    
  /**
   * Utility: Trims whitespace from both ends of a string
   */
  static std::string trim(const std::string &s)
  {
    std::size_t start = 0;
    while (start < s.size() &&
           (s[start] == ' ' || s[start] == '\t' ||
            s[start] == '\n' || s[start] == '\r'))
    {
      ++start;
    }

    if (start >= s.size())
    {
      return std::string();
    }

    std::size_t end = s.size();
    while (end > start &&
           (s[end - 1] == ' ' || s[end - 1] == '\t' ||
            s[end - 1] == '\n' || s[end - 1] == '\r'))
    {
      --end;
    }

    return s.substr(start, end - start);
  }

  /**
   * Utility: Crude JSON parser to extract 'move_command' value
   */
  std::string parseMoveCommand(const std::string &json_str) const
  {
    const std::string key = "\"move_command\"";
    std::size_t key_pos = json_str.find(key);
    if (key_pos == std::string::npos) return std::string();

    std::size_t colon_pos = json_str.find(':', key_pos + key.size());
    if (colon_pos == std::string::npos) return std::string();

    std::size_t first_quote = json_str.find('"', colon_pos);
    if (first_quote == std::string::npos) return std::string();

    std::size_t second_quote = json_str.find('"', first_quote + 1);
    if (second_quote == std::string::npos) return std::string();

    std::string value = json_str.substr(first_quote + 1, second_quote - first_quote - 1);
    return trim(value);
  }

  /**
   * Utility: Crude JSON parser to extract list of strings from '[' ']'
   */
  std::vector<std::string> parseObjectInfoList(const std::string &json_str) const
  {
    std::vector<std::string> result;
    std::size_t start = json_str.find('[');
    std::size_t end = std::string::npos;
    if (start != std::string::npos) end = json_str.find(']', start);

    if (start == std::string::npos || end == std::string::npos || end <= start + 1)
    {
      ROS_WARN("[amr_go_move] Failed to parse object_info list.");
      return result;
    }

    std::string inside = json_str.substr(start + 1, end - start - 1);
    std::stringstream ss(inside);
    std::string item;
    while (std::getline(ss, item, ','))
    {
      std::string token = trim(item);
      if (!token.empty() && token.front() == '"' && token.back() == '"' && token.size() >= 2)
      {
        token = token.substr(1, token.size() - 2);
      }
      token = trim(token);
      if (!token.empty()) result.push_back(token);
    }
    return result;
  }

  /**
   * Call ArUco marker alignment service
   */
  bool callGoAruco(bool enable)
  {
    std_srvs::SetBool srv;
    srv.request.data = enable;
    if (!go_aruco_client_.call(srv))
    {
      ROS_ERROR("[amr_go_move] Failed to call /go_aruco service.");
      return false;
    }
    return srv.response.success;
  }

  /**
   * Send command value to MCU with retry logic
   */
  bool callAmrMcu(int value, const std::string &tag, int retry_count = 3)
  {
    amr_msg::SetMcuValue srv;
    srv.request.value = value;
    for (int attempt = 1; attempt <= retry_count; ++attempt)
    {
      if (amr_mcu_client_.call(srv) && srv.response.success) return true;
      ros::Duration(0.5).sleep();
    }
    return false;
  }

  /**
   * Update robot status via service with retry logic
   */
  bool callSetStatus(bool enable, const std::string &message, int retry_count = 3)
  {
    amr_msg::SetStatus srv;
    srv.request.enable = enable;
    for (int attempt = 1; attempt <= retry_count; ++attempt)
    {
      if (set_status_client_.call(srv) && srv.response.success) return true;
      ros::Duration(0.5).sleep();
    }
    return false;
  }
  
  /**
   * Update robot mission log via service with retry logic
   */
  bool callSetMission(const std::string& status, const std::string& source, const std::string& target, const std::string& action)
  {
      amr_msg::SetMission srv;
      srv.request.status = status;
      srv.request.source_station = source;
      srv.request.target_station = target;
      srv.request.action_type = action;
  
      if (set_mission_client_.call(srv))
      {
          ROS_INFO("[amr_go_move] SetMission Success: %s", srv.response.message.c_str());
          return srv.response.success;
      }
      else
      {
          ROS_ERROR("[amr_go_move] Failed to call /set_mission service");
          return false;
      }
  }

  /**
   * Callback for the internal service server to trigger unloading sequence
   */
  bool amrMcuServiceCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
  {
    if (!req.data)
    {
      res.success = true;
      res.message = "No action.";
      return true;
    }
    callSetStatus(true, "LOGGER_ON", 1);
    callGoAruco(true);
    callSetMission("RUNNING", "HOME", "UNLOADING", "MOVE");
    callAmrMcu(0, "PLACE", 1);
    std_msgs::String state_msg;
    state_msg.data = "UNLOADING";
    mission_pub_.publish(state_msg);
    res.success = true;
    res.message = "UNLOADING sequence completed.";
    return true;
  }

  /**
   * Core navigation function: Sends a 2D pose goal to move_base
   */
  bool sendGoal(double x, double y, double yaw_deg, const std::string &target_name, const std::string &mission_state = std::string())
  {
    if (stop_requested_) return false;
    const double yaw_rad = yaw_deg * M_PI / 180.0;
    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();
    goal.target_pose.pose.orientation.w = q.w();

    ROS_INFO("[amr_go_move] Sending %s goal...", target_name.c_str());
    
    ac_.sendGoal(goal);
    
    ros::Time start_wait = ros::Time::now();
    while (ros::ok()) {
        if (stop_requested_) {
            ac_.cancelGoal();
            ROS_WARN("[amr_go_move] Goal cancelled during wait.");
            return false;
        }
        
        if (ac_.waitForResult(ros::Duration(0.1))) break;

        if ((ros::Time::now() - start_wait).toSec() > 60.0) {
            ROS_ERROR("[amr_go_move] Goal timeout!");
            return false;
        }
    }

    if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      if (!mission_state.empty())
      {
        std_msgs::String msg;
        msg.data = mission_state;
        mission_pub_.publish(msg);
      }
      return true;
    }
    return false;
  }

  /**
   * Helper to send navigation goals based on object name
   */
  bool sendObjectGoalByName(const std::string &name)
  {
    if (name == "ESP32") return sendGoal(esp32_x_, esp32_y_, esp32_yaw_deg_, "");
    if (name == "L298N") return sendGoal(motordriver_x_, motordriver_y_, motordriver_yaw_deg_, "");
    if (name == "MB102") return sendGoal(powersupply_x_, powersupply_y_, powersupply_yaw_deg_, "");
    return false;
  }
  
  bool sendGoHomeGoal()
  {
    return sendGoal(go_home_x_, go_home_y_, go_home_yaw_deg_, "go_home", "HOME");
  }
  
  bool sendGostandbyGoal()
  {
    return sendGoal(go_standby_x_, go_standby_y_, go_standby_yaw_deg_, "go_standby", "");
  }
  
  /**
   * Complex workflow for picking up items: Back up -> Standby -> Align -> Approach
   */
  void handlePickUpZoneCommand()
  {
    if (last_move_command_ != "pick_up_zone") {
      ROS_INFO("[amr_go_move] last_move_command_ %s", last_move_command_.c_str());
      ROS_INFO("[amr_go_move] Starting Pick-up Zone sequence.");
      
      // Step 1: Move back to clear the rack
      ROS_INFO("[amr_go_move] [Step 1/4] Moving backward manually ");
      moveManual(-0.10, 0, 6.5);
      moveManual(0, -0.16, 4.7);
      // Step 2: Navigate to intermediate standby point
      ROS_INFO("[amr_go_move] [Step 2/4] Navigating to standby position...");
      if (!sendGostandbyGoal())
      {
        ROS_ERROR("[amr_go_move] Failed to reach standby position. Aborting mission.");
      }
  
      // Step 3: Align using ArUco and rotate
      ROS_INFO("[amr_go_move] [Step 3/4] Performing ArUco alignment and rotation at standby...");
      if (!callGoAruco(true))
      {
        ROS_WARN("[amr_go_move] ArUco alignment at standby failed. Aborting mission.");
      }
      
      // Face the pick up zone
      sendGoal(3.153, -0.266, 180, "rotate", "");
  
      // Step 4: Final docking approach
      ROS_INFO("[amr_go_move] [Step 4/4] Finalizing approach to pick_up_zone via ArUco...");
      if (!callGoAruco(true))
      {
        ROS_ERROR("[amr_go_move] Final ArUco approach to pick_up_zone failed.");
      }
  
      callAmrMcu(1, "PICK", 1);
    } else 
    {
        ROS_INFO("[amr_go_move] Repeated pick_up_zone - Only publishing PICK");
    }
    
    //callSetMission("RUNNING", "LOADING", "LOADING", "LOADING");
    std_msgs::String msg;
    msg.data = "PICK";
    mission_pub_.publish(msg);
    
    ROS_INFO("[amr_go_move] Pick-up Zone sequence successfully completed.");
  }

  /**
   * Workflow to return to home position and reset states
   */
  void handleGoHomeCommand()
  {
    if (sendGoHomeGoal())
    {
      callSetStatus(false, "LOGGER_OFF", 1);
      callAmrMcu(0, "RESET", 1);
      callSetMission("DONE", "", "", "");
    }
  }

  /**
   * Sequentially visit positions for a list of objects
   */
  void handlePositionsCommand(const std::vector<std::string> &object_list)
  {
    if (object_list.empty()) return;
    stop_requested_ = false;
    
    callAmrMcu(0, "RESET", 1);
    callSetMission("RUNNING", "PICK_UP_ZONE", object_list[0], "MOVE");

    bool all_succeeded = true;
    for (size_t i = 0; i < object_list.size(); ++i)
    {
      if (stop_requested_) {
          ROS_WARN("[amr_go_move] Stop requested. Aborting position sequence.");
          all_succeeded = false;
          break;
      }
      const std::string& current_target = object_list[i];
      if (!sendObjectGoalByName(current_target)) 
      {
        all_succeeded = false;
        if (stop_requested_) break;
      }
      else 
      {
        callAmrMcu(2, "PLACE", 1);
        if (i + 1 < object_list.size()) 
        {
          callSetMission("RUNNING", current_target, object_list[i + 1], "MOVE");
        }
      }
    }
    if (all_succeeded)
    {
      callSetMission("RUNNING", object_list.back(), "HOME", "MOVE");
      std_msgs::String msg;
      msg.data = "DONE";
      mission_pub_.publish(msg);
    }
  }

  /**
   * Callback for main movement commands (from OPC UA)
   */
  void opcuaMoveCallback(const std_msgs::String::ConstPtr &msg)
  {
    const std::string &data = msg->data;
    if (data == "Ready") return;
    
    if (data.find("stop") != std::string::npos)
    {
      ROS_WARN("[amr_go_move] STOP command received!");
      stop_requested_ = true;
      ac_.cancelAllGoals();
      stopRobot();
      
      callSetStatus(false, "STOPPED", 1);
      std_msgs::String st;
      st.data = "STOPPED";
      mission_pub_.publish(st);
      return;
    }
    std::string cmd = parseMoveCommand(data);
    ROS_INFO("Parsed CMD: [%s]", cmd.c_str());
    stop_requested_ = false;    
    
    if (cmd == "pick_up_zone" || data.find("pick_up_zone") != std::string::npos) 
        handlePickUpZoneCommand();
    else if (cmd == "go_home" || data.find("go_home") != std::string::npos) 
        handleGoHomeCommand();
    else if (data.find("unloading") != std::string::npos)
    {
      callAmrMcu(2, "PLACE", 1);
      callSetMission("RUNNING", "UNLOADING", "", "UNLOADING");
//      std_msgs::String st;
//      st.data = "UNLOADING";
//      mission_pub_.publish(st);
    }
    else if (data.find("stop") != std::string::npos)
    {
      //all action cancel
    }
    last_move_command_ = cmd;
  }

  /**
   * Callback for object delivery commands (from OPC UA)
   */
  void opcuaPositionsCallback(const std_msgs::String::ConstPtr &msg)
  {
    if (msg->data == "Ready") return;
    std::vector<std::string> object_list = parseObjectInfoList(msg->data);
    if (!object_list.empty()) handlePositionsCommand(object_list);
  }

  // ROS infrastructure
  ros::NodeHandle nh_;
  ros::Subscriber sub_move_, sub_positions_;
  ros::Publisher mission_pub_, cmd_vel_pub_;
  MoveBaseClient ac_;
  tf::TransformListener tf_listener_;
  
  // State tracking
  std::string last_move_command_, last_positions_command_;
  
  // Coordinate storage
  double pick_up_zone_x_, pick_up_zone_y_, pick_up_zone_yaw_deg_;
  double go_standby_x_, go_standby_y_, go_standby_yaw_deg_;
  double esp32_x_, esp32_y_, esp32_yaw_deg_;
  double motordriver_x_, motordriver_y_, motordriver_yaw_deg_;
  double powersupply_x_, powersupply_y_, powersupply_yaw_deg_;
  double go_home_x_, go_home_y_, go_home_yaw_deg_;
  
  // Service communication
  ros::ServiceClient go_aruco_client_, amr_mcu_client_, set_status_client_, set_mission_client_;
  ros::ServiceServer amr_mcu_service_server_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "amr_go_move");
  AmrGoMove node;
  ros::spin();
  return 0;
}