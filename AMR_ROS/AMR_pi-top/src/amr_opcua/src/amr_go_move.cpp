#include <ros/ros.h>
#include <std_msgs/String.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/tf.h>
#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <std_srvs/SetBool.h>
#include <amr_msg/SetMcuValue.h>
#include <amr_msg/SetStatus.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class AmrGoMove
{
public:
  AmrGoMove()
    : ac_("move_base", true),
      last_move_command_(""),
      last_positions_command_("")
  {
    ros::NodeHandle private_nh("~");

    // Parameters for go_move (logical destinations)
    private_nh.param("pick_up_zone_x",       pick_up_zone_x_,       1.0);
    private_nh.param("pick_up_zone_y",       pick_up_zone_y_,       0.0);
    private_nh.param("pick_up_zone_yaw_deg", pick_up_zone_yaw_deg_, 0.0);

    private_nh.param("go_home_x",            go_home_x_,       1.0);
    private_nh.param("go_home_y",            go_home_y_,       0.0);
    private_nh.param("go_home_yaw_deg",      go_home_yaw_deg_, 0.0);

    // Parameters for go_positions (object-related positions)
    private_nh.param("esp32_x",              esp32_x_,       2.0);
    private_nh.param("esp32_y",              esp32_y_,       1.0);
    private_nh.param("esp32_yaw_deg",        esp32_yaw_deg_, 0.0);

    private_nh.param("motordriver_x",        motordriver_x_,       2.0);
    private_nh.param("motordriver_y",        motordriver_y_,       1.0);
    private_nh.param("motordriver_yaw_deg",  motordriver_yaw_deg_, 0.0);

    private_nh.param("powersuplpy_x",        powersuplpy_x_,       2.0);
    private_nh.param("powersuplpy_y",        powersuplpy_y_,       1.0);
    private_nh.param("powersuplpy_yaw_deg",  powersuplpy_yaw_deg_, 0.0);

    // Subscribe OPC UA command topics
    sub_move_ = nh_.subscribe("/amr/opcua/go_move", 10,
                              &AmrGoMove::opcuaMoveCallback, this);

    sub_positions_ = nh_.subscribe("/amr/opcua/go_positions", 10,
                                   &AmrGoMove::opcuaPositionsCallback, this);

    // Publisher for mission state (write_opcua_node.py will subscribe this)
    mission_pub_ = nh_.advertise<std_msgs::String>("amr_mission_state", 10);

    // Service clients
    go_aruco_client_  = nh_.serviceClient<std_srvs::SetBool>("/go_aruco");
    amr_mcu_client_   = nh_.serviceClient<amr_msg::SetMcuValue>("/amr_mcu");
    set_status_client_ = nh_.serviceClient<amr_msg::SetStatus>("/set_status");

    ROS_INFO("Waiting for move_base action server...");
    ac_.waitForServer();
    ROS_INFO("Connected to move_base action server.");
  }

private:
  // Trim spaces and tabs from both ends of a string.
  static std::string trim(const std::string& s)
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

  // Parse move_command from simple JSON string.
  // Expected:
  // { "move_command" : "go_home" }
  // or
  // { "move_command" : "pick_up_zone" }
  std::string parseMoveCommand(const std::string& json_str)
  {
    const std::string key = "\"move_command\"";
    std::size_t key_pos = json_str.find(key);
    if (key_pos == std::string::npos)
    {
      return std::string();
    }

    std::size_t colon_pos = json_str.find(':', key_pos + key.size());
    if (colon_pos == std::string::npos)
    {
      return std::string();
    }

    std::size_t first_quote = json_str.find('"', colon_pos);
    if (first_quote == std::string::npos)
    {
      return std::string();
    }
    std::size_t second_quote = json_str.find('"', first_quote + 1);
    if (second_quote == std::string::npos)
    {
      return std::string();
    }

    std::string value = json_str.substr(first_quote + 1,
                                        second_quote - first_quote - 1);
    return trim(value);
  }

  // Parse object_info list from simple JSON string.
  // Example:
  // { "object_info" : ["ESP32","MB102","L298N"] }
  std::vector<std::string> parseObjectInfoList(const std::string& json_str)
  {
    std::vector<std::string> result;

    std::size_t start = json_str.find('[');
    std::size_t end   = std::string::npos;
    if (start != std::string::npos)
    {
      end = json_str.find(']', start);
    }

    if (start == std::string::npos ||
        end == std::string::npos ||
        end <= start + 1)
    {
      ROS_WARN("[amr_go_move] Failed to parse object_info list (no [..] found).");
      return result;
    }

    std::string inside = json_str.substr(start + 1, end - start - 1);
    std::stringstream ss(inside);
    std::string item;

    while (std::getline(ss, item, ','))
    {
      std::string token = trim(item);

      if (!token.empty() &&
          token.front() == '"' &&
          token.back() == '"' &&
          token.size() >= 2)
      {
        token = token.substr(1, token.size() - 2);
      }

      token = trim(token);
      if (!token.empty())
      {
        result.push_back(token);
      }
    }

    return result;
  }

  // Common helper function to send a navigation goal to move_base.
  void sendGoal(double x, double y, double yaw_deg,
                const std::string& target_name,
                const std::string& mission_state)
  {
    double yaw_rad = yaw_deg * M_PI / 180.0;

    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();
    goal.target_pose.pose.orientation.w = q.w();

    ROS_INFO("[amr_go_move] Sending %s goal: x=%.3f, y=%.3f, yaw=%.3f deg",
             target_name.c_str(), x, y, yaw_deg);

    ac_.sendGoal(goal);

    bool finished_before_timeout = ac_.waitForResult(ros::Duration(60.0));

    if (finished_before_timeout &&
        ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("[amr_go_move] Robot arrived at %s target.", target_name.c_str());

      if (!mission_state.empty())
      {
        std_msgs::String msg;
        msg.data = mission_state;
        mission_pub_.publish(msg);
        ROS_INFO("[amr_go_move] Published amr_mission_state = '%s'",
                 mission_state.c_str());
      }
    }
    else
    {
      ROS_WARN("[amr_go_move] Failed to reach %s target. State: %s",
               target_name.c_str(),
               ac_.getState().toString().c_str());
    }
  }

  // Expects:
  // { "object_info" : ["ESP32","MB102","L298N"] }
  bool sendObjectGoalByName(const std::string& name)
  {
    if (name == "ESP32")
    {
      ROS_INFO("[amr_go_move] Detected object_info: ESP32");
      sendGoal(esp32_x_, esp32_y_, esp32_yaw_deg_, "ESP32", "");
      return true;
    }
    else if (name == "L298N")
    {
      ROS_INFO("[amr_go_move] Detected object_info: L298N");
      sendGoal(motordriver_x_, motordriver_y_, motordriver_yaw_deg_, "L298N", "");
      return true;
    }
    else if (name == "MB102")
    {
      ROS_INFO("[amr_go_move] Detected object_info: MB102");
      sendGoal(powersuplpy_x_, powersuplpy_y_, powersuplpy_yaw_deg_, "MB102", "");
      return true;
    }
    else
    {
      ROS_WARN("[amr_go_move] Unknown object_info: %s", name.c_str());
      return false;
    }
  }

  bool callAmrMcu(int value, const std::string& tag, int retries = 1)
  {
    if (value != 0 && value != 1 && value != 2)
    {
      ROS_WARN("[amr_go_move] callAmrMcu invalid value=%d (tag=%s)", value, tag.c_str());
      return false;
    }

    amr_msg::SetMcuValue mcu_srv;
    mcu_srv.request.value = value;

    for (int attempt = 0; attempt <= retries; ++attempt)
    {
      if (amr_mcu_client_.call(mcu_srv))
      {
        if (mcu_srv.response.success)
        {
          ROS_INFO("[amr_go_move] /amr_mcu OK (tag=%s, value=%d): %s",
                   tag.c_str(), value, mcu_srv.response.message.c_str());
          return true;
        }
        else
        {
          ROS_WARN("[amr_go_move] /amr_mcu FAIL response (tag=%s, value=%d): %s",
                   tag.c_str(), value, mcu_srv.response.message.c_str());
          return false;
        }
      }

      ROS_ERROR("[amr_go_move] /amr_mcu call failed (tag=%s, value=%d, attempt=%d/%d)",
                tag.c_str(), value, attempt + 1, retries + 1);

      ros::Duration(0.1).sleep();
    }
    return false;
  }

  bool callSetStatus(bool enable, const std::string& tag, int retries = 1)
  {
    amr_msg::SetStatus status_srv;
    status_srv.request.enable = enable;

    for (int attempt = 0; attempt <= retries; ++attempt)
    {
      if (set_status_client_.call(status_srv))
      {
        if (status_srv.response.success)
        {
          ROS_INFO("[amr_go_move] /set_status OK (tag=%s, enable=%s): %s",
                   tag.c_str(),
                   enable ? "true" : "false",
                   status_srv.response.message.c_str());
          return true;
        }
        else
        {
          ROS_WARN("[amr_go_move] /set_status FAIL response (tag=%s, enable=%s): %s",
                   tag.c_str(),
                   enable ? "true" : "false",
                   status_srv.response.message.c_str());
          return false;
        }
      }

      ROS_ERROR("[amr_go_move] /set_status call failed (tag=%s, enable=%s, attempt=%d/%d)",
                tag.c_str(),
                enable ? "true" : "false",
                attempt + 1, retries + 1);

      ros::Duration(0.1).sleep();
    }
    return false;
  }

  // Callback for /amr/opcua/go_move
  void opcuaMoveCallback(const std_msgs::String::ConstPtr& msg)
  {
    const std::string& data = msg->data;
    ROS_INFO_STREAM("[amr_go_move] Received /amr/opcua/go_move: " << data);

    if (data == "Ready")
    {
      ROS_INFO("[amr_go_move] State: Ready. No navigation command.");
      return;
    }

    if (data == last_move_command_)
    {
      ROS_INFO("[amr_go_move] Same move command as last processed. Ignoring.");
      return;
    }

    std::string cmd = parseMoveCommand(data);
    std::string effective_cmd;

    if (!cmd.empty())
    {
      effective_cmd = cmd;
    }
    else
    {
      if (data.find("pick_up_zone") != std::string::npos)
      {
        effective_cmd = "pick_up_zone";
      }
      else if (data.find("go_home") != std::string::npos)
      {
        effective_cmd = "go_home";
      }
    }

    if (effective_cmd.empty())
    {
      ROS_WARN("[amr_go_move] Unknown move_command in payload. No action.");
      return;
    }

    last_move_command_ = data;

    if (effective_cmd == "pick_up_zone")
    {
      ROS_INFO("[amr_go_move] Detected move_command: pick_up_zone");

      std_srvs::SetBool srv;
      srv.request.data = true;

      if (go_aruco_client_.call(srv))
      {
        if (srv.response.success)
        {
          ROS_INFO("[amr_go_move] /go_aruco service call succeeded: %s",
                   srv.response.message.c_str());

          const bool mcu_ok = callAmrMcu(1, "PICK", 1);
          if (!mcu_ok)
          {
            ROS_WARN("[amr_go_move] MCU signal failed, but continuing mission publish.");
          }

          const bool status_ok = callSetStatus(true, "LOGGER_ON", 1);
          if (!status_ok)
          {
            ROS_WARN("[amr_go_move] /set_status failed to enable logger.");
          }

          std_msgs::String state_msg;
          state_msg.data = "PICK";
          mission_pub_.publish(state_msg);
          ROS_INFO("[amr_go_move] Published amr_mission_state = 'PICK'");
        }
        else
        {
          ROS_WARN("[amr_go_move] /go_aruco responded but reported failure: %s",
                   srv.response.message.c_str());
        }
      }
      else
      {
        ROS_ERROR("[amr_go_move] Failed to call /go_aruco service.");
      }
    }
    else if (effective_cmd == "go_home")
    {
      ROS_INFO("[amr_go_move] Detected move_command: go_home");
      sendGoHomeGoal();
    
      // NEW: disable DB logger via /set_status after going home
      const bool status_ok = callSetStatus(false, "LOGGER_OFF", 1);
      if (!status_ok)
      {
        ROS_WARN("[amr_go_move] /set_status failed to disable logger after go_home.");
      }
    
      std_srvs::SetBool srv;
      srv.request.data = true;
    
      if (go_aruco_client_.call(srv))
      {
        if (srv.response.success)
        {
          ROS_INFO("[amr_go_move] /go_aruco service call succeeded: %s",
                   srv.response.message.c_str());
    
          const bool mcu_ok = callAmrMcu(0, "RESET", 1);
          if (!mcu_ok)
          {
            ROS_WARN("[amr_go_move] MCU signal failed, but continuing.");
          }
        }
        else
        {
          ROS_WARN("[amr_go_move] /go_aruco responded but reported failure: %s",
                   srv.response.message.c_str());
        }
      }
      else
      {
        ROS_ERROR("[amr_go_move] Failed to call /go_aruco service.");
      }
    }
    else
    {
      ROS_WARN("[amr_go_move] move_command '%s' is not supported.", effective_cmd.c_str());
    }
  }

  // Callback for /amr/opcua/go_positions
  void opcuaPositionsCallback(const std_msgs::String::ConstPtr& msg)
  {
    const std::string& data = msg->data;
    ROS_INFO_STREAM("[amr_go_move] Received /amr/opcua/go_positions: " << data);

    if (data == "Ready")
    {
      ROS_INFO("[amr_go_move] State: Ready. No navigation command.");
      return;
    }

    if (data == last_positions_command_)
    {
      ROS_INFO("[amr_go_move] Same positions command as last processed. Ignoring.");
      return;
    }

    std::vector<std::string> object_list = parseObjectInfoList(data);
    if (object_list.empty())
    {
      ROS_WARN("[amr_go_move] object_info list is empty or could not be parsed.");
      return;
    }

    last_positions_command_ = data;

    ROS_INFO("[amr_go_move] Parsed %zu object(s) from object_info list.", object_list.size());

    const bool mcu_ok0 = callAmrMcu(0, "RESET", 1);
    if (!mcu_ok0)
    {
      ROS_WARN("[amr_go_move] MCU signal failed, but continuing.");
    }

    for (std::size_t i = 0; i < object_list.size(); ++i)
    {
      const std::string& name = object_list[i];
      ROS_INFO("[amr_go_move] Processing object_info[%zu] = %s", i, name.c_str());
      const bool goal_ok = sendObjectGoalByName(name);
      if (goal_ok)
      {
        const bool mcu_ok2 = callAmrMcu(2, "PLACE", 1);
        if (!mcu_ok2)
        {
          ROS_WARN("[amr_go_move] MCU signal failed after PLACE, but continuing.");
        }
      }
    }

    std_msgs::String state_msg;
    state_msg.data = "DONE";
    mission_pub_.publish(state_msg);
    ROS_INFO("[amr_go_move] Published amr_mission_state = 'DONE'");
  }

  void sendPickUpZoneGoal()
  {
    sendGoal(pick_up_zone_x_,
             pick_up_zone_y_,
             pick_up_zone_yaw_deg_,
             "pick_up_zone",
             "PICK");
  }

  void sendGoHomeGoal()
  {
    sendGoal(go_home_x_,
             go_home_y_,
             go_home_yaw_deg_,
             "go_home",
             "HOME");
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_move_;
  ros::Subscriber sub_positions_;
  MoveBaseClient ac_;

  ros::Publisher mission_pub_;

  std::string last_move_command_;
  std::string last_positions_command_;

  double pick_up_zone_x_;
  double pick_up_zone_y_;
  double pick_up_zone_yaw_deg_;

  double go_home_x_;
  double go_home_y_;
  double go_home_yaw_deg_;

  double esp32_x_;
  double esp32_y_;
  double esp32_yaw_deg_;

  double motordriver_x_;
  double motordriver_y_;
  double motordriver_yaw_deg_;

  double powersuplpy_x_;
  double powersuplpy_y_;
  double powersuplpy_yaw_deg_;

  ros::ServiceClient go_aruco_client_;
  ros::ServiceClient amr_mcu_client_;
  ros::ServiceClient set_status_client_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "amr_go_move");
  AmrGoMove node;
  ros::spin();
  return 0;
}
