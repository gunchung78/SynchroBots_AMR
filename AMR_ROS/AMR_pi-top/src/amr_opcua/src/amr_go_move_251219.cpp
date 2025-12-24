#include <ros/ros.h>
#include <std_msgs/String.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <std_srvs/SetBool.h>
#include <amr_msg/SetMcuValue.h>
#include <amr_msg/SetStatus.h>

#include <string>
#include <vector>
#include <sstream>
#include <cmath>
#include <amr_msg/SetMission.h>


using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

class AmrGoMove
{
public:
  AmrGoMove()
    : ac_("move_base", true),
      last_move_command_(""),
      last_positions_command_("")
  {
    ros::NodeHandle private_nh("~");

    // Logical destinations (go_move)
    private_nh.param("pick_up_zone_x",       pick_up_zone_x_,       1.0);
    private_nh.param("pick_up_zone_y",       pick_up_zone_y_,       0.0);
    private_nh.param("pick_up_zone_yaw_deg", pick_up_zone_yaw_deg_, 0.0);

    private_nh.param("go_home_x",            go_home_x_,       1.0);
    private_nh.param("go_home_y",            go_home_y_,       0.0);
    private_nh.param("go_home_yaw_deg",      go_home_yaw_deg_, 0.0);

    // Object destinations (go_positions)
    private_nh.param("esp32_x",              esp32_x_,       2.0);
    private_nh.param("esp32_y",              esp32_y_,       1.0);
    private_nh.param("esp32_yaw_deg",        esp32_yaw_deg_, 0.0);

    private_nh.param("motordriver_x",        motordriver_x_,       2.0);
    private_nh.param("motordriver_y",        motordriver_y_,       1.0);
    private_nh.param("motordriver_yaw_deg",  motordriver_yaw_deg_, 0.0);

    private_nh.param("powersuplpy_x",        powersuplpy_x_,       2.0);
    private_nh.param("powersuplpy_y",        powersuplpy_y_,       1.0);
    private_nh.param("powersuplpy_yaw_deg",  powersuplpy_yaw_deg_, 0.0);

    // Subscribers (OPC UA commands)
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

    // Mission state publisher (consumed by write_opcua_node.py)
    mission_pub_ = nh_.advertise<std_msgs::String>("amr_mission_state", 10);

    // Service clients
    go_aruco_client_   = nh_.serviceClient<std_srvs::SetBool>("/go_aruco");
    amr_mcu_client_    = nh_.serviceClient<amr_msg::SetMcuValue>("/amr_mcu");
    set_status_client_ = nh_.serviceClient<amr_msg::SetStatus>("/set_status");
    set_mission_client_ = nh_.serviceClient<amr_msg::SetMission>("/set_mission");

    ROS_INFO("[amr_go_move] Waiting for move_base action server...");
    ac_.waitForServer();
    ROS_INFO("[amr_go_move] Connected to move_base action server.");
  }

private:
  // --------------------------------------------------------------------------
  // String helpers
  // --------------------------------------------------------------------------

  static std::string trim(const std::string& s)
  {
    std::size_t start = 0;
    while (start < s.size() &&
           (s[start] == ' '  || s[start] == '\t' ||
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
           (s[end - 1] == ' '  || s[end - 1] == '\t' ||
            s[end - 1] == '\n' || s[end - 1] == '\r'))
    {
      --end;
    }

    return s.substr(start, end - start);
  }

  // Extract "move_command" value from a simple JSON-like string.
  // Expected examples:
  //   { "move_command" : "go_home" }
  //   { "move_command" : "pick_up_zone" }
  std::string parseMoveCommand(const std::string& json_str) const
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

  // Extract ["ESP32","MB102","L298N"] style list from JSON-like payload:
  //   { "object_info" : ["ESP32","MB102","L298N"] }
  std::vector<std::string> parseObjectInfoList(const std::string& json_str) const
  {
    std::vector<std::string> result;

    std::size_t start = json_str.find('[');
    std::size_t end   = std::string::npos;

    if (start != std::string::npos)
    {
      end = json_str.find(']', start);
    }

    if (start == std::string::npos ||
        end   == std::string::npos ||
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

      // Remove double quotes if present
      if (!token.empty() &&
          token.front() == '"' &&
          token.back()   == '"' &&
          token.size()  >= 2)
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

  // --------------------------------------------------------------------------
  // Service helpers
  // --------------------------------------------------------------------------

  bool callGoAruco(bool enable)
  {
    std_srvs::SetBool srv;
    srv.request.data = enable;

    if (!go_aruco_client_.call(srv))
    {
      ROS_ERROR("[amr_go_move] Failed to call /go_aruco service.");
      return false;
    }

    if (!srv.response.success)
    {
      ROS_WARN("[amr_go_move] /go_aruco returned failure: %s",
               srv.response.message.c_str());
      return false;
    }

    ROS_INFO("[amr_go_move] /go_aruco succeeded: %s",
             srv.response.message.c_str());
    return true;
  }

  // Call /amr_mcu (SetMcuValue) with retry.
  bool callAmrMcu(int value,
                  const std::string& tag,
                  int retry_count = 3)
  {
    amr_msg::SetMcuValue srv;
    srv.request.value = value;
    srv.request.tag   = tag;

    for (int attempt = 1; attempt <= retry_count; ++attempt)
    {
      if (amr_mcu_client_.call(srv))
      {
        if (srv.response.success)
        {
          ROS_INFO("[amr_go_move] /amr_mcu success (attempt %d/%d): %s",
                   attempt, retry_count, srv.response.message.c_str());
          return true;
        }
        else
        {
          ROS_WARN("[amr_go_move] /amr_mcu failure (attempt %d/%d): %s",
                   attempt, retry_count, srv.response.message.c_str());
        }
      }
      else
      {
        ROS_ERROR("[amr_go_move] /amr_mcu call failed (attempt %d/%d)",
                  attempt, retry_count);
      }

      ros::Duration(0.5).sleep();
    }

    ROS_ERROR("[amr_go_move] /amr_mcu failed after %d attempts.", retry_count);
    return false;
  }

  // Call /set_status (SetStatus) with retry.
  bool callSetStatus(bool enable,
                     const std::string& message,
                     int retry_count = 3)
  {
    amr_msg::SetStatus srv;
    srv.request.enable = enable;

    for (int attempt = 1; attempt <= retry_count; ++attempt)
    {
      if (set_status_client_.call(srv))
      {
        if (srv.response.success)
        {
          ROS_INFO("[amr_go_move] /set_status success (%s): %s",
                   message.c_str(),
                   srv.response.message.c_str());
          return true;
        }
        else
        {
          ROS_WARN("[amr_go_move] /set_status failure (%s): %s",
                   message.c_str(),
                   srv.response.message.c_str());
        }
      }
      else
      {
        ROS_ERROR("[amr_go_move] /set_status call failed (%s), attempt %d/%d",
                  message.c_str(), attempt, retry_count);
      }

      ros::Duration(0.5).sleep();
    }

    ROS_ERROR("[amr_go_move] /set_status failed after %d attempts (%s).",
              retry_count, message.c_str());
    return false;
  }
  
  // Call /set_mission (SetStatus) with retry.
  bool callSetMission(const std::string& status,
                      const std::string& source_station,
                      const std::string& target_station,
                      const std::string& action_type,
                      int retries = 1)
  {
    amr_msg::SetMission srv;
    srv.request.status         = status;
    srv.request.source_station = source_station;
    srv.request.target_station = target_station;
    srv.request.action_type    = action_type;

    for (int attempt = 0; attempt <= retries; ++attempt)
    {
      if (set_mission_client_.call(srv))
      {
        if (srv.response.success)
        {
          ROS_INFO("[amr_go_move] /set_mission success: "
                   "status=%s, src=%s, dst=%s, action=%s, msg=%s",
                   status.c_str(),
                   source_station.c_str(),
                   target_station.c_str(),
                   action_type.c_str(),
                   srv.response.message.c_str());
          return true;
        }
        else
        {
          ROS_WARN("[amr_go_move] /set_mission returned failure: "
                   "status=%s, msg=%s",
                   status.c_str(),
                   srv.response.message.c_str());
          return false;
        }
      }

      ROS_ERROR("[amr_go_move] /set_mission call failed "
                "(attempt %d/%d)",
                attempt + 1, retries + 1);
      ros::Duration(0.1).sleep();
    }

    return false;
  }

  // --------------------------------------------------------------------------
  // move_base helpers
  // --------------------------------------------------------------------------

  // Send a generic navigation goal to move_base.
  // Returns true when SUCCEEDED state is reported.
  bool sendGoal(double x,
                double y,
                double yaw_deg,
                const std::string& target_name,
                const std::string& mission_state = std::string())
  {
    const double yaw_rad = yaw_deg * M_PI / 180.0;

    tf::Quaternion q;
    q.setRPY(0.0, 0.0, yaw_rad);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp    = ros::Time::now();

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
    if (!finished_before_timeout)
    {
      ROS_WARN("[amr_go_move] Timeout while waiting for %s goal.", target_name.c_str());
      return false;
    }

    actionlib::SimpleClientGoalState state = ac_.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("[amr_go_move] Reached %s target.", target_name.c_str());

      if (!mission_state.empty())
      {
        std_msgs::String msg;
        msg.data = mission_state;
        mission_pub_.publish(msg);
        ROS_INFO("[amr_go_move] Published amr_mission_state = '%s'",
                 mission_state.c_str());
      }

      return true;
    }

    ROS_WARN("[amr_go_move] Failed to reach %s target. State: %s",
             target_name.c_str(), state.toString().c_str());
    return false;
  }

  // Rotate robot around its current position by delta yaw (degrees).
  bool rotateRelativeYaw(double delta_yaw_deg)
  {
    tf::StampedTransform transform;

    try
    {
      tf_listener_.lookupTransform(
        "map",
        "base_footprint",
        ros::Time(0),
        transform);
    }
    catch (tf::TransformException& ex)
    {
      ROS_WARN("[amr_go_move] rotateRelativeYaw: TF exception: %s", ex.what());
      return false;
    }

    double current_yaw = tf::getYaw(transform.getRotation());
    double new_yaw     = current_yaw + (delta_yaw_deg * M_PI / 180.0);

    double x = transform.getOrigin().x();
    double y = transform.getOrigin().y();

    tf::Quaternion q;
    q.setRPY(0.0, 0.0, new_yaw);

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp    = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = 0.0;

    goal.target_pose.pose.orientation.x = q.x();
    goal.target_pose.pose.orientation.y = q.y();
    goal.target_pose.pose.orientation.z = q.z();
    goal.target_pose.pose.orientation.w = q.w();

    ROS_INFO("[amr_go_move] Sending rotateRelativeYaw goal: delta=%.3f deg",
             delta_yaw_deg);

    ac_.sendGoal(goal);
    bool finished_before_timeout = ac_.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout &&
        ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("[amr_go_move] Rotation goal succeeded.");
      return true;
    }

    ROS_WARN("[amr_go_move] Rotation goal failed. State: %s",
             ac_.getState().toString().c_str());
    return false;
  }

  // Send goal by object name (for go_positions).
  // Payload uses "ESP32", "MB102", "L298N".
  bool sendObjectGoalByName(const std::string& name)
  {
    if (name == "ESP32")
    {
      ROS_INFO("[amr_go_move] Object target: ESP32");
      return sendGoal(esp32_x_, esp32_y_, esp32_yaw_deg_, "ESP32");
    }
    else if (name == "L298N")
    {
      ROS_INFO("[amr_go_move] Object target: L298N");
      return sendGoal(motordriver_x_, motordriver_y_, motordriver_yaw_deg_, "L298N");
    }
    else if (name == "MB102")
    {
      ROS_INFO("[amr_go_move] Object target: MB102");
      return sendGoal(powersuplpy_x_, powersuplpy_y_, powersuplpy_yaw_deg_, "MB102");
    }

    ROS_WARN("[amr_go_move] Unknown object_info name: %s", name.c_str());
    return false;
  }

  bool sendPickUpZoneGoal()
  {
    return sendGoal(pick_up_zone_x_,
                    pick_up_zone_y_,
                    pick_up_zone_yaw_deg_,
                    "pick_up_zone",
                    "PICK");
  }

  bool sendGoHomeGoal()
  {
    return sendGoal(go_home_x_,
                    go_home_y_,
                    go_home_yaw_deg_,
                    "go_home",
                    "HOME");
  }

  // --------------------------------------------------------------------------
  // Command handlers
  // --------------------------------------------------------------------------

  void handlePickUpZoneCommand()
  {
    ROS_INFO("[amr_go_move] Handling move_command: pick_up_zone");

    // Use ArUco-based precise approach instead of static goal.
    if (!callGoAruco(true))
    {
      ROS_WARN("[amr_go_move] /go_aruco failed, skip PICK handling.");
      return;
    }

    // MCU: PICK state
    callAmrMcu(1, "PICK", 1);

    // Enable DB logger
    callSetStatus(true, "LOGGER_ON", 1);

    // Publish mission state "PICK"
    std_msgs::String msg;
    msg.data = "PICK";
    mission_pub_.publish(msg);
    ROS_INFO("[amr_go_move] Published amr_mission_state = 'PICK'");
  }

  void handleGoHomeCommand()
  {
    ROS_INFO("[amr_go_move] Handling move_command: go_home");

    // Navigate to home position
    bool ok = sendGoHomeGoal();
    if (!ok)
    {
      ROS_WARN("[amr_go_move] go_home navigation failed.");
    }

    // Disable DB logger (even if navigation failed)
    callSetStatus(false, "LOGGER_OFF", 1);

    // Optional post-home ArUco alignment + MCU reset + relative rotation
    if (callGoAruco(true))
    {
      callAmrMcu(0, "RESET", 1);
      // Rotate 90 degrees at current position
      rotateRelativeYaw(90.0);
    }
  }

  void handlePositionsCommand(const std::vector<std::string>& object_list)
  {
    ROS_INFO("[amr_go_move] Handling go_positions with %zu objects.",
             object_list.size());

    // MCU reset before starting placement sequence
    callAmrMcu(0, "RESET", 1);

    bool all_succeeded = true;

    for (std::size_t i = 0; i < object_list.size(); ++i)
    {
      const std::string& name = object_list[i];
      ROS_INFO("[amr_go_move] Processing object_info[%zu] = %s",
               i, name.c_str());

      bool goal_ok = sendObjectGoalByName(name);
      if (!goal_ok)
      {
        all_succeeded = false;
        ROS_WARN("[amr_go_move] Navigation to %s failed.", name.c_str());
        // Continue next object even if this one failed
        continue;
      }

      // MCU: PLACE after each successful object placement
      callAmrMcu(2, "PLACE", 1);
    }

    // Only publish DONE when all object goals were successful
    if (all_succeeded && !object_list.empty())
    {
      std_msgs::String state_msg;
      state_msg.data = "DONE";
      mission_pub_.publish(state_msg);
      ROS_INFO("[amr_go_move] Published amr_mission_state = 'DONE'");
    }
    else
    {
      ROS_WARN("[amr_go_move] Not all object goals succeeded. 'DONE' not published.");
    }
  }

  // --------------------------------------------------------------------------
  // OPC UA topic callbacks
  // --------------------------------------------------------------------------

  void opcuaMoveCallback(const std_msgs::String::ConstPtr& msg)
  {
    const std::string& data = msg->data;
    ROS_INFO_STREAM("[amr_go_move] Received /amr/opcua/go_move: " << data);

    if (data == "Ready")
    {
      ROS_INFO("[amr_go_move] State: Ready (no navigation).");
      return;
    }

    // If you want to ignore repeated commands, uncomment this block.
    /*
    if (data == last_move_command_)
    {
      ROS_INFO("[amr_go_move] Same move command as last processed. Ignoring.");
      return;
    }
    */

    std::string effective_cmd;
    std::string cmd = parseMoveCommand(data);

    if (!cmd.empty())
    {
      effective_cmd = cmd;
    }
    else
    {
      // Fallback: simple substring detection
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
      handlePickUpZoneCommand();
    }
    else if (effective_cmd == "go_home")
    {
      handleGoHomeCommand();
    }
    else
    {
      ROS_WARN("[amr_go_move] move_command '%s' is not supported.",
               effective_cmd.c_str());
    }
  }

  void opcuaPositionsCallback(const std_msgs::String::ConstPtr& msg)
  {
    const std::string& data = msg->data;
    ROS_INFO_STREAM("[amr_go_move] Received /amr/opcua/go_positions: " << data);

    if (data == "Ready")
    {
      ROS_INFO("[amr_go_move] State: Ready (no navigation).");
      return;
    }

    // If you want to ignore repeated commands, uncomment this block.
    /*
    if (data == last_positions_command_)
    {
      ROS_INFO("[amr_go_move] Same positions command as last processed. Ignoring.");
      return;
    }
    */

    std::vector<std::string> object_list = parseObjectInfoList(data);
    if (object_list.empty())
    {
      ROS_WARN("[amr_go_move] object_info list is empty or invalid.");
      return;
    }

    last_positions_command_ = data;

    handlePositionsCommand(object_list);
  }

  // --------------------------------------------------------------------------
  // Members
  // --------------------------------------------------------------------------

  ros::NodeHandle nh_;

  ros::Subscriber sub_move_;
  ros::Subscriber sub_positions_;

  ros::Publisher mission_pub_;

  MoveBaseClient ac_;
  tf::TransformListener tf_listener_;

  // Last raw commands (optional duplicate filtering)
  std::string last_move_command_;
  std::string last_positions_command_;

  // Logical destinations (pick-up, home)
  double pick_up_zone_x_;
  double pick_up_zone_y_;
  double pick_up_zone_yaw_deg_;

  double go_home_x_;
  double go_home_y_;
  double go_home_yaw_deg_;

  // Object positions
  double esp32_x_;
  double esp32_y_;
  double esp32_yaw_deg_;

  double motordriver_x_;
  double motordriver_y_;
  double motordriver_yaw_deg_;

  double powersuplpy_x_;
  double powersuplpy_y_;
  double powersuplpy_yaw_deg_;

  // Service clients
  ros::ServiceClient go_aruco_client_;
  ros::ServiceClient amr_mcu_client_;
  ros::ServiceClient set_status_client_;
  ros::ServiceClient set_mission_client_;
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "amr_go_move");
  AmrGoMove node;
  ros::spin();
  return 0;
}
