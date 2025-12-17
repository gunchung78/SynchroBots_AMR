#include <ros/ros.h>
#include <amr_msg/SetMission.h>
#include <memory>
#include <sstream>
#include "amr_db_logger/mission_log_writer.hpp"

class AmrMissionLoggerNode {
public:
  AmrMissionLoggerNode() {
    ros::NodeHandle pnh("~");
    pnh.param("equipment_id", equipment_id_, std::string("AMR01"));

    writer_.reset(new MissionLogWriter(DBConfig(), equipment_id_));

    srv_ = nh_.advertiseService("set_mission", &AmrMissionLoggerNode::onSetMission, this);

    if (!writer_->connect()) ROS_WARN_STREAM("[MISSION][DB] connect failed: " << writer_->lastError());
    else ROS_INFO("[MISSION][DB] connected");

    ROS_INFO_STREAM("[MISSION] ready. service=/" << ros::this_node::getName()
                    << "/set_mission equipment_id=" << equipment_id_);
  }

private:
  bool onSetMission(amr_msg::SetMission::Request& req,
                    amr_msg::SetMission::Response& res) {
    const bool has_status = !req.status.empty();
    const bool has_src    = !req.source_station.empty();
    const bool has_tgt    = !req.target_station.empty();
    const bool has_act    = !req.action_type.empty();
    if (!(has_status || has_src || has_tgt || has_act)) {
      res.success = false;
      res.message = "all fields empty (status/source_station/target_station/action_type)";
      return true;
    }

    MissionRequestData d;
    d.status = req.status;
    d.source_station = req.source_station;
    d.target_station = req.target_station;
    d.action_type = req.action_type;
    
    uint64_t mission_id = 0;
    bool mission_log_updated = false;
    bool amr_log_inserted = false;

    if (!writer_->handleMissionUpdate(d, mission_id, mission_log_updated, amr_log_inserted)) {
      res.success = false;
      res.message = "DB handleMissionUpdate failed: " + writer_->lastError();
      return true;
    }

    res.success = true;
    res.message = "mission_id=" + std::to_string(mission_id)
                + " mission_logs:" + (mission_log_updated ? "updated/inserted" : "nochange")
                + " mission_amr_logs:" + (amr_log_inserted ? "inserted" : "nochange");

    ROS_INFO_STREAM("[MISSION] " << res.message
                    << " status=" << d.status
                    << " src=" << d.source_station
                    << " tgt=" << d.target_station);
    return true;
  }

private:
  ros::NodeHandle nh_;
  ros::ServiceServer srv_;
  std::string equipment_id_ = "AMR01";
  std::unique_ptr<MissionLogWriter> writer_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "amr_mission_logger");
  AmrMissionLoggerNode node;
  ros::spin();
  return 0;
}
