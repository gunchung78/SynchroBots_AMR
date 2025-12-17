#pragma once
#include <string>
#include <mutex>
#include <cstdint>
#include <mysql/mysql.h>

struct DBConfig {
  std::string host     = "172.30.1.29";
  std::string user     = "root";
  std::string password = "1234";
  std::string database = "synchrobots";
  unsigned int port    = 3306;
  std::string charset  = "utf8mb4";
};

struct MissionRequestData {
  std::string status;          // WAITING/RUNNING/DONE/ERROR
  std::string source_station;
  std::string target_station;
  std::string action_type;
};

class MissionLogWriter {
public:
  MissionLogWriter(const DBConfig& cfg = DBConfig(),
                   const std::string& equipment_id = "AMR01");
  ~MissionLogWriter();

  bool connect();
  void disconnect();
  bool isConnected() const;
  std::string lastError() const;

  // ? 핵심 API: DONE 전에는 mission_id 유지 + UPDATE, station 변경은 mission_amr_logs INSERT
  bool handleMissionUpdate(const MissionRequestData& in,
                           uint64_t& mission_id_out,
                           bool& mission_log_updated,     // mission_logs UPDATE/INSERT 여부
                           bool& amr_log_inserted);       // mission_amr_logs INSERT 여부

  // 노드 재시작 시: 마지막 미션 복구 (DONE이 아니면 그 mission_id를 계속 사용)
  bool recoverActiveMission(uint64_t& mission_id_out, std::string& status_out);

private:
  bool ensureConnectedLocked();
  void setErrorLocked(const std::string& e);

  // mission_logs
  bool createNewMissionLocked(const std::string& status, uint64_t& mission_id_out);
  bool updateMissionStatusLocked(uint64_t mission_id, const std::string& status);

  // mission_amr_logs
  bool getLastStationsLocked(uint64_t mission_id,
                             std::string& last_src,
                             std::string& last_tgt,
                             std::string& last_action,
                             bool& has_row);
  bool insertMissionAmrLogLocked(uint64_t mission_id,
                                 const std::string& src,
                                 const std::string& tgt,
                                 const std::string& action_type);

private:
  bool connectLocked(); 
  bool recoverActiveMissionLocked(uint64_t& mission_id_out, std::string& status_out);
  
  DBConfig cfg_;
  std::string equipment_id_;
  MYSQL* conn_ = nullptr;

  mutable std::mutex mtx_;
  std::string last_error_;

  // 캐시(옵션): DB 조회 줄이기
  uint64_t active_mission_id_ = 0;
  std::string active_status_;
};
