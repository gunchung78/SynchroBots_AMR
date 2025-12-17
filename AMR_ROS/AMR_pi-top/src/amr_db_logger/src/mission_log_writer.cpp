#include "amr_db_logger/mission_log_writer.hpp"
#include <cstring>

static std::string mysqlErr(MYSQL* c) {
  if (!c) return "MYSQL* is null";
  const char* e = mysql_error(c);
  return e ? std::string(e) : std::string("unknown mysql error");
}

MissionLogWriter::MissionLogWriter(const DBConfig& cfg, const std::string& equipment_id)
: cfg_(cfg), equipment_id_(equipment_id) {}

MissionLogWriter::~MissionLogWriter() { disconnect(); }

// ------------------------------------------------------------
// ? connectLocked(): mtx_ 잡힌 상태에서만 호출 (락 중복 금지)
// ------------------------------------------------------------
bool MissionLogWriter::connectLocked() {
  if (conn_) return true;

  conn_ = mysql_init(nullptr);
  if (!conn_) { last_error_ = "mysql_init failed"; return false; }

  mysql_options(conn_, MYSQL_SET_CHARSET_NAME, cfg_.charset.c_str());

  if (!mysql_real_connect(conn_, cfg_.host.c_str(), cfg_.user.c_str(), cfg_.password.c_str(),
                          cfg_.database.c_str(), cfg_.port, nullptr, 0)) {
    last_error_ = "mysql_real_connect failed: " + mysqlErr(conn_);
    mysql_close(conn_);
    conn_ = nullptr;
    return false;
  }

  last_error_.clear();
  return true;
}

bool MissionLogWriter::connect() {
  std::lock_guard<std::mutex> lk(mtx_);
  return connectLocked();
}

void MissionLogWriter::disconnect() {
  std::lock_guard<std::mutex> lk(mtx_);
  if (conn_) { mysql_close(conn_); conn_ = nullptr; }
}

bool MissionLogWriter::isConnected() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return conn_ != nullptr;
}

std::string MissionLogWriter::lastError() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return last_error_;
}

// ------------------------------------------------------------
// ? ensureConnectedLocked(): 락 잡은 상태에서 connect() 호출 금지!
// ------------------------------------------------------------
bool MissionLogWriter::ensureConnectedLocked() {
  return conn_ ? true : connectLocked();
}

void MissionLogWriter::setErrorLocked(const std::string& e) {
  last_error_ = e;
}

// ------------------------------------------------------------
// ? recoverActiveMission(): public (락 1번)
// ------------------------------------------------------------
bool MissionLogWriter::recoverActiveMission(uint64_t& mission_id_out, std::string& status_out) {
  std::lock_guard<std::mutex> lk(mtx_);
  return recoverActiveMissionLocked(mission_id_out, status_out);
}

// ------------------------------------------------------------
// ? recoverActiveMissionLocked(): 락 잡힌 상태에서만 실행
// ------------------------------------------------------------
bool MissionLogWriter::recoverActiveMissionLocked(uint64_t& mission_id_out, std::string& status_out) {
  if (!ensureConnectedLocked()) return false;

  const char* sql =
    "SELECT mission_id, status "
    "FROM mission_logs "
    "WHERE equipment_id=? "
    "ORDER BY mission_id DESC "
    "LIMIT 1";

  MYSQL_STMT* stmt = mysql_stmt_init(conn_);
  if (!stmt) { setErrorLocked("mysql_stmt_init failed"); return false; }

  if (mysql_stmt_prepare(stmt, sql, (unsigned long)std::strlen(sql)) != 0) {
    setErrorLocked("prepare failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  MYSQL_BIND b[1]; std::memset(b, 0, sizeof(b));
  unsigned long eq_len = (unsigned long)equipment_id_.size();
  b[0].buffer_type = MYSQL_TYPE_STRING;
  b[0].buffer = (void*)equipment_id_.c_str();
  b[0].buffer_length = eq_len;
  b[0].length = &eq_len;

  if (mysql_stmt_bind_param(stmt, b) != 0) {
    setErrorLocked("bind_param failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  // ? execute 먼저
  if (mysql_stmt_execute(stmt) != 0) {
    setErrorLocked("execute failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  // result
  uint64_t mid = 0;
  char status_buf[32]; unsigned long status_len = 0;
  bool is_null_mid = false, is_null_status = false;

  MYSQL_BIND r[2]; std::memset(r, 0, sizeof(r));
  r[0].buffer_type = MYSQL_TYPE_LONGLONG;
  r[0].buffer = &mid;
  r[0].is_null = &is_null_mid;

  r[1].buffer_type = MYSQL_TYPE_STRING;
  r[1].buffer = status_buf;
  r[1].buffer_length = sizeof(status_buf);
  r[1].length = &status_len;
  r[1].is_null = &is_null_status;

  if (mysql_stmt_bind_result(stmt, r) != 0) {
    setErrorLocked("bind_result failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  int fetch = mysql_stmt_fetch(stmt);
  mysql_stmt_close(stmt);

  // 0: row 있음, MYSQL_NO_DATA(100): row 없음
  if (fetch != 0) {
    mission_id_out = 0;
    status_out.clear();
    active_mission_id_ = 0;
    active_status_.clear();
    last_error_.clear();
    return true;
  }

  std::string st(status_buf, status_len);

  if (st == "DONE") {
    mission_id_out = 0;
    status_out = st;
    active_mission_id_ = 0;
    active_status_.clear();
  } else {
    mission_id_out = mid;
    status_out = st;
    active_mission_id_ = mid;
    active_status_ = st;
  }

  last_error_.clear();
  return true;
}

// ------------------------------------------------------------
// mission_logs: 새 미션 생성 (DONE 이후에만)
// ------------------------------------------------------------
bool MissionLogWriter::createNewMissionLocked(const std::string& status, uint64_t& mission_id_out) {
  const char* sql =
    "INSERT INTO mission_logs (equipment_id, status) VALUES (?, ?)";

  MYSQL_STMT* stmt = mysql_stmt_init(conn_);
  if (!stmt) { setErrorLocked("mysql_stmt_init failed"); return false; }

  if (mysql_stmt_prepare(stmt, sql, (unsigned long)std::strlen(sql)) != 0) {
    setErrorLocked("prepare failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  MYSQL_BIND b[2]; std::memset(b, 0, sizeof(b));
  unsigned long eq_len = (unsigned long)equipment_id_.size();
  unsigned long st_len = (unsigned long)status.size();

  b[0].buffer_type = MYSQL_TYPE_STRING;
  b[0].buffer = (void*)equipment_id_.c_str();
  b[0].buffer_length = eq_len;
  b[0].length = &eq_len;

  b[1].buffer_type = MYSQL_TYPE_STRING;
  b[1].buffer = (void*)status.c_str();
  b[1].buffer_length = st_len;
  b[1].length = &st_len;

  if (mysql_stmt_bind_param(stmt, b) != 0) {
    setErrorLocked("bind_param failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  if (mysql_stmt_execute(stmt) != 0) {
    setErrorLocked("execute failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  mission_id_out = (uint64_t)mysql_insert_id(conn_);
  mysql_stmt_close(stmt);
  last_error_.clear();
  return true;
}

// ------------------------------------------------------------
// mission_logs: DONE 전까지는 UPDATE만
// ------------------------------------------------------------
bool MissionLogWriter::updateMissionStatusLocked(uint64_t mission_id, const std::string& status) {
  const char* sql =
    "UPDATE mission_logs SET status=? WHERE mission_id=? AND equipment_id=?";

  MYSQL_STMT* stmt = mysql_stmt_init(conn_);
  if (!stmt) { setErrorLocked("mysql_stmt_init failed"); return false; }

  if (mysql_stmt_prepare(stmt, sql, (unsigned long)std::strlen(sql)) != 0) {
    setErrorLocked("prepare failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  MYSQL_BIND b[3]; std::memset(b, 0, sizeof(b));
  unsigned long st_len = (unsigned long)status.size();
  unsigned long eq_len = (unsigned long)equipment_id_.size();

  b[0].buffer_type = MYSQL_TYPE_STRING;
  b[0].buffer = (void*)status.c_str();
  b[0].buffer_length = st_len;
  b[0].length = &st_len;

  b[1].buffer_type = MYSQL_TYPE_LONGLONG;
  b[1].buffer = &mission_id;

  b[2].buffer_type = MYSQL_TYPE_STRING;
  b[2].buffer = (void*)equipment_id_.c_str();
  b[2].buffer_length = eq_len;
  b[2].length = &eq_len;

  if (mysql_stmt_bind_param(stmt, b) != 0) {
    setErrorLocked("bind_param failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  if (mysql_stmt_execute(stmt) != 0) {
    setErrorLocked("execute failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  mysql_stmt_close(stmt);
  last_error_.clear();
  return true;
}

// ------------------------------------------------------------
// mission_amr_logs: 마지막 src/tgt 조회
// ------------------------------------------------------------
bool MissionLogWriter::getLastStationsLocked(uint64_t mission_id,
                                             std::string& last_src,
                                             std::string& last_tgt,
                                             std::string& last_action,
                                             bool& has_row) {
  has_row = false;
  last_src.clear(); last_tgt.clear(); last_action.clear();

  const char* sql =
    "SELECT source_station, target_station, action_type "
    "FROM mission_amr_logs "
    "WHERE mission_id=? "
    "ORDER BY log_amr_id DESC "
    "LIMIT 1";

  MYSQL_STMT* stmt = mysql_stmt_init(conn_);
  if (!stmt) { setErrorLocked("mysql_stmt_init failed"); return false; }

  if (mysql_stmt_prepare(stmt, sql, (unsigned long)std::strlen(sql)) != 0) {
    setErrorLocked("prepare failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  MYSQL_BIND b[1]; std::memset(b, 0, sizeof(b));
  b[0].buffer_type = MYSQL_TYPE_LONGLONG;
  b[0].buffer = &mission_id;

  if (mysql_stmt_bind_param(stmt, b) != 0) {
    setErrorLocked("bind_param failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  if (mysql_stmt_execute(stmt) != 0) {
    setErrorLocked("execute failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  char src_buf[128], tgt_buf[128], act_buf[64];
  unsigned long src_len = 0, tgt_len = 0, act_len=0;
  bool src_null = false, tgt_null = false, act_null=false;

  MYSQL_BIND r[3]; std::memset(r, 0, sizeof(r));
  r[0].buffer_type = MYSQL_TYPE_STRING; r[0].buffer = src_buf; r[0].buffer_length = sizeof(src_buf); r[0].length = &src_len; r[0].is_null = &src_null;
  r[1].buffer_type = MYSQL_TYPE_STRING; r[1].buffer = tgt_buf; r[1].buffer_length = sizeof(tgt_buf); r[1].length = &tgt_len; r[1].is_null = &tgt_null;
  r[2].buffer_type = MYSQL_TYPE_STRING; r[2].buffer = act_buf; r[2].buffer_length = sizeof(act_buf); r[2].length = &act_len; r[2].is_null = &act_null;

  if (mysql_stmt_bind_result(stmt, r) != 0) {
    setErrorLocked("bind_result failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  int fetch = mysql_stmt_fetch(stmt);
  mysql_stmt_close(stmt);

  if (fetch == 0) {
    has_row = true;
    last_src = src_null ? "" : std::string(src_buf, src_len);
    last_tgt = tgt_null ? "" : std::string(tgt_buf, tgt_len);
    last_action = act_null ? "" : std::string(act_buf, act_len);
  } else {
    has_row = false;
  }

  last_error_.clear();
  return true;
}

// ------------------------------------------------------------
// mission_amr_logs: 바뀌면 INSERT, 같으면 스킵
// ------------------------------------------------------------
bool MissionLogWriter::insertMissionAmrLogLocked(uint64_t mission_id,
                                                 const std::string& src,
                                                 const std::string& tgt,
                                                 const std::string& action_type) {
  const char* sql =
    "INSERT INTO mission_amr_logs (mission_id, source_station, target_station, action_type) "
    "VALUES (?, ?, ?, ?)";

  MYSQL_STMT* stmt = mysql_stmt_init(conn_);
  if (!stmt) { setErrorLocked("mysql_stmt_init failed"); return false; }

  if (mysql_stmt_prepare(stmt, sql, (unsigned long)std::strlen(sql)) != 0) {
    setErrorLocked("prepare failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  MYSQL_BIND b[4]; std::memset(b, 0, sizeof(b));
  unsigned long src_len = (unsigned long)src.size();
  unsigned long tgt_len = (unsigned long)tgt.size();
  unsigned long act_len = (unsigned long)action_type.size();
  
  b[0].buffer_type = MYSQL_TYPE_LONGLONG; b[0].buffer = &mission_id;

  b[1].buffer_type = MYSQL_TYPE_STRING; b[1].buffer = (void*)src.c_str(); b[1].buffer_length = src_len; b[1].length = &src_len;
  b[2].buffer_type = MYSQL_TYPE_STRING; b[2].buffer = (void*)tgt.c_str(); b[2].buffer_length = tgt_len; b[2].length = &tgt_len;
  b[3].buffer_type = MYSQL_TYPE_STRING; b[3].buffer = (void*)action_type.c_str(); b[3].buffer_length = act_len; b[3].length = &act_len;

  if (mysql_stmt_bind_param(stmt, b) != 0) {
    setErrorLocked("bind_param failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  if (mysql_stmt_execute(stmt) != 0) {
    setErrorLocked("execute failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  mysql_stmt_close(stmt);
  last_error_.clear();
  return true;
}

// ------------------------------------------------------------
// ? 최종 엔트리: DONE 전 mission_id 고정 + status는 UPDATE, station 바뀌면 INSERT
// ------------------------------------------------------------
bool MissionLogWriter::handleMissionUpdate(const MissionRequestData& in,
                                           uint64_t& mission_id_out,
                                           bool& mission_log_updated,
                                           bool& amr_log_inserted) {
  std::lock_guard<std::mutex> lk(mtx_);
  mission_log_updated = false;
  amr_log_inserted = false;

  if (!ensureConnectedLocked()) return false;

  const bool has_status = !in.status.empty();
  const bool has_src    = !in.source_station.empty();
  const bool has_tgt    = !in.target_station.empty();
  const bool has_act    = !in.action_type.empty();
  const bool has_any_station_field = (has_src || has_tgt || has_act);

  // 1) active 없으면 DB에서 복구 시도 (? locked 버전 호출)
  if (active_mission_id_ == 0) {
    uint64_t recovered = 0;
    std::string st;
    if (!recoverActiveMissionLocked(recovered, st)) return false;
  }

  // 2) active 없으면 새 mission 생성
  //    - status가 비어있으면 기본값으로 생성(원하면 WAITING 고정)
  if (active_mission_id_ == 0) {
    uint64_t new_id = 0;
    const std::string create_status = has_status ? in.status : "WAITING";
    if (!createNewMissionLocked(create_status, new_id)) return false;
    active_mission_id_ = new_id;
    active_status_ = create_status;
    mission_log_updated = true; // INSERT
  } 
  else {
    // 3) status 처리
    //    - status가 비어있으면: "이전 값 참조"만 하고 UPDATE는 안 함
    //    - status가 있고 값이 바뀌면: UPDATE
    if (has_status) {
      if (in.status != active_status_) {
        if (!updateMissionStatusLocked(active_mission_id_, in.status)) return false;
        active_status_ = in.status;
        mission_log_updated = true;
      }
    }
  }

  // 4) station/action 처리
  //    - source/target/action 중 하나라도 들어오면
  //      나머지 빈 값은 "이전 값"으로 보정해서 INSERT 판단
  if (has_any_station_field) {
    std::string last_src, last_tgt, last_action;
    bool has_row = false;
    if (!getLastStationsLocked(active_mission_id_, last_src, last_tgt, last_action, has_row)) return false;

    if (!has_row) {
      // 첫 station log는 보정 불가 -> 3개 모두 필요
      if (in.source_station.empty() || in.target_station.empty() || in.action_type.empty()) {
        setErrorLocked("first mission_amr_logs row needs source_station, target_station, action_type");
        return false;
      }
      if (!insertMissionAmrLogLocked(active_mission_id_, in.source_station, in.target_station, in.action_type)) return false;
      amr_log_inserted = true;
    } else {
      // ? 빈 값은 이전 값으로 보정
      const std::string eff_src = in.source_station.empty() ? last_src : in.source_station;
      const std::string eff_tgt = in.target_station.empty() ? last_tgt : in.target_station;
      const std::string eff_act = in.action_type.empty() ? last_action : in.action_type;

      // 보정 후에도 비어있으면(예: DB에 last_action이 비어있음) -> 불가
      if (eff_src.empty() || eff_tgt.empty() || eff_act.empty()) {
        setErrorLocked("cannot infer missing station/action from DB (please send full fields once)");
        return false;
      }

      // ? 보정된 값이 "이전 row"와 다를 때만 INSERT
      if (eff_src != last_src || eff_tgt != last_tgt || eff_act != last_action) {
        if (!insertMissionAmrLogLocked(active_mission_id_, eff_src, eff_tgt, eff_act)) return false;
        amr_log_inserted = true;
      }
    }
  }

  mission_id_out = active_mission_id_;

  // 5) DONE이면 미션 종료 (다음 호출에서 새 mission 생성)
  //    status가 비어있으면 이전값 참조이므로, DONE 처리도 "요청에 status가 있을 때만" 수행
  if (has_status && in.status == "DONE") {
    active_mission_id_ = 0;
    active_status_.clear();
  }

  last_error_.clear();
  return true;
}
