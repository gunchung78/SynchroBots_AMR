#include "amr_db_logger/amr_state_log_writer.hpp"
#include <cstring>   // memset, strlen

static std::string mysqlErr(MYSQL* c) {
  if (!c) return "MYSQL* is null";
  const char* e = mysql_error(c);
  return e ? std::string(e) : std::string("unknown mysql error");
}

AmrStateLogWriter::AmrStateLogWriter(const DBConfig& cfg,
                                     std::string equipment_id)
  : cfg_(cfg),
    equipment_id_(std::move(equipment_id)) {}

AmrStateLogWriter::~AmrStateLogWriter() {
  disconnect();
}

bool AmrStateLogWriter::connect() {
  std::lock_guard<std::mutex> lk(mtx_);

  if (conn_) return true;

  conn_ = mysql_init(nullptr);
  if (!conn_) {
    setError("mysql_init failed");
    return false;
  }

  // charset 설정
  mysql_options(conn_, MYSQL_SET_CHARSET_NAME, cfg_.charset.c_str());

  // 실제 접속
  if (!mysql_real_connect(conn_,
                          cfg_.host.c_str(),
                          cfg_.user.c_str(),
                          cfg_.password.c_str(),
                          cfg_.database.c_str(),
                          cfg_.port,
                          nullptr,
                          0)) {
    setError("mysql_real_connect failed: " + mysqlErr(conn_));
    mysql_close(conn_);
    conn_ = nullptr;
    return false;
  }

  setError(""); // clear
  return true;
}

void AmrStateLogWriter::disconnect() {
  std::lock_guard<std::mutex> lk(mtx_);
  if (conn_) {
    mysql_close(conn_);
    conn_ = nullptr;
  }
}

bool AmrStateLogWriter::isConnected() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return conn_ != nullptr;
}

std::string AmrStateLogWriter::lastError() const {
  std::lock_guard<std::mutex> lk(mtx_);
  return last_error_;
}

void AmrStateLogWriter::setError(const std::string& e) {
  last_error_ = e;
}

bool AmrStateLogWriter::ensureConnected() {
  if (conn_) return true;
  return connect();
}

bool AmrStateLogWriter::insert(const AmrStateData& d) {
  std::lock_guard<std::mutex> lk(mtx_);

  if (!ensureConnected()) return false;

  // idx(PK auto_increment), updated_at(default timestamp)는 INSERT에서 제외
  const char* sql =
      "INSERT INTO amr_state_log "
      "(equipment_id, pos_x, pos_y, heading, battery_pct, speed) "
      "VALUES (?, ?, ?, ?, ?, ?)";

  MYSQL_STMT* stmt = mysql_stmt_init(conn_);
  if (!stmt) {
    setError("mysql_stmt_init failed");
    return false;
  }

  if (mysql_stmt_prepare(stmt, sql, (unsigned long)std::strlen(sql)) != 0) {
    setError("mysql_stmt_prepare failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  MYSQL_BIND bind[6];
  std::memset(bind, 0, sizeof(bind));

  // 1) equipment_id (string)
  unsigned long equipment_len = (unsigned long)equipment_id_.size();
  bind[0].buffer_type = MYSQL_TYPE_STRING;
  bind[0].buffer = (void*)equipment_id_.c_str();
  bind[0].buffer_length = equipment_len;
  bind[0].length = &equipment_len;

  // 2~6) double
  double pos_x = d.pos_x;
  double pos_y = d.pos_y;
  double heading = d.heading;
  double battery = d.battery_pct;
  double speed = d.speed;

  bind[1].buffer_type = MYSQL_TYPE_DOUBLE; bind[1].buffer = &pos_x;
  bind[2].buffer_type = MYSQL_TYPE_DOUBLE; bind[2].buffer = &pos_y;
  bind[3].buffer_type = MYSQL_TYPE_DOUBLE; bind[3].buffer = &heading;
  bind[4].buffer_type = MYSQL_TYPE_DOUBLE; bind[4].buffer = &battery;
  bind[5].buffer_type = MYSQL_TYPE_DOUBLE; bind[5].buffer = &speed;

  if (mysql_stmt_bind_param(stmt, bind) != 0) {
    setError("mysql_stmt_bind_param failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  if (mysql_stmt_execute(stmt) != 0) {
    setError("mysql_stmt_execute failed: " + std::string(mysql_stmt_error(stmt)));
    mysql_stmt_close(stmt);
    return false;
  }

  mysql_stmt_close(stmt);
  setError("");
  return true;
}
