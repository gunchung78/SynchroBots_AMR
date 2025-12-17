#pragma once
#include <string>
#include <mutex>

// MySQL C API
#include <mysql/mysql.h>

struct DBConfig {
  std::string host     = "172.30.1.29";
  std::string user     = "root";
  std::string password = "1234";
  std::string database = "synchrobots";
  unsigned int port    = 3306;
  std::string charset  = "utf8mb4";
};

struct AmrStateData {
  double pos_x = 0.0;
  double pos_y = 0.0;
  double heading = 0.0;
  double battery_pct = 0.0;
  double speed = 0.0;
};

class AmrStateLogWriter {
public:
  explicit AmrStateLogWriter(
      const DBConfig& cfg = DBConfig(),
      std::string equipment_id = "AMR01"
  );

  ~AmrStateLogWriter();

  bool connect();
  void disconnect();
  bool isConnected() const;

  // 1건 INSERT (성공/실패)
  bool insert(const AmrStateData& d);

  // 마지막 에러 문자열
  std::string lastError() const;

private:
  bool ensureConnected();
  void setError(const std::string& e);

private:
  DBConfig cfg_;
  std::string equipment_id_;

  MYSQL* conn_ = nullptr;

  mutable std::mutex mtx_;
  std::string last_error_;
};
