#include <Wire.h>
#include <math.h>
#include "Adafruit_VL53L0X.h"

//////////////////////////////
// Dual VL53L0X I2C Address
//////////////////////////////
#define LOX1_ADDRESS 0x30  // FRONT: 박스 감지용
#define LOX2_ADDRESS 0x31  // REAR : 밀어낸 거리 측정용

// XSHUT 핀 (센서 리셋용)
#define SHT_LOX1 7
#define SHT_LOX2 6

// VL53L0X objects
Adafruit_VL53L0X lox1;  // FRONT sensor
Adafruit_VL53L0X lox2;  // REAR sensor

bool frontTofOk = false;
bool rearTofOk  = false;

// 마지막 측정 거리 (cm)
float frontDistCm = 999.0f;
float rearDistCm  = 999.0f;

//////////////////////////////
// Pin Config
//////////////////////////////
const uint8_t LIMIT_FRONT_PIN = 3;   // FRONT limit (ACTIVE LOW, INPUT_PULLUP)
const uint8_t LIMIT_REAR_PIN  = 12;  // REAR  limit (ACTIVE LOW, INPUT_PULLUP)

// L9110S 제어핀
const uint8_t MOTOR_DIR_PIN   = 8;   // 방향(DIR)  → L9110 IN2
const uint8_t MOTOR_PWM_PIN   = 9;   // 속도(PWM) → L9110 IN1 (PWM 핀)

// 모터 속도 (0~255)
const uint8_t MOTOR_SPEED_FWD = 220;  // 앞으로 밀 때 속도
const uint8_t MOTOR_SPEED_REV = 180;  // 뒤로 돌아올 때 속도

//////////////////////////////
// Self Test (0: off)
//////////////////////////////
#define SELF_TEST 0

//////////////////////////////
// Tunable Parameters
//////////////////////////////
const int   DIST_THRESHOLD_CM        = 5;     // 앞 센서 박스 감지 임계값

// 로딩용 거리 (rear delta, cm)
//  - 1,2번째 로딩: 동일 거리만큼 밀기
const float LOAD_PUSH_CM_12         = 6.0f;  // 1,2번째 로딩용
//  - 3번째 로딩: 전혀 밀지 않음 (drop only)

// 언로딩용 거리 (rear delta, cm)
//  - 3개 있을 때: 조금 밀기 (짧은 거리)
//  - 2개 있을 때: 3개보다 더 많이 밀기 (중간 거리)
//  - 1개 있을 때: delta 안 쓰고, front limit까지 밀기
const float UNLOAD_PUSH_CM_3        = 6.5f;   // 3개 있을 때 언로딩
const float UNLOAD_PUSH_CM_2        = 9.0f;   // 2개 있을 때 언로딩

// delta 비교 여유 (센서 노이즈 대응)
const float PUSH_DELTA_TOL_CM       = 0.3f;

// 안전 타임아웃 (ms)
const unsigned long MAX_PUSH_TIME_MS = 4000UL;

// INPUT_PULLUP 기준: 눌리면 LOW
const uint8_t LIMIT_ACTIVE_STATE = LOW;

// 중복 트리거/바운스 완화
const unsigned long ACTION_COOLDOWN_MS = 200UL;
const unsigned long LIMIT_DEBOUNCE_MS  = 20UL;

// 거리센서 주기적 갱신 주기
const unsigned long SENSOR_READ_INTERVAL_MS = 50UL;
unsigned long lastSensorReadMs = 0;

//////////////////////////////
// Mode & State (low-level)
//////////////////////////////
enum WorkMode : uint8_t {
  MODE_NONE      = 0,
  MODE_LOADING   = 1,
  MODE_UNLOADING = 2
};

enum MotionState : uint8_t {
  ST_IDLE = 0,
  ST_HOME_BACK,
  ST_WAIT_READY,
  ST_MOVE_FWD,
  ST_MOVE_REV,
  ST_COMPLETE
};

WorkMode    mode  = MODE_NONE;
MotionState state = ST_IDLE;

//////////////////////////////
// High-level command mode (from Raspi)
//////////////////////////////
enum HighCmdMode : uint8_t {
  HC_IDLE    = 0,   // 0: 대기 (기본값) → rear limit 쪽으로 붙기
  HC_LOADING = 1    // 1: 계속 박스 받기
};

// ★ 기본값을 0(HC_IDLE)로 유지
HighCmdMode highCmdMode = HC_LOADING;

//////////////////////////////
// Push stop 방식
//////////////////////////////
enum PushStopMode : uint8_t {
  PUSH_STOP_NONE = 0,
  PUSH_STOP_DELTA,
  PUSH_STOP_FRONT_LIMIT
};

PushStopMode pushStopMode = PUSH_STOP_NONE;
float        targetDeltaCm = 0.0f;

//////////////////////////////
// Box Memory
//////////////////////////////
int     currentCount      = 0;   // 0~3
uint8_t boxesBeforeAction = 0;

//////////////////////////////
// Error latch
//////////////////////////////
bool errorFlag = false;

//////////////////////////////
// Timing
//////////////////////////////
unsigned long lastActionMs   = 0;
unsigned long lastLimitHitMs = 0;
unsigned long moveFwdStartMs = 0;

//////////////////////////////
// Rear distance 측정용
//////////////////////////////
bool  rearPushValid   = false;
float rearPushStartCm = 0.0f;

//////////////////////////////
// --- Low-level IO ---
//////////////////////////////
void motorForward() {
  analogWrite(MOTOR_PWM_PIN, MOTOR_SPEED_FWD);
  digitalWrite(MOTOR_DIR_PIN, LOW);
}

void motorReverse() {
  analogWrite(MOTOR_PWM_PIN, 255 - MOTOR_SPEED_REV);
  digitalWrite(MOTOR_DIR_PIN, HIGH);
}

void motorStop() {
  analogWrite(MOTOR_PWM_PIN, 0);
}

bool isFrontLimitActive() {
  return (digitalRead(LIMIT_FRONT_PIN) == LIMIT_ACTIVE_STATE);
}

bool isRearLimitActive() {
  return (digitalRead(LIMIT_REAR_PIN) == LIMIT_ACTIVE_STATE);
}

bool limitDebounced() {
  unsigned long now = millis();
  if (now - lastLimitHitMs < LIMIT_DEBOUNCE_MS) return false;
  lastLimitHitMs = now;
  return true;
}

//////////////////////////////
// Dual VL53L0X init
//////////////////////////////
void initToFSensors() {
  Serial.println(F("[INIT] VL53L0X dual init"));

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);
  delay(10);

  // FRONT
  digitalWrite(SHT_LOX1, HIGH);
  delay(10);

  if (!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("[ERR] FRONT VL53L0X"));
    frontTofOk = false;
  } else {
    frontTofOk = true;
    Serial.println(F("[OK] FRONT @0x30"));
  }

  // REAR
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  if (!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("[ERR] REAR VL53L0X"));
    rearTofOk = false;
  } else {
    rearTofOk = true;
    Serial.println(F("[OK] REAR @0x31"));
  }
}

//////////////////////////////
// Distance read (once)
//////////////////////////////
float readFrontDistanceCmOnce() {
  if (!frontTofOk) return 999.0f;
  VL53L0X_RangingMeasurementData_t m;
  lox1.rangingTest(&m, false);
  if (m.RangeStatus != 4) {
    float cm = m.RangeMilliMeter / 10.0f;
    if (cm < 0) cm = 0;
    return cm;
  }
  return 999.0f;
}

float readRearDistanceCmOnce() {
  if (!rearTofOk) return 999.0f;
  VL53L0X_RangingMeasurementData_t m;
  lox2.rangingTest(&m, false);
  if (m.RangeStatus != 4) {
    float cm = m.RangeMilliMeter / 10.0f;
    if (cm < 0) cm = 0;
    return cm;
  }
  return 999.0f;
}

// 주기적으로 frontDistCm / rearDistCm 갱신
void updateDistancesPeriodic() {
  unsigned long now = millis();
  if (now - lastSensorReadMs < SENSOR_READ_INTERVAL_MS) return;
  lastSensorReadMs = now;

  frontDistCm = readFrontDistanceCmOnce();
  rearDistCm  = readRearDistanceCmOnce();
}

//////////////////////////////
// Error helpers
//////////////////////////////
void setErrorSimple() {
  errorFlag   = true;
  motorStop();
  state       = ST_IDLE;
  mode        = MODE_NONE;
  highCmdMode = HC_IDLE;
}

void clearError() {
  errorFlag = false;
  Serial.println(F("[OK] Error cleared"));
}

//////////////////////////////
// Status
//////////////////////////////
void printStatus() {
  Serial.print(F("[STATE] highCmd="));
  Serial.print((int)highCmdMode);
  Serial.print(F(" m="));
  Serial.print((int)mode);
  Serial.print(F(" s="));
  Serial.print((int)state);
  Serial.print(F(" cnt="));
  Serial.print(currentCount);
  Serial.print(F(" front="));
  Serial.print(frontDistCm);
  Serial.print(F("cm rear="));
  Serial.print(rearDistCm);
  Serial.println(F("cm"));
}

//////////////////////////////
// UART Parsing: '0' / '1' / '2'
//////////////////////////////
void handleSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  // 유틸 명령
  if (line == F("STATUS")) {
    printStatus();
    return;
  }
  if (line == F("CLRERR")) {
    clearError();
    return;
  }
  if (line == F("RESETALL")) {
    clearError();
    mode          = MODE_NONE;
    state         = ST_IDLE;
    currentCount  = 0;
    rearPushValid = false;
    highCmdMode   = HC_IDLE;
    motorStop();
    Serial.println(F("[OK] Reset"));
    printStatus();
    return;
  }

  // 메인 명령: "0", "1", "2"
  if (line == "0") {
    highCmdMode = HC_IDLE;   // ★ 0 일 때는 항상 대기 + 홈으로 복귀
    Serial.println(F("[CMD] 0 -> HC_IDLE (stop auto loading, go home to rear limit)"));
    printStatus();
    return;
  }

  if (line == "1") {
    highCmdMode = HC_LOADING;
    Serial.println(F("[CMD] 1 -> HC_LOADING (auto load while boxes come)"));
    printStatus();
    return;
  }

  if (line == "2") {
    // 단발 언로딩
    if (mode != MODE_NONE || state != ST_IDLE) {
      Serial.println(F("[WARN] Busy, ignore '2'"));
      return;
    }
    if (currentCount <= 0) {
      Serial.println(F("[ERR] No boxes to unload (cnt=0)"));
      return;
    }

    mode          = MODE_UNLOADING;
    rearPushValid = false;
    state         = isRearLimitActive() ? ST_WAIT_READY : ST_HOME_BACK;

    Serial.print(F("[CMD] 2 -> UNLOAD once, boxes="));
    Serial.println(currentCount);
    printStatus();
    return;
  }

  Serial.print(F("[ERR] Unknown cmd: "));
  Serial.println(line);
}

//////////////////////////////
// Push profile 결정
//////////////////////////////
void configurePushProfile() {
  boxesBeforeAction = (uint8_t)currentCount;
  pushStopMode      = PUSH_STOP_NONE;
  targetDeltaCm     = 0.0f;

  // -------------------------
  // LOADING
  // -------------------------
  if (mode == MODE_LOADING) {
    if (boxesBeforeAction == 0 || boxesBeforeAction == 1) {
      // 1번째, 2번째 박스 → rear delta 기반으로 동일 거리만큼 밀기
      pushStopMode  = PUSH_STOP_DELTA;
      targetDeltaCm = LOAD_PUSH_CM_12;
    } else if (boxesBeforeAction == 2) {
      // 3번째 박스 → 전혀 밀지 않음 (실린더는 HOME 유지, drop만)
      pushStopMode  = PUSH_STOP_NONE;
      targetDeltaCm = 0.0f;
      Serial.println(F("[LOAD] 3rd box: no forward push (drop only)."));
    } else {
      setErrorSimple();
      return;
    }
  }

  // -------------------------
  // UNLOADING
  // -------------------------
  else if (mode == MODE_UNLOADING) {
    if (boxesBeforeAction >= 3) {
      // 3개 있을 때: rear delta로 조금만 밀기
      pushStopMode  = PUSH_STOP_DELTA;
      targetDeltaCm = UNLOAD_PUSH_CM_3;
      Serial.println(F("[UNLOAD] 3 boxes: small push (delta-based)."));
    } else if (boxesBeforeAction == 2) {
      // 2개 있을 때: 3개보다 더 많이 밀기
      pushStopMode  = PUSH_STOP_DELTA;
      targetDeltaCm = UNLOAD_PUSH_CM_2;
      Serial.println(F("[UNLOAD] 2 boxes: medium push (delta-based)."));
    } else if (boxesBeforeAction == 1) {
      // 1개 있을 때: front limit까지
      pushStopMode  = PUSH_STOP_FRONT_LIMIT;
      targetDeltaCm = 0.0f;
      Serial.println(F("[UNLOAD] 1 box: front limit required."));
    } else {
      setErrorSimple();
      return;
    }
  }
}

//////////////////////////////
// Action completion
//////////////////////////////
void onActionCompleteUpdateCounts() {
  if (mode == MODE_LOADING) {
    currentCount++;
    if (currentCount > 3) currentCount = 3;

    Serial.print(F("[LOAD] cnt="));
    Serial.println(currentCount);

    mode  = MODE_NONE;
    state = ST_IDLE;
    return;
  }

  if (mode == MODE_UNLOADING) {
    currentCount--;
    if (currentCount < 0) currentCount = 0;

    Serial.print(F("[UNLOAD] cnt="));
    Serial.println(currentCount);

    mode  = MODE_NONE;
    state = ST_IDLE;
    return;
  }

  state = ST_IDLE;
}

//////////////////////////////
// State handlers
//////////////////////////////
void handleStateHomeBack() {
  if (isRearLimitActive()) {
    motorStop();
    state        = ST_WAIT_READY;
    lastActionMs = millis();
    return;
  }
  motorReverse();
  delay(5);
}

void handleStateWaitReady() {
  if (!isRearLimitActive()) {
    state        = ST_HOME_BACK;
    lastActionMs = millis();
    return;
  }

  // LOADING일 때만 앞 센서로 박스 감지
  if (mode == MODE_LOADING) {
    float distF      = frontDistCm;
    bool  readyToAct = (distF < DIST_THRESHOLD_CM);
    if (!readyToAct) {
      delay(10);
      return;
    }
    // ★ 박스 감지 후 약간 대기하고 로딩 시작
    delay(700);
  }

  configurePushProfile();
  if (errorFlag) return;

  // LOADING 3번째: 전진 없이 바로 완료 처리
  if (mode == MODE_LOADING && boxesBeforeAction == 2) {
    Serial.println(F("[LOAD] 3rd box: no move -> complete."));
    state        = ST_COMPLETE;
    lastActionMs = millis();
    return;
  }

  // rear 기준 시작 거리 저장
  if (rearTofOk) {
    rearPushValid   = true;
    rearPushStartCm = rearDistCm;
  } else {
    rearPushValid = false;
  }

  motorForward();
  moveFwdStartMs = millis();
  state          = ST_MOVE_FWD;
  lastActionMs   = millis();
}

void handleStateMoveFwd() {
  unsigned long now = millis();

  if (now - moveFwdStartMs > MAX_PUSH_TIME_MS) {
    motorStop();
    Serial.println(F("[ERR] push timeout"));
    setErrorSimple();
    return;
  }

  // 공통 safety
  if (isFrontLimitActive() && limitDebounced()) {
    motorStop();
    delay(1000);
    motorReverse();
    state        = ST_MOVE_REV;
    lastActionMs = now;
    return;
  }

  if (pushStopMode == PUSH_STOP_DELTA && rearPushValid) {
    float delta = fabs(rearDistCm - rearPushStartCm);
    if (delta + PUSH_DELTA_TOL_CM >= targetDeltaCm) {
      motorStop();
      delay(1000);
      motorReverse();
      state        = ST_MOVE_REV;
      lastActionMs = now;
      return;
    }
  }

  // PUSH_STOP_FRONT_LIMIT 모드는 위 safety(front limit)로 정지
  motorForward();
  delay(5);
}

void handleStateMoveRev() {
  if (isRearLimitActive() && limitDebounced()) {
    motorStop();
    state        = ST_COMPLETE;
    lastActionMs = millis();
    return;
  }
  motorReverse();
  delay(5);
}

void handleStateComplete() {
  onActionCompleteUpdateCounts();
  rearPushValid = false;
  lastActionMs  = millis();
}

//////////////////////////////
// State machine driver
//////////////////////////////
void stepStateMachine() {
  if (millis() - lastActionMs < ACTION_COOLDOWN_MS) {
    delay(5);
    return;
  }

  // --------------------------------------
  // (A) highCmdMode == 0 (HC_IDLE) 일 때:
  //     항상 rear limit 쪽으로 "홈 인"
  // --------------------------------------
  if (mode == MODE_NONE && highCmdMode == HC_IDLE) {
    if (!isRearLimitActive()) {
      // 리어 리밋 안 닿아 있으면 계속 후진
      motorReverse();
    } else {
      // 리어 리밋 닿으면 정지
      motorStop();
    }
    delay(10);
    return;
  }

  // --------------------------------------
  // (B) idle + HC_LOADING일 때,
  //     앞에 박스가 있으면 자동 로딩 1회 시작
  // --------------------------------------
  if (mode == MODE_NONE && state == ST_IDLE) {
    if (highCmdMode == HC_LOADING && currentCount < 3 && isRearLimitActive()) {
      if (frontDistCm < DIST_THRESHOLD_CM) {
        mode  = MODE_LOADING;
        state = ST_WAIT_READY;
        Serial.println(F("[AUTO] Start one loading action"));
      }
    }
  }

  // 2) 더 이상 액션이 없으면 모터 정지
  if (mode == MODE_NONE) {
    motorStop();
    delay(20);
    return;
  }

  // 3) 원래 state machine
  switch (state) {
    case ST_HOME_BACK:  handleStateHomeBack();   break;
    case ST_WAIT_READY: handleStateWaitReady();  break;
    case ST_MOVE_FWD:   handleStateMoveFwd();    break;
    case ST_MOVE_REV:   handleStateMoveRev();    break;
    case ST_COMPLETE:   handleStateComplete();   break;
    case ST_IDLE:
    default:
      if (isRearLimitActive()) state = ST_WAIT_READY;
      else                     state = ST_HOME_BACK;
      break;
  }
}

//////////////////////////////
// Setup / Loop
//////////////////////////////
void initIO() {
  pinMode(LIMIT_FRONT_PIN, INPUT_PULLUP);
  pinMode(LIMIT_REAR_PIN,  INPUT_PULLUP);

  pinMode(MOTOR_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DIR_PIN,  OUTPUT);

  motorStop();
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println(F("=== Loader dual VL53L0X (0/1/2 Raspi control + auto home) ==="));

  Wire.begin();
  initToFSensors();
  initIO();

  Serial.println(F("[BOOT] Ready"));
  Serial.println(F("Raspi commands: '0' (idle+home), '1' (auto load), '2' (unload one)"));
  Serial.println(F("Utility: STATUS / CLRERR / RESETALL"));
  printStatus();
}

void loop() {
  handleSerial();

  if (errorFlag) {
    motorStop();
    delay(20);
    return;
  }

  updateDistancesPeriodic();
  stepStateMachine();
}
