// esp32_imu_can_sender.ino
// ESP32 #1: SEN0253(BNO055)에서 Euler 각도를 읽어서
// CAN(TWAI)으로 ESP32 #2에게 전송

#include <Arduino.h>
#include <Wire.h>
#include "driver/twai.h"
#include "DFRobot_BNO055.h"

// ===== I2C 핀 (IMU) =====
#define IMU_SDA 21
#define IMU_SCL 22

// ===== CAN 핀 (SN65HVD230) =====
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_16;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_17;

// ===== BNO055 객체 =====
typedef DFRobot_BNO055_IIC BNO;
BNO bno(&Wire, 0x28);   // COM3=LOW 기본 주소 0x28

// ===== CAN 설정 =====
const uint32_t CAN_ID_IMU_EULER = 0x100;  // 임의로 0x100 사용
// 속도는 250kbps (STM32와 맞추기) – 나중에 500kbps 쓰고 싶으면 500으로 변경

void printLastStatus(BNO::eStatus_t s) {
  switch (s) {
    case BNO::eStatusOK:                  Serial.println("BNO status: OK"); break;
    case BNO::eStatusErrDeviceNotDetect:  Serial.println("BNO status: DEVICE NOT DETECTED"); break;
    case BNO::eStatusErrDeviceReadyTimeOut: Serial.println("BNO status: READY TIMEOUT"); break;
    default:                              Serial.println("BNO status: ERROR"); break;
  }
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32 #1 : IMU + CAN Sender ===");

  // I2C 시작
  Wire.begin(IMU_SDA, IMU_SCL);

  // BNO055 초기화
  bno.reset();
  delay(100);

  while (bno.begin() != BNO::eStatusOK) {
    Serial.println("bno.begin FAILED");
    printLastStatus(bno.lastOperateStatus);
    Serial.println("Check I2C wiring and address (0x28 / 0x29)");
    delay(2000);
  }
  Serial.println("bno.begin SUCCESS");

  bno.setOprMode(BNO::eOprModeNdof);
  delay(20);

  // TWAI(CAN) 설정
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  // 250 kbps 타이밍
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  // 모든 ID 수신 (여긴 송신 위주지만 일단 풀 오픈)
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("twai_driver_install FAILED");
    while (1) delay(1000);
  }
  if (twai_start() != ESP_OK) {
    Serial.println("twai_start FAILED");
    while (1) delay(1000);
  }

  Serial.println("TWAI started @ 250kbps");
}

unsigned long lastSendMs = 0;

void loop()
{
  unsigned long now = millis();

  // 100ms마다(10Hz) IMU 읽어서 CAN으로 전송
  if (now - lastSendMs >= 100) {
    lastSendMs = now;

    // 1) IMU에서 Euler 각도 읽기 (deg)
    BNO::sEulAnalog_t eul = bno.getEul();

    float yaw   = eul.head;   // heading
    float roll  = eul.roll;
    float pitch = eul.pitch;

    // 2) CAN 8바이트에 담기 위해 0.01도 단위 정수(int16)로 변환
    int16_t yaw_centi   = (int16_t)roundf(yaw   * 100.0f);
    int16_t roll_centi  = (int16_t)roundf(roll  * 100.0f);
    int16_t pitch_centi = (int16_t)roundf(pitch * 100.0f);

    twai_message_t tx_msg = {};
    tx_msg.identifier = CAN_ID_IMU_EULER;
    tx_msg.extd = 0;   // 표준 ID
    tx_msg.rtr = 0;    // 데이터 프레임
    tx_msg.data_length_code = 6; // 3 * int16_t = 6바이트

    // little-endian으로 데이터 패킹
    tx_msg.data[0] = (uint8_t)(yaw_centi & 0xFF);
    tx_msg.data[1] = (uint8_t)((yaw_centi >> 8) & 0xFF);
    tx_msg.data[2] = (uint8_t)(roll_centi & 0xFF);
    tx_msg.data[3] = (uint8_t)((roll_centi >> 8) & 0xFF);
    tx_msg.data[4] = (uint8_t)(pitch_centi & 0xFF);
    tx_msg.data[5] = (uint8_t)((pitch_centi >> 8) & 0xFF);
    // data[6], data[7]는 비워둠

    esp_err_t res = twai_transmit(&tx_msg, pdMS_TO_TICKS(20));
    if (res == ESP_OK) {
      Serial.print("[TX] yaw=");   Serial.print(yaw, 2);
      Serial.print(" roll=");      Serial.print(roll, 2);
      Serial.print(" pitch=");     Serial.print(pitch, 2);
      Serial.print("  (centi: ");
      Serial.print(yaw_centi); Serial.print(",");
      Serial.print(roll_centi); Serial.print(",");
      Serial.print(pitch_centi); Serial.println(")");
    } else {
      Serial.print("[TX ERROR] twai_transmit res=");
      Serial.println((int)res);
    }
  }

  // (원하면 여기서 수신도 볼 수 있지만, 이 노드는 송신 위주라 생략)
  delay(1);
}
