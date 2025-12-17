// esp32_imu_can_receiver.ino
// ESP32 #2: CAN(TWAI)으로 들어오는 IMU Euler 패킷을 수신해서
// 시리얼에 출력

#include <Arduino.h>
#include "driver/twai.h"

// CAN 핀 (SN65HVD230)
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_16;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_17;

// 보낼 때 쓴 ID와 동일
const uint32_t CAN_ID_IMU_EULER = 0x100;

void setup()
{
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP32 #2 : CAN IMU Receiver ===");

  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("twai_driver_install FAILED");
    while (1) delay(1000);
  }
  if (twai_start() != ESP_OK) {
    Serial.println("twai_start FAILED");
    while (1) delay(1000);
  }

  Serial.println("TWAI started @ 250kbps, waiting for IMU frames...");
}

void loop()
{
  twai_message_t rx_msg;
  // 100ms 타임아웃으로 대기 (없으면 바로 리턴)
  esp_err_t res = twai_receive(&rx_msg, pdMS_TO_TICKS(100));

  if (res == ESP_OK) {
    if ((rx_msg.identifier == CAN_ID_IMU_EULER) &&
        (rx_msg.data_length_code >= 6)) {

      // int16_t로 값 복원
      int16_t yaw_centi   = (int16_t)(rx_msg.data[0] | (rx_msg.data[1] << 8));
      int16_t roll_centi  = (int16_t)(rx_msg.data[2] | (rx_msg.data[3] << 8));
      int16_t pitch_centi = (int16_t)(rx_msg.data[4] | (rx_msg.data[5] << 8));

      float yaw   = yaw_centi   / 100.0f;
      float roll  = roll_centi  / 100.0f;
      float pitch = pitch_centi / 100.0f;

      Serial.print("[RX] yaw=");   Serial.print(yaw, 2);
      Serial.print(" roll=");      Serial.print(roll, 2);
      Serial.print(" pitch=");     Serial.print(pitch, 2);
      Serial.print("  (centi: ");
      Serial.print(yaw_centi); Serial.print(",");
      Serial.print(roll_centi); Serial.print(",");
      Serial.print(pitch_centi); Serial.println(")");
    } else {
      // 다른 ID나 다른 길이의 프레임이 온 경우
      Serial.print("[RX] id=0x");
      Serial.print(rx_msg.identifier, HEX);
      Serial.print(" dlc=");
      Serial.println(rx_msg.data_length_code);
    }
  }

  // 너무 바쁘지 않게
  delay(1);
}
