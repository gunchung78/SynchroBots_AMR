#include <Arduino.h>
#include "driver/twai.h"

// === 노드마다 이 값만 다르게 설정 ===
// 1번 보드: 1
// 2번 보드: 2
// 3번 보드: 3
#define NODE_ID 1

// CAN TX/RX 핀 (보드에 맞게 변경 가능)
#define CAN_TX_PIN 21
#define CAN_RX_PIN 22

// CAN 속도 (500 kbps 예시)
#define CAN_SPEED_500K  500E3

void setup()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.printf("[NODE %d] Booting...\n", NODE_ID);

  // TWAI(CAN) 설정
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
      CAN_TX_PIN,
      CAN_RX_PIN,
      TWAI_MODE_NORMAL);         // 일반 모드

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS(); // 500kbps
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL(); // 모든 ID 수신

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("twai_driver_install failed");
    while (1) delay(1000);
  }

  if (twai_start() != ESP_OK) {
    Serial.println("twai_start failed");
    while (1) delay(1000);
  }

  Serial.printf("[NODE %d] TWAI started. Listening on bus...\n", NODE_ID);
}

unsigned long last_tx_ms = 0;
uint8_t tx_counter = 0;

void loop()
{
  // 1) 주기적으로 메시지 송신 (500ms마다)
  if (millis() - last_tx_ms >= 500) {
    last_tx_ms = millis();

    twai_message_t tx_msg = {0};
    tx_msg.identifier = 0x100 + NODE_ID;   // 노드별 다른 ID
    tx_msg.extd = 0;                       // 표준 ID
    tx_msg.rtr = 0;                        // 데이터 프레임
    tx_msg.data_length_code = 2;           // 데이터 2바이트 사용

    tx_msg.data[0] = NODE_ID;             // 첫 바이트: 송신자 ID
    tx_msg.data[1] = tx_counter++;        // 두 번째: 카운터

    esp_err_t res = twai_transmit(&tx_msg, pdMS_TO_TICKS(10));
    if (res == ESP_OK) {
      Serial.printf("[NODE %d] TX: ID=0x%03X data=[%u,%u]\n",
                    NODE_ID,
                    tx_msg.identifier,
                    tx_msg.data[0],
                    tx_msg.data[1]);
    } else {
      Serial.printf("[NODE %d] TX ERROR: %d\n", NODE_ID, (int)res);
    }
  }

  // 2) 수신 처리 (논블로킹)
  twai_message_t rx_msg;
  esp_err_t rx_res = twai_receive(&rx_msg, 0); // timeout=0 → 즉시 리턴

  if (rx_res == ESP_OK) {
    Serial.printf("[NODE %d] RX: ID=0x%03X DLC=%d data:",
                  NODE_ID,
                  rx_msg.identifier,
                  rx_msg.data_length_code);

    for (int i = 0; i < rx_msg.data_length_code; i++) {
      Serial.printf(" %u", rx_msg.data[i]);
    }
    Serial.println();
  }

  // CPU 너무 안 바쁘게
  delay(1);
}
