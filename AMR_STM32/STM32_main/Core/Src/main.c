/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static void IMU_SendData(float heading, float roll, float pitch);
static void CAN1_FilterOnly_StdId_FIFO0(uint16_t stdId);
static void CAN1_StartRxInterrupt(void);

static void CAN1_TxInit_LoadingCmd(void);
static void CAN1_SendLoadingCmd(uint8_t cmd, uint8_t arg);

static void UART3_StartRxIT(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static CAN_RxHeaderTypeDef rxh;
static uint8_t rxData[8];

volatile float g_roll_deg  = 0.0f;
volatile float g_pitch_deg = 0.0f;
volatile float g_yaw_deg   = 0.0f;
volatile uint8_t g_imu_new = 0;

volatile uint8_t  g_seq = 0;
volatile uint8_t  g_status = 0;
volatile uint32_t g_rx_count = 0;
volatile uint32_t g_seq_miss_count = 0;
volatile uint32_t g_last_rx_ms = 0;

/* UART2 출력 버퍼 */
static char uart_tx_buffer[96];

/* ------------------------- UART3 RX (Raspi -> main) ------------------------- */
static uint8_t  u3_rx_byte;
static char     u3_line_buf[64];
static uint16_t u3_line_len = 0;
volatile uint8_t g_u3_line_ready = 0;
static char     g_u3_line_copy[64];   // main loop에서 안전하게 쓰기 위한 복사본

/* ------------------------- CAN TX (to STM32_loading) ------------------------- */
static CAN_TxHeaderTypeDef txh_load;
static uint8_t txLoadData[8];
static uint32_t txMailbox;

/* ------------------------- Utility: IMU UART Output ------------------------- */
static void IMU_SendData(float heading, float roll, float pitch)
{
    // Raspi에서 파싱하기 쉬운 포맷
    // 예: "H:123.45,R:1.23,P:-4.56\r\n"
    int len = snprintf(uart_tx_buffer, sizeof(uart_tx_buffer),
                       "H:%.2f,R:%.2f,P:%.2f\r\n",
                       heading, roll, pitch);

    if (len <= 0) return;
    if (len > (int)sizeof(uart_tx_buffer)) len = sizeof(uart_tx_buffer);

    // main loop에서만 호출(인터럽트에서 호출하지 않음)
    (void)HAL_UART_Transmit(&huart2, (uint8_t*)uart_tx_buffer, (uint16_t)len, 50);
}

/* ------------------------- CAN Filter: only StdId -> FIFO0 ------------------------- */
static void CAN1_FilterOnly_StdId_FIFO0(uint16_t stdId)
{
    CAN_FilterTypeDef f = {0};

    f.FilterBank = 0;
    f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    f.FilterActivation = ENABLE;

    // 16-bit IDLIST: 동일 ID를 4슬롯에 넣어 사실상 1개 ID만 허용
    f.FilterMode  = CAN_FILTERMODE_IDLIST;
    f.FilterScale = CAN_FILTERSCALE_16BIT;

    uint16_t id = (uint16_t)(stdId << 5);
    f.FilterIdHigh      = id;
    f.FilterIdLow       = id;
    f.FilterMaskIdHigh  = id;
    f.FilterMaskIdLow   = id;

    f.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) Error_Handler();
}

static void CAN1_StartRxInterrupt(void)
{
    if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();

    // RX FIFO0 pending interrupt enable (callback 패턴)
    // 이 패턴이 HAL에서 흔히 쓰는 방식임 :contentReference[oaicite:2]{index=2}
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        Error_Handler();
}

/* ------------------------- CAN TX to loading board ------------------------- */
static void CAN1_TxInit_LoadingCmd(void)
{
    txh_load.StdId = 0x201;      // <-- loading command ID (원하면 변경)
    txh_load.IDE   = CAN_ID_STD;
    txh_load.RTR   = CAN_RTR_DATA;
    txh_load.DLC   = 2;

    memset(txLoadData, 0, sizeof(txLoadData));
}

static void CAN1_SendLoadingCmd(uint8_t cmd, uint8_t arg)
{
    txLoadData[0] = cmd;
    txLoadData[1] = arg;

    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {
        // TX 밀림 표시(선택): LD5 토글
        HAL_GPIO_TogglePin(GPIOD, LD5_Pin);
        return;
    }

    if (HAL_CAN_AddTxMessage(&hcan1, &txh_load, txLoadData, &txMailbox) != HAL_OK) {
        // TX 실패 표시(선택): LD5 ON
        HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
    }
}

/* ------------------------- UART3 RX Start ------------------------- */
static void UART3_StartRxIT(void)
{
    u3_line_len = 0;
    g_u3_line_ready = 0;
    memset(u3_line_buf, 0, sizeof(u3_line_buf));
    memset(g_u3_line_copy, 0, sizeof(g_u3_line_copy));

    (void)HAL_UART_Receive_IT(&huart3, &u3_rx_byte, 1);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_USB_HOST_Init();
  MX_CAN1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  // 1) CAN 수신(IMU) 준비
  CAN1_FilterOnly_StdId_FIFO0(0x103);
  CAN1_StartRxInterrupt();

  // 2) CAN 송신(loading cmd) 준비
  CAN1_TxInit_LoadingCmd();

  // 3) UART3 수신(라즈베리파이 명령) 시작
  UART3_StartRxIT();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    // --- 에러 복구 및 강제 재시작 코드 ---
	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE) || __HAL_UART_GET_FLAG(&huart3, UART_FLAG_NE)) {
		__HAL_UART_CLEAR_OREFLAG(&huart3);
		__HAL_UART_CLEAR_NEFLAG(&huart3);
		HAL_UART_Receive_IT(&huart3, &u3_rx_byte, 1);
		HAL_GPIO_TogglePin(GPIOD, LD5_Pin); // 에러 발생 시 빨간불 토글
	}

	// 수신 상태가 준비 완료(Ready)인데 인터럽트가 안 걸려 있다면 다시 걸어줌
	if (huart3.RxState == HAL_UART_STATE_READY) {
		HAL_UART_Receive_IT(&huart3, &u3_rx_byte, 1);
	}
	// ----------------------------------
    /* (A) IMU 데이터 출력 */
	if (g_imu_new) {
		g_imu_new = 0;
		IMU_SendData(g_yaw_deg, g_roll_deg, g_pitch_deg);
	}

	/* (B) 라즈베리파이 명령 처리 및 전송 */
	if (g_u3_line_ready) {
		g_u3_line_ready = 0;
		unsigned int cmd = 999;
		if (sscanf(g_u3_line_copy, "%u", &cmd) == 1) {
			if (cmd <= 2) { // 0, 1, 2 명령 처리
				CAN1_SendLoadingCmd((uint8_t)cmd, 0);
				HAL_GPIO_TogglePin(GPIOD, LD4_Pin); // 성공 시 녹색 토글
			}
		}
	}

	/* (C) 상태 체크 LED (빨간불 관리) */
	static uint32_t t_prev = 0;
	uint32_t now = HAL_GetTick();
	if ((now - t_prev) >= 200) {
		t_prev = now;
		if ((now - g_last_rx_ms) > 300) {
			HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET); // IMU 없으면 빨간불
		} else {
			HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET);
		}
	}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_96K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin|SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* ------------------------- CAN RX Callback ------------------------- */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxh, rxData) != HAL_OK)
        return;

    if (rxh.IDE != CAN_ID_STD) return;
    if (rxh.StdId != 0x103) return;
    if (rxh.DLC < 8) return;

    // [MSB,LSB] 조립
    int16_t roll_cdeg  = (int16_t)((rxData[0] << 8) | rxData[1]);
    int16_t pitch_cdeg = (int16_t)((rxData[2] << 8) | rxData[3]);
    int16_t yaw_cdeg   = (int16_t)((rxData[4] << 8) | rxData[5]);

    g_roll_deg  = roll_cdeg  / 100.0f;
    g_pitch_deg = pitch_cdeg / 100.0f;
    g_yaw_deg   = yaw_cdeg   / 100.0f;

    uint8_t seq    = rxData[6];
    uint8_t status = rxData[7];

    g_seq = seq;
    g_status = status;
    g_rx_count++;
    g_last_rx_ms = HAL_GetTick();

    static uint8_t last_seq = 0;
    uint8_t expected = (uint8_t)(last_seq + 1);
    if (g_rx_count > 1 && seq != expected) {
        g_seq_miss_count++;
    }
    last_seq = seq;

    // ISR에서는 플래그만 세움 (UART 전송은 main loop에서)
    g_imu_new = 1;

    // (선택) 수신 살아있음 표시: 100프레임마다 블루 토글
    if ((seq % 100) == 0) {
        HAL_GPIO_TogglePin(GPIOD, LD6_Pin);
    }
}

/* ------------------------- UART RX Complete Callback ------------------------- */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
    	HAL_GPIO_TogglePin(GPIOD, LD3_Pin);
        // 1바이트 수신 -> 라인 버퍼에 누적
        char c = (char)u3_rx_byte;

        if (c == '\r') {
            // ignore
        } else if (c == '\n') {
            // 라인 완성
            if (u3_line_len > 0) {
                u3_line_buf[u3_line_len] = '\0';
                // main loop에서 쓸 복사본 생성
                strncpy(g_u3_line_copy, u3_line_buf, sizeof(g_u3_line_copy) - 1);
                g_u3_line_copy[sizeof(g_u3_line_copy) - 1] = '\0';
                g_u3_line_ready = 1;
            }
            // 버퍼 리셋
            u3_line_len = 0;
            memset(u3_line_buf, 0, sizeof(u3_line_buf));
        } else {
            if (u3_line_len < (sizeof(u3_line_buf) - 1)) {
                u3_line_buf[u3_line_len++] = c;
            } else {
                // overflow -> 리셋
                u3_line_len = 0;
                memset(u3_line_buf, 0, sizeof(u3_line_buf));
            }
        }

        // 다음 바이트 수신 재개
        // (UART RX 콜백은 인터럽트 컨텍스트이므로 오래 걸리면 안 됨)
        // 블로킹 transmit 같은 건 피하는 게 좋음 :contentReference[oaicite:3]{index=3}
        (void)HAL_UART_Receive_IT(&huart3, &u3_rx_byte, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
