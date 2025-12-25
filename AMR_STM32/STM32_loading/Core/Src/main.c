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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>

// VL53L0X ST API (your folder structure)
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
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
CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* =========================
 * User configurable defines
 * ========================= */

// ---- CAN IDs (example) ----
// STM32_main  -> STM32_loading : command
#define CAN_ID_CMD_STD        (0x201)

// Optional: STM32_loading -> STM32_main : status (if you want)
#define CAN_ID_STATUS_STD     (0x202)

// ---- VL53L0X I2C addresses ----
// ST API often uses 8-bit address (7-bit << 1).
// Default is commonly 0x52 (0x29<<1).
#define VL53L0X_ADDR_DEFAULT_8BIT   (0x52)
#define VL53L0X_ADDR_FRONT_8BIT     (0x30 << 1)   // 0x60
#define VL53L0X_ADDR_REAR_8BIT      (0x31 << 1)   // 0x62

// ---- Loader parameters (from your Arduino code) ----
static const int   DIST_THRESHOLD_CM        = 5;
static const float LOAD_PUSH_CM_12          = 6.0f;
static const float UNLOAD_PUSH_CM_3         = 5.5f;
static const float UNLOAD_PUSH_CM_2         = 8.7f;
static const float PUSH_DELTA_TOL_CM        = 0.3f;
static const uint32_t MAX_PUSH_TIME_MS      = 4000U;
static const uint32_t ACTION_COOLDOWN_MS    = 200U;
static const uint32_t LIMIT_DEBOUNCE_MS     = 20U;
static uint32_t boxDetectStartTick = 0;
static uint8_t isWaitingBox = 0;

// PWM speed (0~255 like Arduino)
static const uint8_t MOTOR_SPEED_FWD = 250;
static const uint8_t MOTOR_SPEED_REV = 180;

// ---- Active level ----
#define LIMIT_ACTIVE_STATE    (0)   // Active LOW

// ---- XSHUT pins (you named XSHUT as DIST pins) ----
#define XSHUT_FRONT_GPIO_Port   FRONT_DIST_GPIO_Port
#define XSHUT_FRONT_Pin         FRONT_DIST_Pin
#define XSHUT_REAR_GPIO_Port    REAR_DIST_GPIO_Port
#define XSHUT_REAR_Pin          REAR_DIST_Pin

/* =========================
 * State machine types
 * ========================= */
typedef enum {
  MODE_NONE      = 0,
  MODE_LOADING   = 1,
  MODE_UNLOADING = 2
} WorkMode;

typedef enum {
  ST_IDLE = 0,
  ST_HOME_BACK,
  ST_WAIT_READY,
  ST_MOVE_FWD,
  ST_MOVE_REV,
  ST_COMPLETE
} MotionState;

typedef enum {
  HC_IDLE    = 0,   // 0: go home(rear limit) & idle
  HC_LOADING = 1    // 1: auto loading
} HighCmdMode;

typedef enum {
  PUSH_STOP_NONE = 0,
  PUSH_STOP_DELTA,
  PUSH_STOP_FRONT_LIMIT
} PushStopMode;

/* =========================
 * Globals
 * ========================= */
static volatile uint8_t g_cmd_pending = 0;
static volatile uint8_t g_last_cmd = 0;

static CAN_RxHeaderTypeDef g_rxh;
static uint8_t g_rxData[8];

// VL53L0X device structs
static VL53L0X_Dev_t g_tof_front;
static VL53L0X_Dev_t g_tof_rear;

static uint8_t frontTofOk = 0;
static uint8_t rearTofOk  = 0;

static float frontDistCm = 999.0f;
static float rearDistCm  = 999.0f;

// box logic
static WorkMode    mode  = MODE_NONE;
static MotionState state = ST_IDLE;
static HighCmdMode highCmdMode = HC_LOADING;

static PushStopMode pushStopMode = PUSH_STOP_NONE;
static float targetDeltaCm = 0.0f;

static int currentCount = 0; // 0~3
static uint8_t boxesBeforeAction = 0;

static uint8_t errorFlag = 0;

static uint32_t lastActionMs   = 0;
static uint32_t lastLimitHitMs = 0;
static uint32_t moveFwdStartMs = 0;

static uint8_t rearPushValid = 0;
static float   rearPushStartCm = 0.0f;

// sensor update
static uint32_t lastSensorReadMs = 0;
static const uint32_t SENSOR_READ_INTERVAL_MS = 50U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
// ---- CAN helpers ----
static void CAN_FilterOnly_StdId_FIFO0(uint16_t stdId);
static void CAN_StartRxInterrupt(void);

// ---- Motor helpers ----
static void motorSetPwm1000(uint16_t pwmValue);
static void motorForward(void);
static void motorReverse(void);
static void motorStop(void);

// ---- Limit helpers ----
static uint8_t isFrontLimitActive(void);
static uint8_t isRearLimitActive(void);
static uint8_t limitDebounced(void);

// ---- VL53L0X helpers ----
static void ToF_PowerDownAll(void);
static void ToF_PowerUpFrontOnly(void);
static void ToF_PowerUpRearOnly(void);

static uint8_t ToF_InitOne(VL53L0X_Dev_t* dev, uint8_t addr8bit);
static uint8_t ToF_SetAddress(VL53L0X_Dev_t* dev, uint8_t newAddr8bit);
static float   ToF_ReadCmOnce(VL53L0X_Dev_t* dev);

static void initToFSensorsDual(void);
static void updateDistancesPeriodic(void);

// ---- Loader logic ----
static void setErrorSimple(void);
static void clearError(void);

static void configurePushProfile(void);
static void onActionCompleteUpdateCounts(void);

static void handleStateHomeBack(void);
static void handleStateWaitReady(void);
static void handleStateMoveFwd(void);
static void handleStateMoveRev(void);
static void handleStateComplete(void);

static void stepStateMachine(void);
static void applyCommand(uint8_t cmd);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  // Start PWM
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  motorStop();

  // CAN start + filter
  CAN_FilterOnly_StdId_FIFO0(CAN_ID_CMD_STD);
  CAN_StartRxInterrupt();

  // Dual VL53L0X init
  initToFSensorsDual();

  lastActionMs = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// (1) Apply CAN command if received
	if (g_cmd_pending) {
	  uint8_t cmd = g_last_cmd;
	  g_cmd_pending = 0;
	  applyCommand(cmd);
	}

	// (2) Safety if error
	if (errorFlag) {
	  motorStop();
	  HAL_Delay(10);
	  continue;
	}

	// (3) Update sensors
	updateDistancesPeriodic();

	// (4) Run state machine
	stepStateMachine();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, FRONT_DIST_Pin|REAR_DIST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LIMIT_FRONT_Pin LIMIT_REAR_Pin */
  GPIO_InitStruct.Pin = LIMIT_FRONT_Pin|LIMIT_REAR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MOTOR_DIR_Pin */
  GPIO_InitStruct.Pin = MOTOR_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MOTOR_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : FRONT_DIST_Pin REAR_DIST_Pin */
  GPIO_InitStruct.Pin = FRONT_DIST_Pin|REAR_DIST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* =========================
 * CAN
 * ========================= */

static void CAN_FilterOnly_StdId_FIFO0(uint16_t stdId)
{
  CAN_FilterTypeDef f = {0};

  f.FilterBank = 0;
  f.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  f.FilterActivation = ENABLE;

  // 16-bit scale, IDMASK
  f.FilterMode  = CAN_FILTERMODE_IDMASK;
  f.FilterScale = CAN_FILTERSCALE_16BIT;

  // StdId aligned at bit 5 in FilterIdHigh for 16-bit
  uint16_t id = (uint16_t)(stdId << 5);
  f.FilterIdHigh     = id;
  f.FilterIdLow      = 0;
  f.FilterMaskIdHigh = 0xFFE0; // match all 11-bit IDs exactly
  f.FilterMaskIdLow  = 0;

  if (HAL_CAN_ConfigFilter(&hcan, &f) != HAL_OK) {
    Error_Handler();
  }
}

static void CAN_StartRxInterrupt(void)
{
  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcanx)
{
  if (hcanx->Instance != CAN1) return;

  if (HAL_CAN_GetRxMessage(hcanx, CAN_RX_FIFO0, &g_rxh, g_rxData) != HAL_OK)
    return;

  if (g_rxh.IDE != CAN_ID_STD) return;
  if (g_rxh.StdId != CAN_ID_CMD_STD) return;
  if (g_rxh.DLC < 1) return;

  // Command in byte0 : 0/1/2 (same as Arduino UART command)
  g_last_cmd = g_rxData[0];
  g_cmd_pending = 1;
}

/* =========================
 * Motor / Limit
 * ========================= */

static void motorSetPwm1000(uint16_t pwmValue)
{
    // ARR이 999이므로 최대값은 1000입니다.
    if (pwmValue > 1000) pwmValue = 1000;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwmValue);
}

static void motorForward(void)
{
  HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_RESET);
  motorSetPwm1000(MOTOR_SPEED_FWD * 4);
}

static void motorReverse(void)
{
  HAL_GPIO_WritePin(MOTOR_DIR_GPIO_Port, MOTOR_DIR_Pin, GPIO_PIN_SET);
  motorSetPwm1000(1000 - (MOTOR_SPEED_REV * 4));
}

static void motorStop(void)
{
	motorSetPwm1000(0);
}

static uint8_t isFrontLimitActive(void)
{
  return (HAL_GPIO_ReadPin(LIMIT_FRONT_GPIO_Port, LIMIT_FRONT_Pin) == LIMIT_ACTIVE_STATE);
}

static uint8_t isRearLimitActive(void)
{
  return (HAL_GPIO_ReadPin(LIMIT_REAR_GPIO_Port, LIMIT_REAR_Pin) == LIMIT_ACTIVE_STATE);
}

static uint8_t limitDebounced(void)
{
  uint32_t now = HAL_GetTick();
  if ((now - lastLimitHitMs) < LIMIT_DEBOUNCE_MS) return 0;
  lastLimitHitMs = now;
  return 1;
}

/* =========================
 * VL53L0X (Dual)
 * ========================= */

static void ToF_PowerDownAll(void)
{
  HAL_GPIO_WritePin(XSHUT_FRONT_GPIO_Port, XSHUT_FRONT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(XSHUT_REAR_GPIO_Port,  XSHUT_REAR_Pin,  GPIO_PIN_RESET);
  HAL_Delay(10);
}

static void ToF_PowerUpFrontOnly(void)
{
  HAL_GPIO_WritePin(XSHUT_REAR_GPIO_Port,  XSHUT_REAR_Pin,  GPIO_PIN_RESET);
  HAL_GPIO_WritePin(XSHUT_FRONT_GPIO_Port, XSHUT_FRONT_Pin, GPIO_PIN_SET);
  HAL_Delay(10);
}

static void ToF_PowerUpRearOnly(void)
{
  HAL_GPIO_WritePin(XSHUT_FRONT_GPIO_Port, XSHUT_FRONT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(XSHUT_REAR_GPIO_Port,  XSHUT_REAR_Pin,  GPIO_PIN_SET);
  HAL_Delay(10);
}

static uint8_t ToF_InitOne(VL53L0X_Dev_t* dev, uint8_t addr8bit)
{
  VL53L0X_Error st;
  VL53L0X_DeviceInfo_t info;

  memset(dev, 0, sizeof(*dev));

  // This depends on your platform layer definition.
  // Common fields:
  dev->I2cDevAddr = addr8bit;

  st = VL53L0X_GetDeviceInfo(dev, &info);
  if (st != VL53L0X_ERROR_NONE) return 0;

  st = VL53L0X_DataInit(dev);
  if (st != VL53L0X_ERROR_NONE) return 0;

  st = VL53L0X_StaticInit(dev);
  if (st != VL53L0X_ERROR_NONE) return 0;

  // Basic calibration sequence (recommended by ST examples)
  uint8_t VhvSettings = 0;
  uint8_t PhaseCal = 0;
  st = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);
  if (st != VL53L0X_ERROR_NONE) return 0;

  uint32_t refSpadCount = 0;
  uint8_t isApertureSpads = 0;
  st = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);
  if (st != VL53L0X_ERROR_NONE) return 0;

  st = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
  if (st != VL53L0X_ERROR_NONE) return 0;

  // Optional timing budget
  (void)VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, 33000);

  return 1;
}

static uint8_t ToF_SetAddress(VL53L0X_Dev_t* dev, uint8_t newAddr8bit)
{
  VL53L0X_Error st = VL53L0X_SetDeviceAddress(dev, newAddr8bit);
  if (st != VL53L0X_ERROR_NONE) return 0;

  // Update local handle
  dev->I2cDevAddr = newAddr8bit;
  return 1;
}

static float ToF_ReadCmOnce(VL53L0X_Dev_t* dev)
{
  VL53L0X_RangingMeasurementData_t m;
  VL53L0X_Error st = VL53L0X_PerformSingleRangingMeasurement(dev, &m);
  if (st != VL53L0X_ERROR_NONE) return 999.0f;

  // RangeStatus: 0 means valid (most commonly)
  if (m.RangeStatus != 0) return 999.0f;

  return ((float)m.RangeMilliMeter) / 10.0f;
}

static void initToFSensorsDual(void)
{
  frontTofOk = 0;
  rearTofOk  = 0;

  ToF_PowerDownAll();

  // (1) Front only -> init at default -> set new address 0x30
  ToF_PowerUpFrontOnly();
  frontTofOk = ToF_InitOne(&g_tof_front, VL53L0X_ADDR_DEFAULT_8BIT);
  if (frontTofOk) {
    frontTofOk = ToF_SetAddress(&g_tof_front, VL53L0X_ADDR_FRONT_8BIT);
  }

  // (2) Rear only -> init at default -> set new address 0x31
  ToF_PowerUpRearOnly();
  rearTofOk = ToF_InitOne(&g_tof_rear, VL53L0X_ADDR_DEFAULT_8BIT);
  if (rearTofOk) {
    rearTofOk = ToF_SetAddress(&g_tof_rear, VL53L0X_ADDR_REAR_8BIT);
  }

  // (3) Power both ON
  HAL_GPIO_WritePin(XSHUT_FRONT_GPIO_Port, XSHUT_FRONT_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(XSHUT_REAR_GPIO_Port,  XSHUT_REAR_Pin,  GPIO_PIN_SET);
  HAL_Delay(10);

  // Prime first read
  frontDistCm = frontTofOk ? ToF_ReadCmOnce(&g_tof_front) : 999.0f;
  rearDistCm  = rearTofOk  ? ToF_ReadCmOnce(&g_tof_rear)  : 999.0f;
}

static void updateDistancesPeriodic(void)
{
  uint32_t now = HAL_GetTick();
  if ((now - lastSensorReadMs) < SENSOR_READ_INTERVAL_MS) return;
  lastSensorReadMs = now;

  if (frontTofOk) frontDistCm = ToF_ReadCmOnce(&g_tof_front);
  else            frontDistCm = 999.0f;

  if (rearTofOk)  rearDistCm  = ToF_ReadCmOnce(&g_tof_rear);
  else            rearDistCm  = 999.0f;
}

/* =========================
 * Loader logic (ported)
 * ========================= */

static void setErrorSimple(void)
{
  errorFlag = 1;
  motorStop();
  state = ST_IDLE;
  mode = MODE_NONE;
  highCmdMode = HC_IDLE;
}

static void clearError(void)
{
  errorFlag = 0;
}

static void configurePushProfile(void)
{
  boxesBeforeAction = (uint8_t)currentCount;
  pushStopMode      = PUSH_STOP_NONE;
  targetDeltaCm     = 0.0f;

  if (mode == MODE_LOADING) {
    if (boxesBeforeAction == 0 || boxesBeforeAction == 1) {
      pushStopMode  = PUSH_STOP_DELTA;
      targetDeltaCm = LOAD_PUSH_CM_12;
    } else if (boxesBeforeAction == 2) {
      pushStopMode  = PUSH_STOP_NONE;
      targetDeltaCm = 0.0f;
    } else {
      setErrorSimple();
      return;
    }
  } else if (mode == MODE_UNLOADING) {
    if (boxesBeforeAction >= 3) {
      pushStopMode  = PUSH_STOP_DELTA;
      targetDeltaCm = UNLOAD_PUSH_CM_3;
    } else if (boxesBeforeAction == 2) {
      pushStopMode  = PUSH_STOP_DELTA;
      targetDeltaCm = UNLOAD_PUSH_CM_2;
    } else if (boxesBeforeAction == 1) {
      pushStopMode  = PUSH_STOP_FRONT_LIMIT;
      targetDeltaCm = 0.0f;
    } else {
      setErrorSimple();
      return;
    }
  }
}

static void onActionCompleteUpdateCounts(void)
{
  if (mode == MODE_LOADING) {
    currentCount++;
    if (currentCount > 3) currentCount = 3;
    mode = MODE_NONE;
    state = ST_IDLE;
    return;
  }
  if (mode == MODE_UNLOADING) {
    currentCount--;
    if (currentCount < 0) currentCount = 0;
    mode = MODE_NONE;
    state = ST_IDLE;
    return;
  }
  state = ST_IDLE;
}

static void handleStateHomeBack(void)
{
  if (isRearLimitActive()) {
    motorStop();
    state = ST_WAIT_READY;
    lastActionMs = HAL_GetTick();
    return;
  }
  motorReverse();
}

static void handleStateWaitReady(void)
{
  // must be at rear limit
  if (!isRearLimitActive()) {
    state = ST_HOME_BACK;
    lastActionMs = HAL_GetTick();
    return;
  }

  // LOADING: wait for box detection
  if (mode == MODE_LOADING) {
    if (frontDistCm >= (float)DIST_THRESHOLD_CM) {
	  isWaitingBox = 0;
      return;
    }

    if (!isWaitingBox) {
            boxDetectStartTick = HAL_GetTick();
            isWaitingBox = 1;
	}

    if (HAL_GetTick() - boxDetectStartTick < 700) {
            return; // 700ms가 지날 때까지 리턴 (루프는 계속 돔)
	}
    isWaitingBox = 0;
  }

  configurePushProfile();
  if (errorFlag) return;

  // LOADING 3rd box: no forward push
  if (mode == MODE_LOADING && boxesBeforeAction == 2) {
    state = ST_COMPLETE;
    lastActionMs = HAL_GetTick();
    return;
  }

  // rear delta baseline
  if (rearTofOk) {
    rearPushValid   = 1;
    rearPushStartCm = rearDistCm;
  } else {
    rearPushValid = 0;
  }

  motorForward();
  moveFwdStartMs = HAL_GetTick();
  state = ST_MOVE_FWD;
  lastActionMs = HAL_GetTick();
}

static void handleStateMoveFwd(void)
{
  uint32_t now = HAL_GetTick();

  if ((now - moveFwdStartMs) > MAX_PUSH_TIME_MS) {
    setErrorSimple();
    return;
  }

  // safety: front limit
  if (isFrontLimitActive() && limitDebounced()) {
    motorStop();
    HAL_Delay(1000);
    motorReverse();
    state = ST_MOVE_REV;
    lastActionMs = now;
    return;
  }

  if (pushStopMode == PUSH_STOP_DELTA && rearPushValid) {
    float delta = fabsf(rearDistCm - rearPushStartCm);
    if ((delta + PUSH_DELTA_TOL_CM) >= targetDeltaCm) {
      motorStop();
      HAL_Delay(1000);
      motorReverse();
      state = ST_MOVE_REV;
      lastActionMs = now;
      return;
    }
  }

  motorForward();
}

static void handleStateMoveRev(void)
{
  if (isRearLimitActive() && limitDebounced()) {
    motorStop();
    state = ST_COMPLETE;
    lastActionMs = HAL_GetTick();
    return;
  }
  motorReverse();
}

static void handleStateComplete(void)
{
  onActionCompleteUpdateCounts();
  rearPushValid = 0;
  lastActionMs = HAL_GetTick();
}

static void stepStateMachine(void)
{
  uint32_t now = HAL_GetTick();
  if ((now - lastActionMs) < ACTION_COOLDOWN_MS) return;

  // (A) HC_IDLE: always go home to rear limit
  if (mode == MODE_NONE && highCmdMode == HC_IDLE) {
    if (!isRearLimitActive()) motorReverse();
    else motorStop();
    return;
  }

  // (B) idle + HC_LOADING -> auto start loading once
  if (mode == MODE_NONE && state == ST_IDLE) {
    if (highCmdMode == HC_LOADING && currentCount < 3 && isRearLimitActive()) {
      if (frontDistCm < (float)DIST_THRESHOLD_CM) {
        mode = MODE_LOADING;
        state = ST_WAIT_READY;
      }
    }
  }

  // no action -> stop motor
  if (mode == MODE_NONE) {
    motorStop();
    return;
  }

  switch (state) {
    case ST_HOME_BACK:  handleStateHomeBack();   break;
    case ST_WAIT_READY: handleStateWaitReady();  break;
    case ST_MOVE_FWD:   handleStateMoveFwd();    break;
    case ST_MOVE_REV:   handleStateMoveRev();    break;
    case ST_COMPLETE:   handleStateComplete();   break;
    case ST_IDLE:
    default:
      state = isRearLimitActive() ? ST_WAIT_READY : ST_HOME_BACK;
      break;
  }
}

/* =========================
 * Command handling
 * ========================= */

static void applyCommand(uint8_t cmd)
{
  // cmd: 0/1/2 are same meaning with Arduino UART
  if (cmd == 0) {
    highCmdMode = HC_IDLE;
    // do not force reset mode/state; let state machine home it
    return;
  }

  if (cmd == 1) {
    highCmdMode = HC_LOADING;
    return;
  }

  if (cmd == 2) {
    // unload once
    if (mode != MODE_NONE || state != ST_IDLE) return;
    if (currentCount <= 0) return;

    mode = MODE_UNLOADING;
    rearPushValid = 0;
    state = isRearLimitActive() ? ST_WAIT_READY : ST_HOME_BACK;
    return;
  }

  // optional maintenance commands
  if (cmd == 3) { // reset all
    clearError();
    mode = MODE_NONE;
    state = ST_IDLE;
    currentCount = 0;
    rearPushValid = 0;
    highCmdMode = HC_IDLE;
    motorStop();
    return;
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
