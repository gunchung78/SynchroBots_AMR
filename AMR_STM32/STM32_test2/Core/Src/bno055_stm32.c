#include "bno055_stm32.h"
#include <math.h>

// ===========================================
// Register Defines
// ===========================================
#define BNO055_CHIP_ID_ADDR          0x00
#define BNO055_CHIP_ID_VALUE         0xA0
#define BNO055_PAGE_ID_ADDR          0x07
#define BNO055_UNIT_SEL_ADDR         0x3B
#define BNO055_OPR_MODE_ADDR         0x3D
#define BNO055_PWR_MODE_ADDR         0x3E
#define BNO055_SYS_TRIGGER_ADDR      0x3F
#define BNO055_AXIS_MAP_CONFIG_ADDR  0x41
#define BNO055_AXIS_MAP_SIGN_ADDR    0x42
#define BNO055_SYS_STATUS_ADDR       0x39

// Euler angles
#define BNO055_EUL_DATA_ADDR         0x1A

// Quaternion (w, x, y, z)
#define BNO055_QUAT_DATA_ADDR        0x20

// Gyroscope
#define BNO055_GYRO_DATA_ADDR        0x14

// Linear acceleration
#define BNO055_LIA_DATA_ADDR         0x28

// Operation modes
#define BNO055_OPR_MODE_CONFIG       0x00
#define BNO055_OPR_MODE_NDOF         0x0C
#define BNO055_PWR_MODE_NORMAL       0x00

// ===========================================
// Low-level I2C helpers
// ===========================================

static HAL_StatusTypeDef BNO055_Write8(BNO055_HandleTypeDef *dev,
                                       uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(dev->hi2c, (dev->address << 1), reg,
            I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

static HAL_StatusTypeDef BNO055_Read(BNO055_HandleTypeDef *dev,
                                     uint8_t reg, uint8_t *pBuf, uint16_t len)
{
    return HAL_I2C_Mem_Read(dev->hi2c, (dev->address << 1), reg,
            I2C_MEMADD_SIZE_8BIT, pBuf, len, 100);
}

static HAL_StatusTypeDef BNO055_SetOprMode(BNO055_HandleTypeDef *dev, uint8_t mode)
{
    HAL_StatusTypeDef ret = BNO055_Write8(dev, BNO055_OPR_MODE_ADDR, mode);
    HAL_Delay(30);
    return ret;
}

// ===========================================
// Initialization
// ===========================================
HAL_StatusTypeDef BNO055_Init(BNO055_HandleTypeDef *dev)
{
    HAL_StatusTypeDef ret;
    uint8_t id = 0;

    HAL_Delay(700);

    ret = BNO055_Read(dev, BNO055_CHIP_ID_ADDR, &id, 1);
    if (ret != HAL_OK || id != BNO055_CHIP_ID_VALUE) return HAL_ERROR;

    ret = BNO055_SetOprMode(dev, BNO055_OPR_MODE_CONFIG);
    if (ret != HAL_OK) return ret;

    ret = BNO055_Write8(dev, BNO055_SYS_TRIGGER_ADDR, 0x20);
    if (ret != HAL_OK) return ret;

    HAL_Delay(700);

    ret = BNO055_SetOprMode(dev, BNO055_OPR_MODE_CONFIG);
    if (ret != HAL_OK) return ret;

    ret = BNO055_Write8(dev, BNO055_PAGE_ID_ADDR, 0x00);
    if (ret != HAL_OK) return ret;

    ret = BNO055_Write8(dev, BNO055_AXIS_MAP_CONFIG_ADDR, 0x24);
    if (ret != HAL_OK) return ret;

    ret = BNO055_Write8(dev, BNO055_AXIS_MAP_SIGN_ADDR, 0x00);
    if (ret != HAL_OK) return ret;

    ret = BNO055_Write8(dev, BNO055_PWR_MODE_ADDR, BNO055_PWR_MODE_NORMAL);
    if (ret != HAL_OK) return ret;

    ret = BNO055_SetOprMode(dev, BNO055_OPR_MODE_NDOF);
    if (ret != HAL_OK) return ret;

    uint8_t status = 0;
    uint32_t timeout = HAL_GetTick() + 1000;

    do {
        ret = BNO055_Read(dev, BNO055_SYS_STATUS_ADDR, &status, 1);
        if (ret != HAL_OK) return ret;
        if (status == 0x05) break;
    } while (HAL_GetTick() < timeout);

    return (status == 0x05) ? HAL_OK : HAL_TIMEOUT;
}

// ===========================================
// Euler Angle Reader
// ===========================================
HAL_StatusTypeDef BNO055_ReadEuler(BNO055_HandleTypeDef *dev,
                                   float *heading, float *roll, float *pitch)
{
    uint8_t buf[6];
    HAL_StatusTypeDef ret;

    ret = BNO055_Read(dev, BNO055_EUL_DATA_ADDR, buf, 6);
    if (ret != HAL_OK) return ret;

    *heading = (int16_t)((buf[1] << 8) | buf[0]) / 16.0f;
    *roll    = (int16_t)((buf[3] << 8) | buf[2]) / 16.0f;
    *pitch   = (int16_t)((buf[5] << 8) | buf[4]) / 16.0f;

    return HAL_OK;
}

// ===========================================
// QUATERNION Reader
// ===========================================
HAL_StatusTypeDef BNO055_ReadQuaternion(BNO055_HandleTypeDef *dev,
                                        float *qw, float *qx, float *qy, float *qz)
{
    uint8_t buf[8];
    HAL_StatusTypeDef ret = BNO055_Read(dev, BNO055_QUAT_DATA_ADDR, buf, 8);
    if (ret != HAL_OK) return ret;

    int16_t w = (buf[1] << 8) | buf[0];
    int16_t x = (buf[3] << 8) | buf[2];
    int16_t y = (buf[5] << 8) | buf[4];
    int16_t z = (buf[7] << 8) | buf[6];

    float scale = 1.0f / (1 << 14);

    *qw = w * scale;
    *qx = x * scale;
    *qy = y * scale;
    *qz = z * scale;

    return HAL_OK;
}

// ===========================================
// GYROSCOPE Reader (rad/s)
// ===========================================
HAL_StatusTypeDef BNO055_ReadGyro(BNO055_HandleTypeDef *dev,
                                  float *gx, float *gy, float *gz)
{
    uint8_t buf[6];
    HAL_StatusTypeDef ret = BNO055_Read(dev, BNO055_GYRO_DATA_ADDR, buf, 6);
    if (ret != HAL_OK) return ret;

    int16_t gx_raw = (buf[1] << 8) | buf[0];
    int16_t gy_raw = (buf[3] << 8) | buf[2];
    int16_t gz_raw = (buf[5] << 8) | buf[4];

    // deg/sec → rad/sec
    const float DEG2RAD = 3.14159265358979323846f / 180.0f;

    *gx = (gx_raw / 16.0f) * DEG2RAD;
    *gy = (gy_raw / 16.0f) * DEG2RAD;
    *gz = (gz_raw / 16.0f) * DEG2RAD;

    return HAL_OK;
}

// ===========================================
// LINEAR ACCELERATION Reader (m/s²)
// ===========================================
HAL_StatusTypeDef BNO055_ReadAcceleration(BNO055_HandleTypeDef *dev,
                                          float *ax, float *ay, float *az)
{
    uint8_t buf[6];
    HAL_StatusTypeDef ret = BNO055_Read(dev, BNO055_LIA_DATA_ADDR, buf, 6);
    if (ret != HAL_OK) return ret;

    int16_t ax_raw = (buf[1] << 8) | buf[0];
    int16_t ay_raw = (buf[3] << 8) | buf[2];
    int16_t az_raw = (buf[5] << 8) | buf[4];

    *ax = ax_raw * 0.00981f;
    *ay = ay_raw * 0.00981f;
    *az = az_raw * 0.00981f;

    return HAL_OK;
}


//#include "bno055_stm32.h"
//
//// BNO055 register addresses (page 0)
//#define BNO055_CHIP_ID_ADDR          0x00
//#define BNO055_CHIP_ID_VALUE         0xA0
//#define BNO055_PAGE_ID_ADDR          0x07
//#define BNO055_UNIT_SEL_ADDR         0x3B
//#define BNO055_OPR_MODE_ADDR         0x3D
//#define BNO055_PWR_MODE_ADDR         0x3E
//#define BNO055_SYS_TRIGGER_ADDR      0x3F
//#define BNO055_AXIS_MAP_CONFIG_ADDR  0x41
//#define BNO055_AXIS_MAP_SIGN_ADDR    0x42
//#define BNO055_SYS_STATUS_ADDR       0x39
//
//// Euler angle data starts at 0x1A: heading LSB, MSB, roll LSB, MSB, pitch LSB, MSB
//#define BNO055_EUL_DATA_ADDR         0x1A
//
//// Operation modes
//#define BNO055_OPR_MODE_CONFIG       0x00
//#define BNO055_OPR_MODE_NDOF         0x0C   // DFRobot에서 쓰는 NDOF 모드 값 기준
//
//// Power modes
//#define BNO055_PWR_MODE_NORMAL       0x00
//
//// Helper: write a single 8-bit register
//static HAL_StatusTypeDef BNO055_Write8(BNO055_HandleTypeDef *dev,
//                                       uint8_t reg, uint8_t value)
//{
//    return HAL_I2C_Mem_Write(dev->hi2c, (dev->address << 1), reg,
//                             I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
//}
//
//// Helper: read N bytes from a register
//static HAL_StatusTypeDef BNO055_Read(BNO055_HandleTypeDef *dev,
//                                     uint8_t reg, uint8_t *pBuf, uint16_t len)
//{
//    return HAL_I2C_Mem_Read(dev->hi2c, (dev->address << 1), reg,
//                            I2C_MEMADD_SIZE_8BIT, pBuf, len, 100);
//}
//
//// Helper: change operation mode
//static HAL_StatusTypeDef BNO055_SetOprMode(BNO055_HandleTypeDef *dev,
//                                           uint8_t mode)
//{
//    HAL_StatusTypeDef ret = BNO055_Write8(dev, BNO055_OPR_MODE_ADDR, mode);
//    HAL_Delay(30); // mode switch time
//    return ret;
//}
//
//HAL_StatusTypeDef BNO055_Init(BNO055_HandleTypeDef *dev)
//{
//    HAL_StatusTypeDef ret;
//    uint8_t id = 0;
//
//    // Power-on delay
//    HAL_Delay(700);
//
//    // Read CHIP_ID (0xA0 이어야 정상)
//    ret = BNO055_Read(dev, BNO055_CHIP_ID_ADDR, &id, 1);
//    if (ret != HAL_OK || id != BNO055_CHIP_ID_VALUE) {
//        return HAL_ERROR;
//    }
//
//    // Enter CONFIG mode
//    ret = BNO055_SetOprMode(dev, BNO055_OPR_MODE_CONFIG);
//    if (ret != HAL_OK) return ret;
//
//    // Reset sensor
//    ret = BNO055_Write8(dev, BNO055_SYS_TRIGGER_ADDR, 0x20); // RST_SYS = 1
//    if (ret != HAL_OK) return ret;
//    HAL_Delay(700);
//
//    // After reset, re-enter CONFIG mode
//    ret = BNO055_SetOprMode(dev, BNO055_OPR_MODE_CONFIG);
//    if (ret != HAL_OK) return ret;
//
//    // Page 0 선택
//    ret = BNO055_Write8(dev, BNO055_PAGE_ID_ADDR, 0x00);
//    if (ret != HAL_OK) return ret;
//
//    // (선택) Unit 설정 – 기본값이 각도(deg)이므로 생략 가능
//    // DFRobot 라이브러리도 Euler는 deg 기준으로 1 LSB = 1/16 deg 사용 :contentReference[oaicite:1]{index=1}
//
//    // (선택) 축 매핑: DFRobot eMapConfig_P1(0x24)와 동일하게 설정
//    ret = BNO055_Write8(dev, BNO055_AXIS_MAP_CONFIG_ADDR, 0x24);
//    if (ret != HAL_OK) return ret;
//    // Sign는 기본값 사용 (0x00) 또는 필요시 수정
//    ret = BNO055_Write8(dev, BNO055_AXIS_MAP_SIGN_ADDR, 0x00);
//    if (ret != HAL_OK) return ret;
//
//    // Power mode: Normal
//    ret = BNO055_Write8(dev, BNO055_PWR_MODE_ADDR, BNO055_PWR_MODE_NORMAL);
//    if (ret != HAL_OK) return ret;
//
//    // Fusion 모드: NDOF
//    ret = BNO055_SetOprMode(dev, BNO055_OPR_MODE_NDOF);
//    if (ret != HAL_OK) return ret;
//
//    // Fusion algorithm 이 동작할 때까지 대기 (SYS_STATUS == 0x05)
//    uint8_t status = 0;
//    uint32_t timeout = HAL_GetTick() + 1000;
//    do {
//        ret = BNO055_Read(dev, BNO055_SYS_STATUS_ADDR, &status, 1);
//        if (ret != HAL_OK) return ret;
//        if (status == 0x05) break;  // 0x05 = "Fusion algorithm running" 상태
//    } while (HAL_GetTick() < timeout);
//
//    return (status == 0x05) ? HAL_OK : HAL_TIMEOUT;
//}
//
//HAL_StatusTypeDef BNO055_ReadEuler(BNO055_HandleTypeDef *dev,
//                                   float *heading, float *roll, float *pitch)
//{
//    uint8_t buf[6];
//    HAL_StatusTypeDef ret;
//
//    ret = BNO055_Read(dev, BNO055_EUL_DATA_ADDR, buf, 6);
//    if (ret != HAL_OK) {
//        return ret;
//    }
//
//    int16_t raw_heading = (int16_t)((buf[1] << 8) | buf[0]);
//    int16_t raw_roll    = (int16_t)((buf[3] << 8) | buf[2]);
//    int16_t raw_pitch   = (int16_t)((buf[5] << 8) | buf[4]);
//
//    // DFRobot getEul()과 동일: 1 LSB = 1/16 deg 이므로 /16.0f :contentReference[oaicite:2]{index=2}
//    if (heading) *heading = ((float)raw_heading) / 16.0f;
//    if (roll)    *roll    = ((float)raw_roll)    / 16.0f;
//    if (pitch)   *pitch   = ((float)raw_pitch)   / 16.0f;
//
//    return HAL_OK;
//}
