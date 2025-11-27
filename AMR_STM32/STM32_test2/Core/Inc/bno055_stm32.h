#ifndef BNO055_STM32_H
#define BNO055_STM32_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;         // 7-bit I2C address (0x28 or 0x29)
} BNO055_HandleTypeDef;

HAL_StatusTypeDef BNO055_Init(BNO055_HandleTypeDef *dev);
HAL_StatusTypeDef BNO055_ReadEuler(BNO055_HandleTypeDef *dev,
                                   float *heading, float *roll, float *pitch);

#ifdef __cplusplus
}
#endif

#endif /* BNO055_STM32_H */
