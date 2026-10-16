#ifndef ICM20608_H
#define ICM20608_H

#include "main.h"
#include <stdint.h>

#define ICM20608_I2C_ADDR       (0x68U << 1) /* HAL expects 8-bit shifted address */
#define ICM20608_WHO_AM_I_VALUE 0xAFU

typedef struct {
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;
    float ax_g, ay_g, az_g;
    float gx_dps, gy_dps, gz_dps;
} ICM20608_Data;

HAL_StatusTypeDef ICM20608_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef ICM20608_ReadData(I2C_HandleTypeDef *hi2c, ICM20608_Data *data);
HAL_StatusTypeDef ICM20608_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value);

#endif
