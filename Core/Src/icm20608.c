#include "icm20608.h"

#define REG_SMPLRT_DIV      0x19U
#define REG_CONFIG          0x1AU
#define REG_GYRO_CONFIG     0x1BU
#define REG_ACCEL_CONFIG    0x1CU
#define REG_ACCEL_CONFIG2   0x1DU
#define REG_INT_PIN_CFG     0x37U
#define REG_INT_ENABLE      0x38U
#define REG_ACCEL_XOUT_H    0x3BU
#define REG_USER_CTRL       0x6AU
#define REG_PWR_MGMT_1      0x6BU
#define REG_PWR_MGMT_2      0x6CU
#define REG_WHO_AM_I        0x75U

static HAL_StatusTypeDef write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(hi2c, ICM20608_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT, &value, 1U, 100U);
}

HAL_StatusTypeDef ICM20608_ReadRegister(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *value)
{
    return HAL_I2C_Mem_Read(hi2c, ICM20608_I2C_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, value, 1U, 100U);
}

HAL_StatusTypeDef ICM20608_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t who = 0U;
    HAL_Delay(100U); /* device startup time */

    if (ICM20608_ReadRegister(hi2c, REG_WHO_AM_I, &who) != HAL_OK) return HAL_ERROR;
    if (who != ICM20608_WHO_AM_I_VALUE) return HAL_ERROR;

    /* Device reset; allow registers and clock to settle. */
    if (write_reg(hi2c, REG_PWR_MGMT_1, 0x80U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(100U);

    /* Wake up; use X gyro PLL clock. */
    if (write_reg(hi2c, REG_PWR_MGMT_1, 0x01U) != HAL_OK) return HAL_ERROR;
    if (write_reg(hi2c, REG_PWR_MGMT_2, 0x00U) != HAL_OK) return HAL_ERROR;
    HAL_Delay(10U);

    /* DLPF enabled: gyro output base rate 1 kHz; sample divider 9 => 100 Hz. */
    if (write_reg(hi2c, REG_CONFIG, 0x03U) != HAL_OK) return HAL_ERROR;
    if (write_reg(hi2c, REG_SMPLRT_DIV, 9U) != HAL_OK) return HAL_ERROR;

    /* Gyroscope FS_SEL=1: +/-500 dps. */
    if (write_reg(hi2c, REG_GYRO_CONFIG, 0x08U) != HAL_OK) return HAL_ERROR;
    /* Accelerometer ACCEL_FS_SEL=1: +/-4 g. */
    if (write_reg(hi2c, REG_ACCEL_CONFIG, 0x08U) != HAL_OK) return HAL_ERROR;
    /* Accelerometer low-pass filter enabled, A_DLPF_CFG=3. */
    if (write_reg(hi2c, REG_ACCEL_CONFIG2, 0x03U) != HAL_OK) return HAL_ERROR;

    /* Disable interrupts/FIFO for this polling-based prototype. */
    if (write_reg(hi2c, REG_INT_ENABLE, 0x00U) != HAL_OK) return HAL_ERROR;
    if (write_reg(hi2c, REG_USER_CTRL, 0x00U) != HAL_OK) return HAL_ERROR;
    (void)REG_INT_PIN_CFG;
    HAL_Delay(20U);
    return HAL_OK;
}

static int16_t make_i16(uint8_t hi, uint8_t lo)
{
    return (int16_t)((uint16_t)((uint16_t)hi << 8) | (uint16_t)lo);
}

HAL_StatusTypeDef ICM20608_ReadData(I2C_HandleTypeDef *hi2c, ICM20608_Data *data)
{
    uint8_t b[14];
    if (HAL_I2C_Mem_Read(hi2c, ICM20608_I2C_ADDR, REG_ACCEL_XOUT_H,
                         I2C_MEMADD_SIZE_8BIT, b, sizeof(b), 100U) != HAL_OK) {
        return HAL_ERROR;
    }

    data->ax_raw = make_i16(b[0], b[1]);
    data->ay_raw = make_i16(b[2], b[3]);
    data->az_raw = make_i16(b[4], b[5]);
    /* b[6..7] is temperature; intentionally not used. */
    data->gx_raw = make_i16(b[8], b[9]);
    data->gy_raw = make_i16(b[10], b[11]);
    data->gz_raw = make_i16(b[12], b[13]);

    /* +/-4 g = 8192 LSB/g; +/-500 dps = 65.5 LSB/(degree/second). */
    data->ax_g = (float)data->ax_raw / 8192.0f;
    data->ay_g = (float)data->ay_raw / 8192.0f;
    data->az_g = (float)data->az_raw / 8192.0f;
    data->gx_dps = (float)data->gx_raw / 65.5f;
    data->gy_dps = (float)data->gy_raw / 65.5f;
    data->gz_dps = (float)data->gz_raw / 65.5f;
    return HAL_OK;
}
