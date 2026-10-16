/* Complete main.c for STM32F103C8T6 + ICM-20608-G prototype.
 * Clock/I2C/UART/TIM values match the CubeMX main.c uploaded by the user.
 * Check the oscillator hardware before flashing: this assumes an 8 MHz HSE.
 */
#include "main.h"
#include "icm20608.h"
#include "step_tracker.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
void Error_Handler(void);

static volatile uint8_t sample_due = 0U;
static volatile uint32_t sample_overruns = 0U;
static StepTracker tracker;
static ICM20608_Data imu;
static float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;

static void uart_text(const char *s)
{
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 100U);
}

static int32_t f_to_i32(float value, float scale)
{
    float v = value * scale;
    if (v >= 0.0f) return (int32_t)(v + 0.5f);
    return (int32_t)(v - 0.5f);
}

static void send_step_record(void)
{
    char line[180];
    /* Fixed-point integers avoid requiring printf float support in the linker. */
    int32_t ax_mg = f_to_i32(imu.ax_g, 1000.0f);
    int32_t ay_mg = f_to_i32(imu.ay_g, 1000.0f);
    int32_t az_mg = f_to_i32(imu.az_g, 1000.0f);
    int32_t gx_cdeg = f_to_i32(imu.gx_dps, 100.0f);
    int32_t gy_cdeg = f_to_i32(imu.gy_dps, 100.0f);
    int32_t gz_cdeg = f_to_i32(imu.gz_dps, 100.0f);
    int32_t x_mm = f_to_i32(tracker.x_m, 1000.0f);
    int32_t y_mm = f_to_i32(tracker.y_m, 1000.0f);
    int32_t z_mm = f_to_i32(tracker.z_m, 1000.0f);
    int n = snprintf(line, sizeof(line),
        "STEP,%lu,%lu,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
        (unsigned long)tracker.observed_leg_events,
        (unsigned long)tracker.estimated_total_steps,
        (unsigned long)tracker.event_time_ms,
        (long)ax_mg, (long)ay_mg, (long)az_mg,
        (long)gx_cdeg, (long)gy_cdeg, (long)gz_cdeg,
        (long)x_mm, (long)y_mm, (long)z_mm);
    if (n > 0 && n < (int)sizeof(line)) {
        (void)HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)n, 100U);
    }
}

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_USART1_UART_Init();
    MX_TIM2_Init();

    uart_text("BOOT,STM32_STEP_MAP\r\n");
    if (ICM20608_Init(&hi2c1) != HAL_OK) {
        uart_text("ERROR,ICM20608_INIT_OR_WHO_AM_I\r\n");
        Error_Handler();
    }
    uart_text("READY,ICM20608G,ODR=100Hz\r\n");

    StepTracker_Init(&tracker);
    /* Keep the IMU stationary during startup. Timer is not running yet. */
    for (uint16_t i = 0U; i < 200U; i++) {
        if (ICM20608_ReadData(&hi2c1, &imu) != HAL_OK) {
            uart_text("ERROR,IMU_READ_DURING_STARTUP\r\n");
            Error_Handler();
        }
        gyro_bias_x += imu.gx_dps;
        gyro_bias_y += imu.gy_dps;
        gyro_bias_z += imu.gz_dps;
        HAL_Delay(10U);
    }
    gyro_bias_x /= 200.0f;
    gyro_bias_y /= 200.0f;
    gyro_bias_z /= 200.0f;
    uart_text("CALIBRATION_DONE,KEEP_SENSOR_STILL_DURING_STARTUP\r\n");

    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) Error_Handler();

    while (1) {
        if (sample_due != 0U) {
            sample_due = 0U;
            if (ICM20608_ReadData(&hi2c1, &imu) == HAL_OK) {
                imu.gx_dps -= gyro_bias_x;
                imu.gy_dps -= gyro_bias_y;
                imu.gz_dps -= gyro_bias_z;
                uint32_t now_ms = HAL_GetTick();
                StepTracker_Update(&tracker, &imu, now_ms);
                if (tracker.valid_event != 0U) send_step_record();
            } else {
                uart_text("ERROR,I2C_READ\r\n");
                HAL_Delay(2U);
            }
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        if (sample_due != 0U) sample_overruns++;
        sample_due = 1U;
    }
}

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
}

static void MX_TIM2_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 7199;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 99;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) Error_Handler();
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) Error_Handler();
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) Error_Handler();
}

static void MX_DMA_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
}

static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
}
#endif
