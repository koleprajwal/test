/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include <math.h>
/* All IMU and step-detector code is kept in this main.c. */
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
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
#define ICM_ADDR             (0x68u << 1) /* Change to (0x69u << 1) if AD0 is high */
#define ICM_WHO_AM_I_VALUE   0xAFu
#define STEP_LP_ALPHA        0.22f
#define STEP_PEAK_THRESHOLD  0.18f
#define STEP_REARM_LEVEL     0.07f
#define STEP_MIN_INTERVAL_MS 280u
#define CALIBRATION_SAMPLES  400u
#define SAMPLE_PERIOD_MS     20u

typedef struct {
  float ax_g, ay_g, az_g;
  float gx_dps, gy_dps, gz_dps;
} ICM20608_Data;

typedef struct {
  float filtered_mag;
  float previous_dynamic;
  uint32_t last_step_ms;
  uint32_t count;
  uint8_t armed;
} StepDetector;

static volatile uint8_t sample_due = 0;
static volatile uint8_t uart_busy = 0;
static volatile uint32_t missed_samples = 0;
static char tx_line[160];
static ICM20608_Data imu;
static StepDetector steps;
static float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
static uint32_t sample_time_ms = 0;
static volatile uint8_t sensor_ok = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef ICM_Write(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef ICM_Read(uint8_t reg, uint8_t *data, uint16_t length);
static HAL_StatusTypeDef ICM_Init(void);
static HAL_StatusTypeDef ICM_ReadSample(ICM20608_Data *out);
static void Calibrate_Gyro(void);
static void StepDetector_Init(void);
static void StepDetector_Update(float ax, float ay, float az, uint32_t now_ms);
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Keep the IMU stationary throughout the ~8-second gyro calibration. */
  if (ICM_Init() != HAL_OK) Error_Handler();
  Calibrate_Gyro();
  StepDetector_Init();
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK) Error_Handler();
  sensor_ok = 1;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (sample_due && sensor_ok) {
      /* Atomically consume the flag; no I2C or UART work in the ISR. */
      __disable_irq();
      sample_due = 0;
      __enable_irq();

      if (ICM_ReadSample(&imu) == HAL_OK) {
        sample_time_ms = HAL_GetTick();
        imu.gx_dps -= gyro_bias_x;
        imu.gy_dps -= gyro_bias_y;
        imu.gz_dps -= gyro_bias_z;
        StepDetector_Update(imu.ax_g, imu.ay_g, imu.az_g, sample_time_ms);

        if (!uart_busy) {
          int n = snprintf(tx_line, sizeof(tx_line),
            "%lu,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%lu\r\n",
            (unsigned long)sample_time_ms,
            (double)imu.ax_g, (double)imu.ay_g, (double)imu.az_g,
            (double)imu.gx_dps, (double)imu.gy_dps, (double)imu.gz_dps,
            (unsigned long)steps.count);
          if (n > 0 && n < (int)sizeof(tx_line)) {
            uart_busy = 1;
            if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)tx_line, (uint16_t)n) != HAL_OK) {
              uart_busy = 0;
            }
          }
        }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x00201D2B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 199;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ICM-20608-G register-level driver, kept in this file. */
#define REG_SMPLRT_DIV 0x19u
#define REG_CONFIG     0x1Au
#define REG_GYRO_CFG   0x1Bu
#define REG_ACCEL_CFG  0x1Cu
#define REG_ACCEL_CFG2 0x1Du
#define REG_ACCEL_XOUT 0x3Bu
#define REG_PWR_MGMT_1 0x6Bu
#define REG_PWR_MGMT_2 0x6Cu
#define REG_WHO_AM_I   0x75u

static HAL_StatusTypeDef ICM_Write(uint8_t reg, uint8_t value) {
  return HAL_I2C_Mem_Write(&hi2c1, ICM_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1u, 50u);
}
static HAL_StatusTypeDef ICM_Read(uint8_t reg, uint8_t *data, uint16_t length) {
  return HAL_I2C_Mem_Read(&hi2c1, ICM_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, 50u);
}
static int16_t ICM_Signed16(uint8_t hi, uint8_t lo) {
  return (int16_t)(((uint16_t)hi << 8) | (uint16_t)lo);
}
static HAL_StatusTypeDef ICM_Init(void) {
  uint8_t id = 0;
  if (HAL_I2C_IsDeviceReady(&hi2c1, ICM_ADDR, 3u, 100u) != HAL_OK) return HAL_ERROR;
  if (ICM_Write(REG_PWR_MGMT_1, 0x80u) != HAL_OK) return HAL_ERROR;
  /* Non-blocking-delay style wait: no HAL_Delay() is used. */
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < 100u) { }
  if (ICM_Read(REG_WHO_AM_I, &id, 1u) != HAL_OK) return HAL_ERROR;
  if (id != ICM_WHO_AM_I_VALUE) return HAL_ERROR;
  if (ICM_Write(REG_PWR_MGMT_1, 0x01u) != HAL_OK) return HAL_ERROR;
  if (ICM_Write(REG_PWR_MGMT_2, 0x00u) != HAL_OK) return HAL_ERROR;
  if (ICM_Write(REG_CONFIG, 0x03u) != HAL_OK) return HAL_ERROR;
  if (ICM_Write(REG_SMPLRT_DIV, 0x13u) != HAL_OK) return HAL_ERROR; /* 50 Hz */
  if (ICM_Write(REG_GYRO_CFG, 0x08u) != HAL_OK) return HAL_ERROR; /* +/-500 dps */
  if (ICM_Write(REG_ACCEL_CFG, 0x00u) != HAL_OK) return HAL_ERROR; /* +/-2 g */
  if (ICM_Write(REG_ACCEL_CFG2, 0x03u) != HAL_OK) return HAL_ERROR;
  return HAL_OK;
}
static HAL_StatusTypeDef ICM_ReadSample(ICM20608_Data *out) {
  uint8_t b[14];
  if (ICM_Read(REG_ACCEL_XOUT, b, sizeof(b)) != HAL_OK) return HAL_ERROR;
  out->ax_g = (float)ICM_Signed16(b[0], b[1]) / 16384.0f;
  out->ay_g = (float)ICM_Signed16(b[2], b[3]) / 16384.0f;
  out->az_g = (float)ICM_Signed16(b[4], b[5]) / 16384.0f;
  out->gx_dps = (float)ICM_Signed16(b[8], b[9]) / 65.5f;
  out->gy_dps = (float)ICM_Signed16(b[10], b[11]) / 65.5f;
  out->gz_dps = (float)ICM_Signed16(b[12], b[13]) / 65.5f;
  return HAL_OK;
}

/* Gyro bias calibration. Hold the toe-mounted sensor absolutely still. */
static void Calibrate_Gyro(void) {
  double sx = 0.0, sy = 0.0, sz = 0.0;
  uint16_t good = 0;
  uint32_t next_sample = HAL_GetTick();
  for (uint16_t i = 0; i < CALIBRATION_SAMPLES; ) {
    while ((int32_t)(HAL_GetTick() - next_sample) < 0) { }
    next_sample += SAMPLE_PERIOD_MS;
    if (ICM_ReadSample(&imu) == HAL_OK) {
      sx += imu.gx_dps; sy += imu.gy_dps; sz += imu.gz_dps;
      good++; i++;
    }
  }
  if (good < (CALIBRATION_SAMPLES * 9u / 10u)) Error_Handler();
  gyro_bias_x = (float)(sx / (double)good);
  gyro_bias_y = (float)(sy / (double)good);
  gyro_bias_z = (float)(sz / (double)good);
}

static void StepDetector_Init(void) {
  steps.filtered_mag = 1.0f;
  steps.previous_dynamic = 0.0f;
  steps.last_step_ms = 0u;
  steps.count = 0u;
  steps.armed = 1u;
}
static void StepDetector_Update(float ax, float ay, float az, uint32_t now_ms) {
  float mag = sqrtf(ax * ax + ay * ay + az * az);
  steps.filtered_mag += STEP_LP_ALPHA * (mag - steps.filtered_mag);
  float dynamic = steps.filtered_mag - 1.0f;
  if (dynamic < STEP_REARM_LEVEL) steps.armed = 1u;
  if (steps.armed && steps.previous_dynamic >= STEP_PEAK_THRESHOLD &&
      dynamic < steps.previous_dynamic &&
      (steps.last_step_ms == 0u || (now_ms - steps.last_step_ms) >= STEP_MIN_INTERVAL_MS)) {
    steps.count++;
    steps.last_step_ms = now_ms;
    steps.armed = 0u;
  }
  steps.previous_dynamic = dynamic;
}

/* Timer ISR callback only schedules work; it never reads I2C or sends UART. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM2) {
    if (sample_due) missed_samples++;
    sample_due = 1u;
  }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) uart_busy = 0u;
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
