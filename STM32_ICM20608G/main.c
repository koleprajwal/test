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
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ICM20608_ADDR          (0x68U << 1) /* HAL uses 8-bit shifted address */
#define ICM_REG_SMPLRT_DIV     0x19U
#define ICM_REG_CONFIG         0x1AU
#define ICM_REG_GYRO_CONFIG    0x1BU
#define ICM_REG_ACCEL_CONFIG   0x1CU
#define ICM_REG_ACCEL_CONFIG2  0x1DU
#define ICM_REG_ACCEL_XOUT_H   0x3BU
#define ICM_REG_PWR_MGMT_1     0x6BU
#define ICM_REG_PWR_MGMT_2     0x6CU
#define ICM_REG_WHO_AM_I       0x75U
#define ICM_WHO_AM_I_VALUE     0xAFU
#define IMU_READ_TIMEOUT_MS    20U
#define UART_TIMEOUT_MS        20U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static volatile uint8_t sample_due = 0;
static uint32_t sample_timestamp_ms = 0;
static uint8_t imu_online = 0;
static uint8_t imu_whoami = 0;
static uint32_t i2c_error_count = 0;
static uint32_t uart_error_count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef ICM20608_WriteReg(uint8_t reg, uint8_t value);
static HAL_StatusTypeDef ICM20608_ReadReg(uint8_t reg, uint8_t *value);
static HAL_StatusTypeDef ICM20608_Init(void);
static HAL_StatusTypeDef ICM20608_ReadRaw(int16_t raw[7]);
static void SendText(const char *text);
static int16_t MakeInt16(uint8_t msb, uint8_t lsb);
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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_Delay(100); /* allow sensor power-up to settle */

  imu_online = (ICM20608_Init() == HAL_OK) ? 1U : 0U;
  if (imu_online)
  {
    char msg[64];
    (void)snprintf(msg, sizeof(msg),
                   "#ICM20608_OK,WHO_AM_I=0x%02X\r\n", imu_whoami);
    SendText(msg);
    SendText("#DATA: t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,temp_C\r\n");
  }
  else
  {
    char msg[80];
    (void)snprintf(msg, sizeof(msg),
                   "#ICM20608_ERROR,WHO_AM_I=0x%02X,I2C_ERRORS=%lu\r\n",
                   imu_whoami, (unsigned long)i2c_error_count);
    SendText(msg);
  }

  /* TIM2 is configured for 100 Hz: 10 ms interrupt period.
     Keep I2C and UART work out of the interrupt callback. */
  if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    if (sample_due != 0U)
    {
      int16_t raw[7];
      uint32_t timestamp;
      char line[160];

      /* Atomically consume the timer flag. If the loop ever falls behind,
         this deliberately sends the latest sample once instead of queueing
         stale samples. */
      __disable_irq();
      sample_due = 0U;
      timestamp = HAL_GetTick();
      __enable_irq();

      if (imu_online)
      {
        if (ICM20608_ReadRaw(raw) == HAL_OK)
        {
          /* Default full-scale settings: accelerometer +/-2 g (16384 LSB/g),
             gyroscope +/-250 dps (131 LSB/dps).
             Temperature conversion from ICM-20608-G datasheet: raw/326.8 + 25 C. */
          float ax = (float)raw[0] / 16384.0f;
          float ay = (float)raw[1] / 16384.0f;
          float az = (float)raw[2] / 16384.0f;
          float temp_c = ((float)raw[3] / 326.8f) + 25.0f;
          float gx = (float)raw[4] / 131.0f;
          float gy = (float)raw[5] / 131.0f;
          float gz = (float)raw[6] / 131.0f;

          int n = snprintf(line, sizeof(line),
                           "%lu,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f,%.2f\r\n",
                           (unsigned long)timestamp, ax, ay, az, gx, gy, gz, temp_c);
          if (n > 0 && (size_t)n < sizeof(line))
          {
            if (HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)n,
                                  UART_TIMEOUT_MS) != HAL_OK)
            {
              uart_error_count++;
            }
          }
        }
        else
        {
          i2c_error_count++;
          (void)snprintf(line, sizeof(line), "#I2C_READ_ERROR,%lu\r\n",
                         (unsigned long)i2c_error_count);
          SendText(line);
        }
      }
      else
      {
        /* Retry sensor initialization every 1 second after an initial failure. */
        static uint32_t last_retry_ms = 0U;
        if ((timestamp - last_retry_ms) >= 1000U)
        {
          last_retry_ms = timestamp;
          imu_online = (ICM20608_Init() == HAL_OK) ? 1U : 0U;
          if (imu_online)
          {
            (void)snprintf(line, sizeof(line), "#ICM20608_RECOVERED,WHO_AM_I=0x%02X\r\n",
                           imu_whoami);
            SendText(line);
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
  htim2.Init.Period = 99;
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
  huart1.Init.BaudRate = 230400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
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
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static HAL_StatusTypeDef ICM20608_WriteReg(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, ICM20608_ADDR, reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U,
                           IMU_READ_TIMEOUT_MS);
}

static HAL_StatusTypeDef ICM20608_ReadReg(uint8_t reg, uint8_t *value)
{
  return HAL_I2C_Mem_Read(&hi2c1, ICM20608_ADDR, reg,
                          I2C_MEMADD_SIZE_8BIT, value, 1U,
                          IMU_READ_TIMEOUT_MS);
}

static HAL_StatusTypeDef ICM20608_Init(void)
{
  HAL_StatusTypeDef status;
  uint8_t check = 0U;

  /* Verify address/device before attempting configuration. */
  status = ICM20608_ReadReg(ICM_REG_WHO_AM_I, &check);
  if (status != HAL_OK)
  {
    i2c_error_count++;
    return status;
  }
  imu_whoami = check;
  if (check != ICM_WHO_AM_I_VALUE)
  {
    return HAL_ERROR;
  }

  /* Reset device, then wait for reset to complete. */
  status = ICM20608_WriteReg(ICM_REG_PWR_MGMT_1, 0x80U);
  if (status != HAL_OK) { i2c_error_count++; return status; }
  HAL_Delay(100);

  /* Use PLL with X gyro reference; clear SLEEP. */
  status = ICM20608_WriteReg(ICM_REG_PWR_MGMT_1, 0x01U);
  if (status != HAL_OK) { i2c_error_count++; return status; }
  status = ICM20608_WriteReg(ICM_REG_PWR_MGMT_2, 0x00U);
  if (status != HAL_OK) { i2c_error_count++; return status; }
  HAL_Delay(10);

  /* DLPF enabled: internal sample rate 1 kHz. Divider 9 => 100 Hz. */
  status = ICM20608_WriteReg(ICM_REG_CONFIG, 0x03U);
  if (status != HAL_OK) { i2c_error_count++; return status; }
  status = ICM20608_WriteReg(ICM_REG_SMPLRT_DIV, 9U);
  if (status != HAL_OK) { i2c_error_count++; return status; }

  /* Gyro +/-250 dps (FS_SEL=0); accelerometer +/-2 g (AFS_SEL=0). */
  status = ICM20608_WriteReg(ICM_REG_GYRO_CONFIG, 0x00U);
  if (status != HAL_OK) { i2c_error_count++; return status; }
  status = ICM20608_WriteReg(ICM_REG_ACCEL_CONFIG, 0x00U);
  if (status != HAL_OK) { i2c_error_count++; return status; }
  status = ICM20608_WriteReg(ICM_REG_ACCEL_CONFIG2, 0x03U);
  if (status != HAL_OK) { i2c_error_count++; return status; }

  HAL_Delay(10);
  return HAL_OK;
}

static HAL_StatusTypeDef ICM20608_ReadRaw(int16_t raw[7])
{
  uint8_t data[14];
  HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c1, ICM20608_ADDR,
                                               ICM_REG_ACCEL_XOUT_H,
                                               I2C_MEMADD_SIZE_8BIT,
                                               data, sizeof(data),
                                               IMU_READ_TIMEOUT_MS);
  if (status != HAL_OK)
  {
    return status;
  }

  raw[0] = MakeInt16(data[0],  data[1]);  /* accel X */
  raw[1] = MakeInt16(data[2],  data[3]);  /* accel Y */
  raw[2] = MakeInt16(data[4],  data[5]);  /* accel Z */
  raw[3] = MakeInt16(data[6],  data[7]);  /* temperature */
  raw[4] = MakeInt16(data[8],  data[9]);  /* gyro X */
  raw[5] = MakeInt16(data[10], data[11]); /* gyro Y */
  raw[6] = MakeInt16(data[12], data[13]); /* gyro Z */
  return HAL_OK;
}

static int16_t MakeInt16(uint8_t msb, uint8_t lsb)
{
  return (int16_t)((((uint16_t)msb) << 8) | (uint16_t)lsb);
}

static void SendText(const char *text)
{
  if (text != NULL)
  {
    uint16_t len = (uint16_t)strlen(text);
    if (HAL_UART_Transmit(&huart1, (uint8_t *)text, len, 100U) != HAL_OK)
    {
      uart_error_count++;
    }
  }
}

/* Timer ISR only sets a flag. Sensor reads and serial transmission run in main. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)
  {
    sample_due = 1U;
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
