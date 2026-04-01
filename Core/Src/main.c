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
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ads1115.h" /* Driver ADS1115 I2C ADC */
#include <stdio.h>   /* sscanf() – parse chuoi "R:xx P:xx Y:xx" */
#include <stdlib.h>  /* atoi() */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* =========================================================
 * CAU HINH DANG SONG XEN KE N1/N2
 * =========================================================
 * TIM3 tren bus APB1 (APB1 prescaler = 4):
 *   TIM3_CLK = PCLK1 x 2 = (168MHz/4) x 2 = 84 MHz
 *
 * f_key  = tan so co ban (nua chu ky T/2)
 * T_HALF = 1 / (2 * f_key)  =>  ARR_HALF = TIM3_CLK / (2*f_key) - 1
 * TIM3 nap lai ARR_HALF => moi interrupt = 1 nua chu ky (T/2)
 *
 * Trong moi interrupt ta dem:
 *   - N1 nua chu ky (toggle nhanh – f_key)
 *   - N2 x 3 nua chu ky (toggle cham – f_key/3)
 * Va dao cuc logic moi lan doi nhom.
 *
 * Kenh xuat (GPIO toggle thu cong):
 *   PC6 = cuc hien tai
 *   PB5 = dao nguoc PC6
 * ========================================================= */
#define TIM3_CLK_HZ     84000000UL      /* TIM3 input clock = 84 MHz      */
#define PWM_PRESCALER   0U              /* Prescaler = 0                  */

#define PWM_F_KEY       10000UL         /* Tan so co ban f_key = 50 kHz   */
#define PWM_N1          2U              /* So nua chu ky T/2 trong nhom 1  */
#define PWM_N2          1U              /* So nua chu ky 3T/2 trong nhom 2 */

/* ARR cho 1 nua chu ky T/2:  TIM3_CLK / (2 * f_key) - 1 = 84e6/100000 - 1 = 839 */
#define PWM_ARR_HALF    ((uint32_t)(TIM3_CLK_HZ / (2UL * PWM_F_KEY)) - 1U)

/* TIM1 tren APB2 (prescaler=4): TIM1_CLK = PCLK2 x 2 = 84 MHz (giong TIM3)
 * ARR cho 1 chu ky day du tai f_key: 84e6 / 50000 - 1 = 1679
 * CCR = ARR + 1 = 1680  =>  duty 100% trong PWM1 mode */
#define TIM1_CLK_HZ     84000000UL
#define TIM1_ARR_FKEY   ((uint32_t)(TIM1_CLK_HZ / PWM_F_KEY) - 1U)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

volatile int k = 0;               /* Bien nhan gia tri tu CC1310 (legacy) */
volatile uint8_t uart_rx_byte;    /* Buffer nhan 1 byte qua UART interrupt */
volatile uint8_t uart_rx_buf[48]; /* Buffer tich luy chuoi ASCII (du cho
                                     "R:+180.00 P:+180.00 Y:+180.00") */
volatile uint8_t uart_rx_idx = 0; /* Chi so vi tri trong buffer */

/* --- 3 bien goc 3 truc tu MPU6050 (doc tu CC1310 qua UART) --- */
volatile float imu_roll = 0.0f;  /* Goc lan [deg], quay quanh truc X */
volatile float imu_pitch = 0.0f; /* Goc nghieng [deg], quay quanh truc Y */
volatile float imu_yaw = 0.0f;   /* Goc xoay [deg], quay quanh truc Z */

uint16_t j = 0; /* Bien luu raw ADC 16-bit khong dau tu ADS1115 (0-65535) */

/* ======= DEBUG: dem so byte nhan duoc tu CC1310 ======= */
// volatile uint16_t stm_uart_rx_count = 0; /* Tong so byte da nhan */
// volatile uint16_t stm_newline_count = 0; /* So lan nhan duoc '\n' */
// volatile uint16_t stm_parse_ok = 0;     /* So lan parse thanh cong */
volatile uint16_t stm_parse_fail = 0; /* So lan parse that bai */

/* ======= Bien trang thai dang song xen ke N1/N2 ======= */
/* Nhom A: N1 nua chu ky (moi nua chu ky la 1 toggle T/2)  */
/* Nhom B: N2 lan 3 nua chu ky (moi lan B gom 3 half-IRQ)  */
/*                                                          */
/* pwm_group  = 0 => dang trong nhom A (N1 x T/2)           */
/* pwm_group  = 1 => dang trong nhom B (N2 x 3T/2)          */
/* pwm_cnt    = dem so lan toggle trong nhom hien tai        */
/* pwm_pol    = cuc hien tai cua PC6 (0=LOW, 1=HIGH)        */
volatile uint8_t  pwm_group = 0;  /* 0=nhom A, 1=nhom B    */
volatile uint16_t pwm_cnt   = 0;  /* dem T/2 trong nhom     */
volatile uint8_t  pwm_pol   = 0;  /* cuc hien tai PC6       */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USB_HOST_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* =========================================================
   * THUAT TOAN DANG SONG XEN KE N1/N2
   * ---------------------------------------------------------
   * PC6 (PB5 dao nguoc) duoc dieu khien bang GPIO toggle
   * trong TIM3 Update interrupt.
   *
   * TIM3 ARR = PWM_ARR_HALF => moi interrupt = 1 nua chu ky.
   * Nhom A: N1 nua chu ky lien tiep (f_key)
   * Nhom B: N2 x 3 nua chu ky (f_key/3)
   * Moi lan doi nhom: dao cuc PC6/PB5.
   * ========================================================= */

  /* Tat PWM hardware – dung GPIO toggle thu cong */
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);

  /* Cau hinh PC6, PB5 thanh GPIO output (ghi de AF cua TIM3) */
  {
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Pull  = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Pin   = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin   = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &gpio);
  }

  /* Trang thai ban dau: PC6=LOW, PB5=HIGH */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  pwm_pol   = 0;
  pwm_group = 0;
  pwm_cnt   = 0;

  /* Thiet lap ARR = T/2 period */
  __HAL_TIM_SET_AUTORELOAD(&htim3, PWM_ARR_HALF);
  HAL_TIM_Base_Start_IT(&htim3); /* Bat TIM3 base + Update interrupt */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

  /* Bat dau nhan UART interrupt tung byte tu CC1310 */
  HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);

  /* Khoi tao ADS1115 voi I2C1, Data Rate 128SPS, PGA +/-6.144V (do duoc 3V3) */
  ADS1115_Init(&hi2c1, ADS1115_DATA_RATE_128, ADS1115_PGA_TWOTHIRDS);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    /* Doc ADC tu ADS1115 kenh AIN0 (Single-Ended), ket qua tinh bang mV luu vao
     * j */
    ADS1115_readSingleEnded(ADS1115_MUX_AIN1, &j);

    HAL_Delay(10); /* Cho 10ms giua moi lan doc */

    /* Vi du: Tang dan duty cycle tu 0% -> 100% roi quay lai */
    //    for (uint16_t pulse = 0; pulse <= 839; pulse += 84) {
    //      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
    //      HAL_Delay(100);
    //    }
    //    for (uint16_t pulse = 839; pulse > 0; pulse -= 84) {
    //      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
    //      HAL_Delay(100);
    //    }
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = TIM1_ARR_FKEY;  /* f_key = 50 kHz, ARR = 1679 */
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* CCR = ARR + 1 => duty 100% (CNT < CCR luon dung trong PWM1 mode) */
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, TIM1_ARR_FKEY*0.95 + 1U);
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PWM_ARR_HALF;     /* ARR = T/2 period cho dang song N1/N2   */
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* Dang song N1/N2 dung GPIO toggle thu cong trong IRQ,
   * khong can set CCR (PWM compare). Chi can bat NVIC. */
  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

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

/**
 * @brief  TIM3 Update interrupt callback – tao dang song xen ke N1/N2
 *
 * Moi lan goi = 1 nua chu ky T/2 troi qua.
 *
 * Nhom A (pwm_group=0): N1 nua chu ky lien tiep.
 *   - Moi IRQ: toggle PC6/PB5 ngay (1 canh len hoac xuong).
 *   - Sau N1 toggle: doi sang nhom B, dao cuc ban dau cua nhom.
 *
 * Nhom B (pwm_group=1): N2 lan 3T/2.
 *   - Moi 3T/2 = 3 nua chu ky => chi toggle 1 lan moi 3 IRQ.
 *   - Sau N2*3 IRQ: doi sang nhom A, dao cuc ban dau cua nhom.
 *
 * PC6 = pwm_pol;  PB5 = !pwm_pol;
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM3) return;

  if (pwm_group == 0)
  {
    /* ---- NHOM A: moi IRQ = 1 canh toggle (nua chu ky T/2) ---- */
    pwm_pol ^= 1U;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,
                      pwm_pol ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,
                      pwm_pol ? GPIO_PIN_RESET : GPIO_PIN_SET);

    pwm_cnt++;
    if (pwm_cnt >= (uint16_t)PWM_N1)
    {
      /* Het nhom A, sang nhom B; dao cuc (khong toggle them, chi doi nhom) */
      pwm_group = 1;
      pwm_cnt   = 0;
    }
  }
  else
  {
    /* ---- NHOM B: moi 3 IRQ = 1 canh toggle (3T/2) ---- */
    pwm_cnt++;
    if (pwm_cnt % 3U == 0U)
    {
      /* Toggle tai moc 3, 6, 9 ... */
      pwm_pol ^= 1U;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,
                        pwm_pol ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,
                        pwm_pol ? GPIO_PIN_RESET : GPIO_PIN_SET);
    }

    if (pwm_cnt >= (uint16_t)(PWM_N2 * 3U))
    {
      /* Het nhom B, sang nhom A */
      pwm_group = 0;
      pwm_cnt   = 0;
    }
  }
}

/**
 * @brief  Callback khi nhan xong 1 byte qua UART (goi tu HAL_UART_IRQHandler)
 * @param  huart: con tro UART handle
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART2) {
    // stm_uart_rx_count++;  /* dem byte nhan duoc */

    if (uart_rx_byte == '\n') {
      // stm_newline_count++;  /* dem so '\n' nhan duoc */
      /* Ket thuc chuoi: them null terminator */
      uart_rx_buf[uart_rx_idx] = '\0';
      uart_rx_idx = 0;

      /*
       * Parse chuoi dinh dang: "R:427 P:-2584 Y:-125013"
       * CC1310 gui so nguyen x1000 de tranh sscanf %f
       * (arm-none-eabi-gcc khong ho tro %f trong sscanf mac dinh).
       * Chia cho 1000.0f de co lai gia tri goc float.
       */
      int tmp_r = 0, tmp_p = 0, tmp_y = 0;
      int parsed =
          sscanf((char *)uart_rx_buf, "R:%d P:%d Y:%d", &tmp_r, &tmp_p, &tmp_y);
      if (parsed == 3) {
        imu_roll = tmp_r / 1000.0f;
        imu_pitch = tmp_p / 1000.0f;
        imu_yaw = tmp_y / 1000.0f;

        /* Giu lai bien k de tuong thich nguoc (lay phan nguyen cua roll) */
        k = tmp_r / 1000;
        // stm_parse_ok++;  /* dem so lan parse thanh cong */
      } else {
        // stm_parse_fail++;  /* dem so lan parse that bai */
      }

    } else if (uart_rx_byte == '\r') {
      /* Bo qua carriage return */
    } else {
      /* Tich luy byte vao buffer (tranh tran bo nho) */
      if (uart_rx_idx < sizeof(uart_rx_buf) - 1) {
        uart_rx_buf[uart_rx_idx++] = uart_rx_byte;
      }
    }

    /* Kich hoat lai interrupt de nhan byte tiep theo */
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
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
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
