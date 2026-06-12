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
#include "arm_math.h"
#include "current_rms.h"
#include "pfm_control.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* =========================================================
 * CURRENT MEASUREMENT CONFIGURATION - Software RMS via ADC1
 * ---------------------------------------------------------
 * ACS70331 Vout -> PA1 (ADC1_CH1)  : Current Sensor PFM #1
 * ACS70331 Vout -> PA2 (ADC1_CH2)  : Current Sensor PFM #2
 * ACS70331 Vout -> PA3 (ADC1_CH3)  : Current Sensor PFM #3
 *
 * See Core/Inc/current_rms.h for deadband, EMA alpha, and sample settings.
 * ========================================================= */

#define TIM3_CLK_HZ 84000000UL
#define PWM_PRESCALER 0U

#define PWM_F_KEY 50512L

#define PWM_ARR_HALF ((uint32_t)(TIM3_CLK_HZ / (2UL * PWM_F_KEY)) - 1U)

#define TIM1_CLK_HZ 84000000UL
#define TIM1_ARR_FKEY ((uint32_t)(TIM1_CLK_HZ / PWM_F_KEY) - 1U)

/* =========================================================
 * FIXED N1/N2 DEBUG CONFIGURATION
 * ---------------------------------------------------------
 * Uncomment PFM_N1N2_FIXED to bypass PID/PFM computation.
 * Each PFM unit has its own DBG_N1_x / DBG_N2_x pair.
 *   N1 = number of fast toggles (T/2)
 *   N2 = number of slow toggles (3T/2)
 * ========================================================= */
#define PFM_N1N2_FIXED /* Comment this line to enable automatic PID/PFM */
/* Bo PFM #1: TIM3 -> PC6/PB5 */
#define DBG_N1_1 10U
#define DBG_N2_1 1U
/* Bo PFM #2: TIM3 -> PC8/PC9 */
#define DBG_N1_2 2U
#define DBG_N2_2 1U
/* Bo PFM #3: TIM1 -> PE9/PE11 */
#define DBG_N1_3 2U
#define DBG_N2_3 1U

/* =========================================================
 * ACS70331 SENSITIVITY CONFIGURATION
 * =========================================================
 * Sensitivity depends on sensor variant and supply voltage:
 *   ACS70331EESASR-5A-T  @3.3V: ~225 mV/A
 *   ACS70331EESASR-2P5A  @3.3V: ~451 mV/A
 * ========================================================= */
#define ACS_SENS_MV_A_1 225.0f /* Sensor 1 Sensitivity (mV/A) - PFM #1 */
#define ACS_SENS_MV_A_2 225.0f /* Sensor 2 Sensitivity (mV/A) - PFM #2 */
#define ACS_SENS_MV_A_3 451.0f /* Sensor 3 Sensitivity (mV/A) - PFM #3 */
#define CURRENT_MEAS_MS 10U    /* Current update period (ms) */
#define IX_SET_AMP 0.1f        /* Default current setpoint (A) */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* ======= Current Measurement Channels (Software RMS via ACS70331) ======= */
/* Sensor 1: PA1 (ADC1_CH1) -> PFM #1 (PC6/PB5) */
CRMS_Channel_t g_crms_1;

/* Sensor 2: PA2 (ADC1_CH2) -> PFM #2 (PC8/PC9) */
CRMS_Channel_t g_crms_2;

/* Sensor 3: PA3 (ADC1_CH3) -> PFM #3 (PE9/PE11) */
CRMS_Channel_t g_crms_3;

/* ======= PFM Waveform State (N1/N2 Interleaving) ======= */
/* PFM #1 (TIM3 -> PC6/PB5) */
volatile uint8_t pwm1_group = 0U; /* 0 = Group A, 1 = Group B */
volatile uint16_t pwm1_cnt = 0U;  /* Half-period counter within group */
volatile uint8_t pwm1_pol = 0U;   /* Current polarity PC6 (0=LOW, 1=HIGH) */
volatile uint8_t pfm1_N1 = 2U;    /* Fast toggles (updated by PFM_Update_1) */
volatile uint8_t pfm1_N2 = 1U;    /* Slow toggles (updated by PFM_Update_1) */

/* PFM #2 (TIM3 -> PC8/PC9) */
volatile uint8_t pwm2_group = 0U;
volatile uint16_t pwm2_cnt = 0U;
volatile uint8_t pwm2_pol = 0U;
volatile uint8_t pfm2_N1 = 2U;
volatile uint8_t pfm2_N2 = 1U;

/* PFM #3 (TIM1 -> PE9/PE11) */
volatile uint8_t pwm3_group = 0U;
volatile uint16_t pwm3_cnt = 0U;
volatile uint8_t pwm3_pol = 0U;
volatile uint8_t pfm3_N1 = 2U;
volatile uint8_t pfm3_N2 = 1U;

/* ======= PFM Controllers ======= */
PFM_Controller_t pfm_ctrl_1;
PFM_Controller_t pfm_ctrl_2;
PFM_Controller_t pfm_ctrl_3;

/* ======= Dong do thuc vao PID — lay tu CRMS_Channel_t ======= */
volatile float ix1_meas_amp = 0.0f; /* Dong cam bien 1 -> PFM #1 */
volatile float ix2_meas_amp = 0.0f; /* Dong cam bien 2 -> PFM #2 */
volatile float ix3_meas_amp = 0.0f; /* Dong cam bien 3 -> PFM #3 */

/* ======= ADC DMA Buffer ======= */
uint16_t adc_dma_buf[CRMS_SAMPLES * 3];

/* ======= BENCHMARK: DWT cycle counter (168 MHz) ======= */
volatile uint32_t bench_pid_cy = 0U; /* arm_pid_f32() execution [cycles] */
volatile uint32_t bench_pfm_cy = 0U; /* PFM_ComputeN1N2() execution [cycles] */
volatile uint32_t bench_total_cy =
    0U;                             /* Total PFM_Update() execution [cycles] */
volatile float bench_pid_us = 0.0f; /* arm_pid_f32() execution [us] */
volatile float bench_total_us = 0.0f; /* Total PFM_Update() execution [us] */

/* USER CODE BEGIN PV */

/* ======= UART3 RX variables (LabVIEW integration) ======= */
volatile float I1_set = 0.0f; /* Current setpoint 1 (A) */
volatile float I2_set = 0.0f; /* Current setpoint 2 (A) */
volatile float I3_set = 0.0f; /* Current setpoint 3 (A) */

volatile uint8_t rx_data = 0;    /* Latest received byte */
volatile uint8_t rx_index = 0;   /* Current index in rx_buffer */
volatile uint8_t data_ready = 0; /* Flag: complete packet received */
char rx_buffer[50];              /* RX buffer string */

/* ==== DEBUG VARIABLES ==== */
volatile uint32_t rx_byte_count = 0;    /* Total received bytes */
volatile uint32_t error_count = 0;      /* UART ErrorCallback calls */
volatile uint32_t data_ready_count = 0; /* Packets fully received */
volatile uint32_t parse_count = 0;      /* Successful parsing count */
volatile uint32_t parse_fail_count = 0; /* Failed parsing count */
volatile uint32_t tx_count = 0;         /* TX transmissions count */
char rx_snapshot[50];                   /* RX string snapshot for debug */
char tx_snapshot[64];                   /* TX string snapshot for debug */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
static void PFM_Update_1(float i_set, float i_meas);
static void PFM_Update_2(float i_set, float i_meas);
static void PFM_Update_3(float i_set, float i_meas);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* =========================================================
 * DWT CYCLE COUNTER FOR BENCHMARKING (168 MHz)
 * 1 cycle = 1/168e6 s ~ 5.95 ns
 * Usage: DWT_START(); ... code ... uint32_t cy = DWT_STOP();
 * ========================================================= */
#define DWT_START()                                                            \
  do {                                                                         \
    DWT->CYCCNT = 0U;                                                          \
  } while (0)
#define DWT_STOP() (DWT->CYCCNT)
#define DWT_US(cy) ((float)(cy) / 168.0f) /* cycles -> microseconds */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_TIM3_Init();
  MX_USB_HOST_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* =========================================================
   * INITIALIZE 3 PFM UNITS & CMSIS-DSP PID
   * ---------------------------------------------------------
   * Unit 1: TIM3 IRQ -> GPIO PC6 / PB5  (Coil 1)
   * Unit 2: TIM3 IRQ -> GPIO PC8 / PC9  (Coil 2)
   * Unit 3: TIM1 IRQ -> GPIO PE9 / PE11 (Coil 3)
   *
   * Each unit has its own PID instance and N1/N2 values.
   * TIM3 and TIM1 share ARR_HALF for identical toggle frequencies.
   * ========================================================= */

  /* ----- Disable hardware PWM outputs to allow manual GPIO toggling ----- */
  /* (Avoid HAL_TIM_PWM_Stop as it alters timer state via __HAL_TIM_DISABLE) */
  TIM3->CCER &=
      ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
  TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);
  /* Ensure MOE is set for TIM1 (advanced timer) so counter operates */
  TIM1->BDTR |= TIM_BDTR_MOE;

  /* ----- Enable GPIOE clock (required for PE9/PE11) ----- */
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* ----- Configure GPIO Output Push-Pull for all 3 units ----- */
  {
    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    /* Bo 1: PC6, PB5 */
    gpio.Pin = GPIO_PIN_6;
    HAL_GPIO_Init(GPIOC, &gpio);
    gpio.Pin = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOB, &gpio);

    /* Bo 2: PC8, PC9 */
    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    HAL_GPIO_Init(GPIOC, &gpio);

    /* Bo 3: PE9, PE11 */
    gpio.Pin = GPIO_PIN_9 | GPIO_PIN_11;
    HAL_GPIO_Init(GPIOE, &gpio);
  }

  /* ----- Initial State: Channel A = LOW, Channel B = HIGH ----- */
  /* Unit 1 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  pwm1_pol = 0U;
  pwm1_group = 0U;
  pwm1_cnt = 0U;

  /* Unit 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  pwm2_pol = 0U;
  pwm2_group = 0U;
  pwm2_cnt = 0U;

  /* Unit 3 */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);
  pwm3_pol = 0U;
  pwm3_group = 0U;
  pwm3_cnt = 0U;

  /* ----- Bat TIM3 base + Update IRQ (ca bo 1 va bo 2 dung chung TIM3) ----- */
  htim3.State = HAL_TIM_STATE_READY;
  __HAL_TIM_SET_AUTORELOAD(&htim3, PWM_ARR_HALF);
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(&htim3);

  /* ----- Enable TIM1 base + Update IRQ (Unit 3) ----- */
  htim1.State = HAL_TIM_STATE_READY;
  __HAL_TIM_SET_AUTORELOAD(&htim1, PWM_ARR_HALF);
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(&htim1);

  /* ----- Initialize PFM Controllers (Kp=0.3, Ki=0.0005, Kd=0) ----- */
  PFM_Init(&pfm_ctrl_1, 0.3f, 0.0005f, 0.0f);
  PFM_Init(&pfm_ctrl_2, 0.3f, 0.0005f, 0.0f);
  PFM_Init(&pfm_ctrl_3, 0.3f, 0.0005f, 0.0f);

  /* ----- Initialize Software RMS Current Measurement ----- */
  CRMS_Init(&g_crms_1); /* Sensor 1: PA1 -> PFM #1 */
  CRMS_Init(&g_crms_2); /* Sensor 2: PA2 -> PFM #2 */
  CRMS_Init(&g_crms_3); /* Sensor 3: PA3 -> PFM #3 */

  /* Start ADC with DMA continuously filling the buffer */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_dma_buf, CRMS_SAMPLES * 3);

  /* ----- Enable DWT Cycle Counter ----- */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    /* =========================================================
     * READ ACS70331 CURRENT SENSORS (Software RMS)
     * ---------------------------------------------------------
     * Unit 1: PA1 (ADC1_CH1) -> g_crms_1 -> PID #1
     * Unit 2: PA2 (ADC1_CH2) -> g_crms_2 -> PID #2
     * Unit 3: PA3 (ADC1_CH3) -> g_crms_3 -> PID #3
     * ========================================================= */
    static uint32_t adc_tick = 0;
    if ((HAL_GetTick() - adc_tick) >= CURRENT_MEAS_MS) {
      adc_tick = HAL_GetTick();

      CRMS_Update(&g_crms_1, adc_dma_buf, 3, 0, CRMS_SAMPLES, ACS_SENS_MV_A_1);
      ix1_meas_amp = g_crms_1.current_A;

      CRMS_Update(&g_crms_2, adc_dma_buf, 3, 1, CRMS_SAMPLES, ACS_SENS_MV_A_2);
      ix2_meas_amp = g_crms_2.current_A;

      CRMS_Update(&g_crms_3, adc_dma_buf, 3, 2, CRMS_SAMPLES, ACS_SENS_MV_A_3);
      ix3_meas_amp = g_crms_3.current_A;
    }

    /* =========================================================
     * PID LOOP: 1 kHz (1 ms)
     * ========================================================= */
    static uint32_t pid_tick = 0;
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - pid_tick) >= 1U) {
      pid_tick = now_ms;

      /* --- PID + PFM update (benchmark unit 1 execution time) --- */
      DWT_START();
      PFM_Update_1(I1_set, ix1_meas_amp);
      bench_total_cy = DWT_STOP();
      bench_total_us = DWT_US(bench_total_cy);

      PFM_Update_2(I2_set, ix2_meas_amp);
      PFM_Update_3(I3_set, ix3_meas_amp);
    }

    /* =========================================================
     * UART3 COMMUNICATION (FULL DUPLEX)
     * ---------------------------------------------------------
     * Transmit telemetry data non-blocking via IT.
     * Using static buffer since HAL uses it asynchronously.
     * ========================================================= */
    static uint32_t uart3_tick = 0U;
    static char uart3_tx_buf[64];

    if ((HAL_GetTick() - uart3_tick) >= 100U &&
        huart3.gState == HAL_UART_STATE_READY) {
      uart3_tick = HAL_GetTick();

      uint16_t len =
          (uint16_t)sprintf(uart3_tx_buf, "Ia=%.2f Ib=%.2f Ic=%.2f\r\n",
                            (float)I1_set, (float)I2_set, (float)I3_set);

      tx_count++;
      memcpy(tx_snapshot, uart3_tx_buf, sizeof(tx_snapshot));
      HAL_UART_Transmit_IT(&huart3, (uint8_t *)uart3_tx_buf, len);
    }

    if (data_ready == 1) {
      memcpy(rx_snapshot, rx_buffer, sizeof(rx_buffer));

      char *p = rx_buffer;
      char *endp;

      /* Skip non-numeric characters */
      while (*p != '\0' && *p != '-' && *p != '.' && (*p < '0' || *p > '9')) {
        p++;
      }

      float tmp1 = strtof(p, &endp);
      if (endp != p && *endp == ',') {
        p = endp + 1;
        float tmp2 = strtof(p, &endp);
        if (endp != p && *endp == ',') {
          p = endp + 1;
          float tmp3 = strtof(p, &endp);
          if (endp != p) {
            I1_set = tmp1;
            I2_set = tmp2;
            I3_set = tmp3;
            parse_count++;
          } else {
            parse_fail_count++;
          }
        } else {
          parse_fail_count++;
        }
      } else {
        parse_fail_count++;
      }

      /* Reset counters for next packet */
      data_ready = 0;
      rx_index = 0;
      memset(rx_buffer, 0, sizeof(rx_buffer));
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
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
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data
   * Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

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
  htim1.Init.Period = 8399;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  /* TIM1 dung GPIO toggle thu cong cho bo PFM #3 (PE9/PE11).
   * Bat Update IRQ de goi HAL_TIM_PeriodElapsedCallback moi T/2. */
  HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

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
  htim3.Init.Period = 8399;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK) {
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
 * @brief USART3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART3_UART_Init(void) {

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
  if (HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */
}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
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
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin,
                    GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD,
                    LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin,
                    GPIO_PIN_RESET);

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

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin | SPI1_MISO_Pin | SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
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
  GPIO_InitStruct.Pin = LD4_Pin | LD3_Pin | LD5_Pin | LD6_Pin | Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin | I2S3_SCK_Pin | I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#include "pfm_control.h"

static void PFM_Update_1(float i_set, float i_meas) {
#ifdef PFM_N1N2_FIXED
  (void)i_set;
  (void)i_meas;
  pfm1_N1 = DBG_N1_1;
  pfm1_N2 = DBG_N2_1;
#else
  DWT_START();
  PFM_Update(&pfm_ctrl_1, i_set, i_meas);
  pfm1_N1 = pfm_ctrl_1.n1;
  pfm1_N2 = pfm_ctrl_1.n2;
  bench_pid_cy = DWT_STOP();
  bench_pid_us = DWT_US(bench_pid_cy);
#endif
}

static void PFM_Update_2(float i_set, float i_meas) {
#ifdef PFM_N1N2_FIXED
  (void)i_set;
  (void)i_meas;
  pfm2_N1 = DBG_N1_2;
  pfm2_N2 = DBG_N2_2;
#else
  PFM_Update(&pfm_ctrl_2, i_set, i_meas);
  pfm2_N1 = pfm_ctrl_2.n1;
  pfm2_N2 = pfm_ctrl_2.n2;
#endif
}

static void PFM_Update_3(float i_set, float i_meas) {
#ifdef PFM_N1N2_FIXED
  (void)i_set;
  (void)i_meas;
  pfm3_N1 = DBG_N1_3;
  pfm3_N2 = DBG_N2_3;
#else
  PFM_Update(&pfm_ctrl_3, i_set, i_meas);
  pfm3_N1 = pfm_ctrl_3.n1;
  pfm3_N2 = pfm_ctrl_3.n2;
#endif
}

/**
 * @brief  TIM Period Elapsed Callback (PFM Waveform Generator)
 *
 * Each interrupt represents one half period (T/2).
 *
 * Group A (pwm_group=0): Fast toggles (N1 half periods).
 *   - Toggle outputs every IRQ.
 *   - After N1 toggles, switch to Group B and invert initial polarity.
 *
 * Group B (pwm_group=1): Slow toggles (N2 times 3T/2).
 *   - Toggle outputs every 3 IRQs.
 *   - After N2*3 IRQs, switch back to Group A and invert initial polarity.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  /* ================================================================
   * PFM Units 1 & 2: TIM3 IRQ
   * #1: PC6 (A) / PB5 (B)  |  #2: PC8 (A) / PC9 (B)
   * ================================================================ */
  if (htim->Instance == TIM3) {

    /* ---- UNIT #1: PC6/PB5 ---- */
    if (pwm1_group == 0) {
      pwm1_pol ^= 1U;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,
                        pwm1_pol ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,
                        pwm1_pol ? GPIO_PIN_RESET : GPIO_PIN_SET);
      pwm1_cnt++;
      if (pwm1_cnt >= (uint16_t)pfm1_N1) {
        pwm1_group = 1;
        pwm1_cnt = 0;
      }
    } else {
      pwm1_cnt++;
      if (pwm1_cnt % 3U == 0U) {
        pwm1_pol ^= 1U;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6,
                          pwm1_pol ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5,
                          pwm1_pol ? GPIO_PIN_RESET : GPIO_PIN_SET);
      }
      if (pwm1_cnt >= (uint16_t)(pfm1_N2 * 3U)) {
        pwm1_group = 0;
        pwm1_cnt = 0;
      }
    }

    /* ---- UNIT #2: PC8/PC9 ---- */
#if 0 /* Temporarily disabled PFM 2 per user request */
    if (pwm2_group == 0) {
      pwm2_pol ^= 1U;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,
                        pwm2_pol ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,
                        pwm2_pol ? GPIO_PIN_RESET : GPIO_PIN_SET);
      pwm2_cnt++;
      if (pwm2_cnt >= (uint16_t)pfm2_N1) {
        pwm2_group = 1;
        pwm2_cnt = 0;
      }
    } else {
      pwm2_cnt++;
      if (pwm2_cnt % 3U == 0U) {
        pwm2_pol ^= 1U;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,
                          pwm2_pol ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,
                          pwm2_pol ? GPIO_PIN_RESET : GPIO_PIN_SET);
      }
      if (pwm2_cnt >= (uint16_t)(pfm2_N2 * 3U)) {
        pwm2_group = 0;
        pwm2_cnt = 0;
      }
    }
#endif
  }

  /* ================================================================
   * PFM Unit #3: TIM1 IRQ
   * PE9 (A) / PE11 (B)
   * ================================================================ */
  if (htim->Instance == TIM1) {
#if 0 /* Temporarily disabled PFM 3 per user request */
    if (pwm3_group == 0) {
      pwm3_pol ^= 1U;
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,
                        pwm3_pol ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11,
                        pwm3_pol ? GPIO_PIN_RESET : GPIO_PIN_SET);
      pwm3_cnt++;
      if (pwm3_cnt >= (uint16_t)pfm3_N1) {
        pwm3_group = 1;
        pwm3_cnt = 0;
      }
    } else {
      pwm3_cnt++;
      if (pwm3_cnt % 3U == 0U) {
        pwm3_pol ^= 1U;
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9,
                          pwm3_pol ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11,
                          pwm3_pol ? GPIO_PIN_RESET : GPIO_PIN_SET);
      }
      if (pwm3_cnt >= (uint16_t)(pfm3_N2 * 3U)) {
        pwm3_group = 0;
        pwm3_cnt = 0;
      }
    }
#endif
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) /* UART3 (LabVIEW communication) */
  {
    rx_byte_count++;

    uint8_t ch = rx_data;

    if (ch == '\n' || ch == '\r') {
      if (rx_index > 0) {
        rx_buffer[rx_index] = '\0';
        data_ready = 1;
        data_ready_count++;
      }
    } else {
      /* Store character safely */
      if (rx_index < (uint8_t)(sizeof(rx_buffer) - 1U)) {
        rx_buffer[rx_index] = ch;
        rx_index++;
      }
    }

    /* Re-enable UART interrupt for next byte */
    HAL_UART_Receive_IT(&huart3, (uint8_t *)&rx_data, 1);
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
