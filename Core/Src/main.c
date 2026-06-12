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
#include "ads1115.h"  /* Driver ADS1115 I2C ADC                */
#include "arm_math.h" /* CMSIS-DSP: arm_pid_f32, float32_t     */
#include <stdio.h>    /* sprintf()                             */
#include <stdlib.h>   /* rand(), srand()                       */
#include <string.h>   /* memset()                              */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* =========================================================
 * CAU HINH LTC1966 — Doc dien ap AC RMS qua ADC1 STM32
 * ---------------------------------------------------------
 * LTC1966 OUT -> PA0 (ADC1 Channel 0)
 * Vref ADC    = 3.3V
 * ADC 12-bit  = 4096 LSB
 *
 * SCALE: he so chia ap ngoai (R1+R2)/R2
 *   Vi du: Vac_rms = 12 Vrms, chia 12:1 -> Vin_ltc = 1 Vrms
 *   LTC1966 OUT ~ Vin_rms trong dai [0, 1V]
 *   SCALE = Vac_rms / Vout_ltc_rms = ti le phun ap
 *
 * Vac_real (V) = adc_voltage_V * SCALE
 * ========================================================= */
#define LTC1966_VREF_V 3.3f   /* Vref ADC (V)                          */
#define LTC1966_ADC_RES 4096U /* Do phan giai ADC 12-bit               */
#define LTC1966_SCALE 101.73f /* He so phun ap ngoai: Vac_real/Vout   */
                              /* Vi du: 12Vrms AC -> SCALE=12          */
#define LTC1966_MEAS_MS 50U   /* Chu ky doc LTC1966 (ms) – phu hop C3 */

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
#define TIM3_CLK_HZ 84000000UL /* TIM3 input clock = 84 MHz      */
#define PWM_PRESCALER 0U       /* Prescaler = 0                  */

#define PWM_F_KEY 50512L /* Tan so co ban f_key = 10 kHz   */

/* ARR cho 1 nua chu ky T/2:  TIM3_CLK / (2*f_key) - 1 = 84e6/20000 - 1 = 4199
 */
#define PWM_ARR_HALF ((uint32_t)(TIM3_CLK_HZ / (2UL * PWM_F_KEY)) - 1U)

/* TIM1 tren APB2: TIM1_CLK = 84 MHz, ARR = 84e6/f_key - 1, CCR=ARR+1 (100%
 * duty) */
#define TIM1_CLK_HZ 84000000UL
#define TIM1_ARR_FKEY ((uint32_t)(TIM1_CLK_HZ / PWM_F_KEY) - 1U)

/* =========================================================
 * HANG SO HE THONG PFM (tu paper, n=1)
 * =========================================================
 * U_max = 2*sqrt(2)*E/pi (E = DC source voltage)
 * delta   = U_new / U_max   in (0, 1]
 * delta_d = (3 - 1/delta) / 2   [Eq.19, n=1]
 * N1 + N2 = PFM_N_TOTAL = 7
 * N1: le (odd), N2: chan (even)
 * ========================================================= */
#define PFM_U_MAX 10.8f     /* U_max (V): 2*sqrt(2)*E/pi, E=12V          */
#define PFM_U_BASE 10.8f    /* U_base khi delta=1 (V)                    */
#define PFM_DU_MIN (-10.8f) /* Gioi han duoi PID output (V) = -U_max   */
#define PFM_DU_MAX 0.0f     /* Gioi han tren: khong che vuot U_max       */
#define PFM_DELTA_MIN 0.3f  /* delta toi thieu (tranh 1/delta -> inf)    */
#define PFM_PID_TS 0.001f   /* Sampling time PID = 1 ms                 */
#define PFM_MAX_TOTAL 12U   /* Gioi han N1+N2 toi da (tuy chinh)         */

/* =========================================================
 * [DEBUG] N1/N2 CO DINH DE XEMM DANG SONG
 * ---------------------------------------------------------
 * Bat #define PFM_N1N2_FIXED de bypass PID + PFM tinh toan.
 * Moi bo PFM co cap DBG_N1_x / DBG_N2_x rieng de test doc lap.
 *   N1 = so nua chu ky nhom A (toggle nhanh, T/2)
 *   N2 = so lan nhom B (moi lan = 3 nua chu ky, 3T/2)
 * Vi du: N1=2, N2=1 => dang song chuan PFM (paper)
 * ========================================================= */
#define PFM_N1N2_FIXED /* Comment dong nay de bat lai PID/PFM tu dong */
/* Bo PFM #1: TIM3 -> PC6/PB5 */
#define DBG_N1_1 2U
#define DBG_N2_1 1U
/* Bo PFM #2: TIM3 -> PC8/PC9 */
#define DBG_N1_2 2U
#define DBG_N2_2 1U
/* Bo PFM #3: TIM1 -> PE9/PE11 */
#define DBG_N1_3 2U
#define DBG_N2_3 1U

/* =========================================================
 * CAU HINH CAM BIEN DONG ACS70331 doc qua ADS1115 A0
 * =========================================================
 * ACS70331: VIOUT = VIOUT_Q + Sensitivity * Ia
 *   VIOUT_Q     = 1666 mV (dien ap khi dong = 0)
 *   Sensitivity = 216  mV/A
 *   => Ia = (V_meas_mV - VIOUT_Q_MV) / SENS_MV_A
 *
 * ADS1115 PGA = 2/3x (±6.144V), 1 LSB = 0.1875 mV
 * ========================================================= */
#define ACS_VIOUT_Q_MV 1500.0f /* Offset khi I=0A (mV)         */
// #define ACS_SENS_MV_A_1 197.0f /* Sensitivity (mV/A)           */
#define ACS_SENS_MV_A_1 225.0f
#define ACS_SENS_MV_A_2 225.0f
#define ACS_SENS_MV_A_3 451.0f
#define ADS_LSB_MV 0.1875f  /* PGA 2/3x: 6144mV/32768 LSB  */
#define CURRENT_MEAS_MS 10U /* Chu ky doc ADS1115 (ms)      */
#define IX_SET_AMP 0.1f     /* Dong dat Iset (A) = 100 mA   */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* ======= BIEN DO DIEN AP AC RMS — LTC1966 qua ADC1 PA0 ======= */
volatile uint32_t g_ltc_adc_raw = 0U; /* Raw 12-bit tu ADC1            */
volatile float g_ltc_adc_volt = 0.0f; /* Dien ap tai PA0 (V)           */
volatile float g_ltc_vrms = 0.0f;     /* Dien ap AC RMS thuc (V)       */
volatile uint8_t g_ltc_ok = 0U;       /* 1 = doc ADC thanh cong        */

/* ======= BIEN DO DONG THUC — Sensor 1: ADS1115 A0 qua I2C1 ======= */
volatile uint16_t g_adc_raw = 0;    /* Raw 16-bit tu ADS1115 #1     */
volatile float g_voltage_mV = 0.0f; /* Dien ap VIOUT sensor 1 (mV)  */
volatile float g_current_A = 0.0f;  /* Dong dien sensor 1 (A)       */
volatile uint8_t g_ads_ok = 0;      /* 1 = ADS1115 #1 doc OK        */

/* ======= BIEN DO DONG THUC — Sensor 2: ADS1115 A1 qua I2C3 ======= */
volatile uint16_t g_adc_raw_2 = 0;    /* Raw 16-bit tu ADS1115 #2   */
volatile float g_voltage_mV_2 = 0.0f; /* Dien ap VIOUT sensor 2 (mV)*/
volatile float g_current_A_2 = 0.0f;  /* Dong dien sensor 2 (A)     */
volatile uint8_t g_ads_ok_2 = 0;      /* 1 = ADS1115 #2 doc OK      */

/* ======= BIEN DO DONG THUC — Sensor 3: ADS1115 A2 qua I2C3 ======= */
volatile uint16_t g_adc_raw_3 = 0;    /* Raw 16-bit tu ADS1115 #3   */
volatile float g_voltage_mV_3 = 0.0f; /* Dien ap VIOUT sensor 3 (mV)*/
volatile float g_current_A_3 = 0.0f;  /* Dong dien sensor 3 (A)     */
volatile uint8_t g_ads_ok_3 = 0;      /* 1 = ADS1115 #3 doc OK      */

/* ======= Bien trang thai dang song xen ke N1/N2 — BO PFM #1 (TIM3->PC6/PB5)
 * ======= */
volatile uint8_t pwm1_group = 0U; /* 0=nhom A, 1=nhom B                   */
volatile uint16_t pwm1_cnt = 0U;  /* dem nua chu ky trong nhom             */
volatile uint8_t pwm1_pol = 0U;   /* cuc hien tai PC6 (0=LOW, 1=HIGH)      */
volatile uint8_t pfm1_N1 = 2U;    /* cap nhat boi PFM_Update_1()           */
volatile uint8_t pfm1_N2 = 1U;

/* ======= Bien trang thai dang song xen ke N1/N2 — BO PFM #2 (TIM3->PC8/PC9)
 * ======= */
volatile uint8_t pwm2_group = 0U;
volatile uint16_t pwm2_cnt = 0U;
volatile uint8_t pwm2_pol = 0U; /* cuc hien tai PC8 (0=LOW, 1=HIGH)      */
volatile uint8_t pfm2_N1 = 2U;  /* cap nhat boi PFM_Update_2()           */
volatile uint8_t pfm2_N2 = 1U;

/* ======= Bien trang thai dang song xen ke N1/N2 — BO PFM #3 (TIM1->PE9/PE11)
 * ======= */
volatile uint8_t pwm3_group = 0U;
volatile uint16_t pwm3_cnt = 0U;
volatile uint8_t pwm3_pol = 0U; /* cuc hien tai PE9 (0=LOW, 1=HIGH)      */
volatile uint8_t pfm3_N1 = 2U;  /* cap nhat boi PFM_Update_3()           */
volatile uint8_t pfm3_N2 = 1U;

/* ======= CMSIS-DSP PID instances (1 instance / cuon day) ======= */
arm_pid_instance_f32 pid_1; /* PID bo 1: dieu khien I_1 */
arm_pid_instance_f32 pid_2; /* PID bo 2: dieu khien I_2 */
arm_pid_instance_f32 pid_3; /* PID bo 3: dieu khien I_3 */

/* ======= Bien PFM debug — Bo 1 ======= */
volatile float pfm1_delta = 1.0f;
volatile float pfm1_delta_d = 1.0f;
volatile float pfm1_u_new = PFM_U_BASE;
volatile float pfm1_du = 0.0f;

/* ======= Bien PFM debug — Bo 2 ======= */
volatile float pfm2_delta = 1.0f;
volatile float pfm2_delta_d = 1.0f;
volatile float pfm2_u_new = PFM_U_BASE;
volatile float pfm2_du = 0.0f;

/* ======= Bien PFM debug — Bo 3 ======= */
volatile float pfm3_delta = 1.0f;
volatile float pfm3_delta_d = 1.0f;
volatile float pfm3_u_new = PFM_U_BASE;
volatile float pfm3_du = 0.0f;

/* ======= Dong do thuc tu ADS1115 — 3 kenh ======= */
volatile float ix1_meas_amp = 0.0f; /* Kenh A0 -> bo PFM #1 */
volatile float ix2_meas_amp = 0.0f; /* Kenh A1 -> bo PFM #2 */
volatile float ix3_meas_amp = 0.0f; /* Kenh A2 -> bo PFM #3 */

/* ======= BENCHMARK: DWT cycle counter (168 MHz) ======= */
/* Ghi lai so cycles thuc thi cua tung buoc chinh:        */
volatile uint32_t bench_pid_cy = 0U;   /* arm_pid_f32() [cycles]         */
volatile uint32_t bench_pfm_cy = 0U;   /* PFM_ComputeN1N2() [cycles]     */
volatile uint32_t bench_total_cy = 0U; /* Tong PFM_Update() [cycles]     */
volatile float bench_pid_us = 0.0f;    /* arm_pid_f32() [us]           */
volatile float bench_total_us = 0.0f;  /* Tong PFM_Update() [us]       */

/* USER CODE BEGIN PV */

/* ======= UART3 RX tu LabVIEW ======= */
volatile float I1_set = 0.0f; /* Dong dat 1 (A) nhan tu LabVIEW */
volatile float I2_set = 0.0f; /* Dong dat 2 (A) nhan tu LabVIEW */
volatile float I3_set = 0.0f; /* Dong dat 3 (A) nhan tu LabVIEW */

volatile uint8_t rx_data = 0;    /* Byte vua nhan tu ISR            */
volatile uint8_t rx_index = 0;   /* Vi tri ghi trong rx_buffer      */
volatile uint8_t data_ready = 0; /* Co: da nhan xong 1 goi (\r/\n) */
char rx_buffer[50];              /* Buffer RX chuoi tu LabVIEW      */

/* ==== BIEN DEBUG — xem trong Watch window ==== */
volatile uint32_t rx_byte_count = 0; /* Tang moi ISR: bao nhieu byte da den   */
volatile uint32_t error_count = 0;   /* Tang khi ErrorCallback bi goi         */
volatile uint32_t data_ready_count =
    0;                             /* Tang khi nhan xong 1 packet (\n/\r)  */
volatile uint32_t parse_count = 0; /* Tang khi strtof parse thanh cong      */
volatile uint32_t parse_fail_count = 0; /* Tang khi strtof parse that bai */
volatile uint32_t tx_count = 0; /* Tang moi lan STM32 gui TX             */
char rx_snapshot[50];           /* Noi dung rx_buffer luc parse — debug */
char tx_snapshot[64];           /* Chuoi TX cuoi cung STM32 gui — debug */

/* ==== BIEN DEBUG TIMER/PFM — xem trong Watch window ==== */
/* dbg_init_step: tang dan theo tung buoc trong USER CODE BEGIN 2.
 *   0 = chua vao USER CODE BEGIN 2
 *   1 = sau CCER disable
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
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
 * LTC1966 — Doc dien ap AC RMS qua ADC1 (PA0, 12-bit, Vref=3.3V)
 * ---------------------------------------------------------
 * Goi ham nay dinh ky (moi LTC1966_MEAS_MS ms) tu main loop.
 * Ket qua:
 *   g_ltc_adc_raw  = gia tri raw 12-bit
 *   g_ltc_adc_volt = dien ap tai PA0 (V)
 *   g_ltc_vrms     = dien ap AC RMS thuc = adc_volt * SCALE (V)
 *   g_ltc_ok       = 1 neu doc thanh cong
 * ========================================================= */
static void LTC1966_ReadVoltage(void) {
  HAL_ADC_Start(&hadc1);
  if (HAL_ADC_PollForConversion(&hadc1, 10U) == HAL_OK) {
    g_ltc_adc_raw = HAL_ADC_GetValue(&hadc1);
    g_ltc_adc_volt =
        (float)g_ltc_adc_raw * LTC1966_VREF_V / (float)(LTC1966_ADC_RES - 1U);
    g_ltc_vrms = g_ltc_adc_volt * LTC1966_SCALE;
    g_ltc_ok = 1U;
  } else {
    g_ltc_ok = 0U;
  }
  HAL_ADC_Stop(&hadc1);
}

/* =========================================================
 * DWT CYCLE COUNTER — do thoi gian thuc thi (168 MHz)
 * 1 cycle = 1/168e6 s ~ 5.95 ns
 * Cach dung:
 *   DWT_START();  ... code can do ...  uint32_t cy = DWT_STOP();
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
  MX_I2C1_Init();
  MX_TIM3_Init();
  MX_USB_HOST_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  /* =========================================================
   * KHOI TAO 3 BO PFM + CMSIS-DSP PID
   * ---------------------------------------------------------
   * Bo 1: TIM3 IRQ -> GPIO PC6 / PB5  (cuon 1)
   * Bo 2: TIM3 IRQ -> GPIO PC8 / PC9  (cuon 2)
   * Bo 3: TIM1 IRQ -> GPIO PE9 / PE11 (cuon 3)
   *
   * Moi bo co PID instance rieng (pid_1/2/3) va N1/N2 rieng.
   * TIM3 va TIM1 cung ARR_HALF nen cung tan so toggle.
   * ========================================================= */

  /* ----- Tat output cua tung kenh TIM3/TIM1 (khong dung HAL_TIM_PWM_Stop
   * vi ham do goi __HAL_TIM_DISABLE() lam hong State timer) ----- */
  /* Tat CCxE (Output Enable) truc tiep: TIM3 CH1/2/3/4, TIM1 CH1/2 */
  TIM3->CCER &=
      ~(TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E);
  TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E);
  /* TIM1 la advanced timer: dam bao MOE duoc set de counter hoat dong */
  TIM1->BDTR |= TIM_BDTR_MOE;

  /* ----- Dam bao clock GPIOE da duoc bat (can cho PE9/PE11) ----- */
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /* ----- Cau hinh GPIO Output Push-Pull cho ca 3 bo ----- */
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

  /* ----- Trang thai ban dau: kenh A=LOW, kenh B=HIGH ----- */
  /* Bo 1 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  pwm1_pol = 0U;
  pwm1_group = 0U;
  pwm1_cnt = 0U;

  /* Bo 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  pwm2_pol = 0U;
  pwm2_group = 0U;
  pwm2_cnt = 0U;

  /* Bo 3 */
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

  /* ----- Bat TIM1 base + Update IRQ (bo 3) ----- */
  htim1.State = HAL_TIM_STATE_READY;
  __HAL_TIM_SET_AUTORELOAD(&htim1, PWM_ARR_HALF);
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
  HAL_TIM_Base_Start_IT(&htim1);

  /* ----- Khoi tao CMSIS-DSP PID cho ca 3 bo (Kp=0.3, Ki=0.0005, Kd=0) ----- */
  pid_1.Kp = 0.3f;
  pid_1.Ki = 0.0005f;
  pid_1.Kd = 0.0f;
  arm_pid_init_f32(&pid_1, 1);

  pid_2.Kp = 0.3f;
  pid_2.Ki = 0.0005f;
  pid_2.Kd = 0.0f;
  arm_pid_init_f32(&pid_2, 1);

  pid_3.Kp = 0.3f;
  pid_3.Ki = 0.0005f;
  pid_3.Kd = 0.0f;
  arm_pid_init_f32(&pid_3, 1);

  /* ----- Khoi tao ADS1115 tren I2C1 (doc 3 kenh A0, A1, A2) ----- */
  ADS1115_Init(&hi2c1, ADS1115_DATA_RATE_128, ADS1115_PGA_TWOTHIRDS);

  /* ----- Kich hoat DWT Cycle Counter ----- */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  HAL_UART_Receive_IT(&huart3, &rx_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    /* =========================================================
     * DOC DONG DIEN TU 3 CAM BIEN ACS70331 QUA 1 ADS1115 (I2C1)
     * ---------------------------------------------------------
     * Sensor 1: kenh A0, ACS_SENS_MV_A_1 -> ix1_meas_amp -> PID #1
     * Sensor 2: kenh A1, ACS_SENS_MV_A_2 -> ix2_meas_amp -> PID #2
     * Sensor 3: kenh A2, ACS_SENS_MV_A_3 -> ix3_meas_amp -> PID #3
     * ADS1115 PGA 2/3x: 1 LSB = 0.1875 mV (signed 16-bit)
     * ========================================================= */
    static uint32_t adc_tick = 0;
    if ((HAL_GetTick() - adc_tick) >= CURRENT_MEAS_MS) {
      adc_tick = HAL_GetTick();

      /* --- Sensor 1: Doc RMS dong dien tu LTC1966 qua ADC1 (PA1) ---
       * Oversampling 256 mau: dat do phan giai hieu dung 16-bit
       * 256 mau x 3.7us/mau ~ 950us tong thoi gian lay mau
       * Ket qua: adc_16bit tu 0->65520 (tuong duong 16-bit)
       * Do phan giai: 3300mV / 65535 = 0.05mV/buoc (thay vi 0.8mV)
       * ------------------------------------------------------- */
      uint32_t ov_sum = 0;
      for (uint16_t i = 0; i < 256; i++) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 5U) == HAL_OK) {
          ov_sum += HAL_ADC_GetValue(&hadc1);
        }
        HAL_ADC_Stop(&hadc1);
      }
      /* Chia 16 de lay 16-bit: 256mau / 16 = 16x trung binh (16-bit) */
      uint16_t adc_16bit = (uint16_t)(ov_sum >> 4);
      g_adc_raw = adc_16bit; /* Luu lai de xem debug (0->65520) */

      g_voltage_mV = (float)adc_16bit * 3300.0f / 65520.0f; /* 16-bit: 0->3300mV */

      float new_current = g_voltage_mV / ACS_SENS_MV_A_1;

      /* Triet tieu nhieu nen (Deadband): chi ep ve 0A khi rat gan 0 (<5mV) */
      if (g_voltage_mV < 5.0f) {
        new_current = 0.0f;
      }

      /* Bo loc nhieu mem EMA (alpha=0.3): phan hoi nhanh hon, van muot */
      g_current_A = (g_current_A * 0.7f) + (new_current * 0.3f);
      ix1_meas_amp = g_current_A;
      g_ads_ok = 1;

#if 0
      /* --- Sensor 2: ADS1115 A1 qua I2C1 --- */
      uint16_t raw2 = 0;
      if (ADS1115_readSingleEnded(&hi2c1, ADS1115_MUX_AIN1, &raw2) == HAL_OK) {
        g_adc_raw_2 = raw2;
        g_voltage_mV_2 = (float)(int16_t)raw2 * ADS_LSB_MV;
        g_current_A_2 = (g_voltage_mV_2 - ACS_VIOUT_Q_MV) / ACS_SENS_MV_A_2;
        ix2_meas_amp = g_current_A_2;
        g_ads_ok_2 = 1;
      } else {
        g_ads_ok_2 = 0;
      }

      /* --- Sensor 3: ADS1115 A2 qua I2C1 --- */
      uint16_t raw3 = 0;
      if (ADS1115_readSingleEnded(&hi2c1, ADS1115_MUX_AIN2, &raw3) == HAL_OK) {
        g_adc_raw_3 = raw3;
        g_voltage_mV_3 = (float)(int16_t)raw3 * ADS_LSB_MV;
        g_current_A_3 = (g_voltage_mV_3 - ACS_VIOUT_Q_MV) / ACS_SENS_MV_A_3;
        ix3_meas_amp = g_current_A_3;
        g_ads_ok_3 = 1;
      } else {
        g_ads_ok_3 = 0;
      }
#endif
    }

    /* =========================================================
     * DOC DIEN AP AC RMS TU LTC1966 QUA ADC1 (PA0)
     * ---------------------------------------------------------
     * Moi LTC1966_MEAS_MS ms: doc raw -> volt -> Vrms thuc
     * g_ltc_vrms = gia tri RMS sau SCALE (V)
     * ========================================================= */
    static uint32_t ltc_tick = 0U;
    if ((HAL_GetTick() - ltc_tick) >= LTC1966_MEAS_MS) {
      ltc_tick = HAL_GetTick();
      // LTC1966_ReadVoltage(); // Comment lai de tranh dung do ADC1 voi phan
      // doc dong dien
    }

    /* =========================================================
     * PID LOOP: 1 kHz (1 ms)
     * ---------------------------------------------------------
     * Dung ix_meas_amp do thuc tu ADS1115, Iset = IX_SET_AMP
     * ========================================================= */
    static uint32_t pid_tick = 0;
    uint32_t now_ms = HAL_GetTick();

    if ((now_ms - pid_tick) >= 1U) {
      pid_tick = now_ms;

      /* --- PID + PFM update cho ca 3 bo (do thoi gian bo 1 bang DWT) --- */
      DWT_START();
      PFM_Update_1(I1_set, ix1_meas_amp);
      bench_total_cy = DWT_STOP();
      bench_total_us = DWT_US(bench_total_cy);

      // PFM_Update_2(I2_set, ix2_meas_amp); // Tam thoi tat
      // PFM_Update_3(I3_set, ix3_meas_amp); // Tam thoi tat
    }

    /* =========================================================
     * GUI GIA TRI QUA UART3 — FULL DUPLEX (TX + RX cung luc)
     * ---------------------------------------------------------
     * UART3 ho tro full-duplex: TX (PD8) va RX (PB11) doc lap.
     * HAL co 2 state rieng: gState (TX) va RxState (RX) nen
     * HAL_UART_Transmit_IT() va HAL_UART_Receive_IT() chay
     * dong thoi ma khong xung dot nhau.
     *
     * Dung HAL_UART_Transmit_IT (non-blocking) thay vi blocking
     * de khong chiem CPU. Buffer phai la static vi IT van dung
     * sau khi ham nay return.
     * ========================================================= */
    static uint32_t uart3_tick = 0U;
    static char uart3_tx_buf[64]; /* static: IT dung buffer nay sau return */

    if ((HAL_GetTick() - uart3_tick) >= 100U &&
        huart3.gState == HAL_UART_STATE_READY) { /* Tranh gui khi dang busy */
      uart3_tick = HAL_GetTick();

      uint16_t len =
          (uint16_t)sprintf(uart3_tx_buf, "Ia=%.2f Ib=%.2f Ic=%.2f\r\n",
                            (float)I1_set, (float)I2_set, (float)I3_set);

      tx_count++; /* debug: dem so lan TX */
      memcpy(tx_snapshot, uart3_tx_buf,
             sizeof(tx_snapshot)); /* debug: luu chuoi gui */
      HAL_UART_Transmit_IT(&huart3, (uint8_t *)uart3_tx_buf, len);
    }

    if (data_ready == 1) {
      /* Luu snapshot truoc khi parse (de xem trong Watch) */
      memcpy(rx_snapshot, rx_buffer, sizeof(rx_buffer));

      char *p = rx_buffer;
      char *endp;

      /* Bo qua ky tu khong phai so ('"', space ...) */
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
            parse_count++; /* debug: parse thanh cong */
          } else {
            parse_fail_count++; /* debug: strtof so 3 fail */
          }
        } else {
          parse_fail_count++; /* debug: khong tim thay dau phay thu 2 */
        }
      } else {
        parse_fail_count++; /* debug: khong tim thay dau phay thu 1 */
      }

      /* Reset bo dem cho goi tiep theo */
      data_ready = 0;
      rx_index = 0;
      memset(rx_buffer, 0, sizeof(rx_buffer));
    }

    /* I1_set, I2_set, I3_set da duoc cap nhat – su dung trong PID loop */
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE; /* Single conversion mode */
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in
   * the sequencer and its sample time.
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

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
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */
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
 * @brief  Tinh xap xi phan so toi gian tot nhat cua delta_d
 *
 * Tim N1, N2 nguyen duong toi thieu sao cho N1/(N1+N2) xap xi delta_d.
 * Thuat toan: duyet mau so (total = N1+N2) tu 1 den PFM_MAX_TOTAL,
 *   - tu_so p = round(delta_d * total)
 *   - tinh sai so |p/total - delta_d|
 *   - chon cap (N1=p, N2=total-p) co sai so nho nhat.
 * => mau so nho nhat dat xap xi tot nhat (phan so toi gian).
 *
 * Vi du:
 *   delta_d = 0.714  =>  total=7, p=5  =>  N1=5, N2=2  (sai so ~0)
 *   delta_d = 0.333  =>  total=3, p=1  =>  N1=1, N2=2
 *   delta_d = 1.0    =>  N1=1, N2=0   (dac biet)
 *   delta_d = 0.5    =>  total=2, p=1  =>  N1=1, N2=1
 */
static void PFM_ComputeN1N2(float delta_d, uint8_t *pN1, uint8_t *pN2) {
  /* Trang thai dac biet: delta_d = 1 => chi dung f_key, khong co f_key/3 */
  if (delta_d >= 1.0f) {
    *pN1 = 1U;
    *pN2 = 0U;
    return;
  }

  uint8_t best_N1 = 1U;
  uint8_t best_N2 = 1U;
  float best_err = 1.0f;

  for (uint8_t total = 1U; total <= (uint8_t)PFM_MAX_TOTAL; total++) {
    /* p = round(delta_d * total), clamp [1, total] */
    uint8_t p = (uint8_t)(delta_d * (float)total + 0.5f);
    if (p < 1U)
      p = 1U;
    if (p > total)
      p = total;

    float dd = (float)p / (float)total;
    float err = dd - delta_d;
    if (err < 0.0f)
      err = -err;

    if (err < best_err) {
      best_err = err;
      best_N1 = p;
      best_N2 = total - p;
    }

    /* Neu sai so bang 0 (chinh xac tuyet doi): dung luon */
    if (best_err < 1e-6f)
      break;
  }

  *pN1 = best_N1;
  *pN2 = best_N2;
}

/**
 * @brief  PFM_Update — Cap nhat N1/N2 tu Iset va Ix_meas qua PID DSP
 *
 * Chuoi tinh toan (paper Eq.6 + Eq.19, n=1):
 *   error   = i_set - i_meas
 *   DeltaU  = arm_pid_f32(&pid_ix, error)   [CMSIS-DSP, FPU ~50 cycles]
 *   U_new   = U_base + DeltaU               [clamp]
 *   delta   = U_new / U_max                 [Eq.6]
 *   delta_d = (3 - 1/delta) / 2            [Eq.19, n=1]
 *   (N1, N2): xap xi delta_d = N1/(N1+N2)  [phan so toi gian, tong min]
 *
 * @param i_set   dong dat (A)
 * @param i_meas  dong do (A) — do thuc tu ADS1115/ACS70331
 */
static void PFM_Update_1(float i_set, float i_meas) {

#ifdef PFM_N1N2_FIXED
  (void)i_set;
  (void)i_meas;
  pfm1_N1 = DBG_N1_1;
  pfm1_N2 = DBG_N2_1;

#else
  float32_t error = (float32_t)(i_set - i_meas);
  DWT_START();
  float32_t du = arm_pid_f32(&pid_1, error);
  bench_pid_cy = DWT_STOP();
  bench_pid_us = DWT_US(bench_pid_cy);

  if (du > PFM_DU_MAX)
    du = PFM_DU_MAX;
  if (du < PFM_DU_MIN)
    du = PFM_DU_MIN;
  pfm1_du = du;

  float u_new = PFM_U_BASE + (float)du;
  if (u_new > PFM_U_MAX)
    u_new = PFM_U_MAX;
  if (u_new < PFM_U_MAX * PFM_DELTA_MIN)
    u_new = PFM_U_MAX * PFM_DELTA_MIN;

  float delta = u_new / PFM_U_MAX;
  float delta_d = (3.0f - 1.0f / delta) / 2.0f;
  if (delta_d < 0.0f)
    delta_d = 0.0f;
  if (delta_d > 1.0f)
    delta_d = 1.0f;

  uint8_t N1, N2;
  DWT_START();
  PFM_ComputeN1N2(delta_d, &N1, &N2);
  bench_pfm_cy = DWT_STOP();

  pfm1_delta = delta;
  pfm1_delta_d = delta_d;
  pfm1_u_new = u_new;
  pfm1_N1 = N1;
  pfm1_N2 = N2;

#endif
}

static void PFM_Update_2(float i_set, float i_meas) {

#ifdef PFM_N1N2_FIXED
  (void)i_set;
  (void)i_meas;
  pfm2_N1 = DBG_N1_2;
  pfm2_N2 = DBG_N2_2;

#else
  float32_t error = (float32_t)(i_set - i_meas);
  float32_t du = arm_pid_f32(&pid_2, error);

  if (du > PFM_DU_MAX)
    du = PFM_DU_MAX;
  if (du < PFM_DU_MIN)
    du = PFM_DU_MIN;
  pfm2_du = du;

  float u_new = PFM_U_BASE + (float)du;
  if (u_new > PFM_U_MAX)
    u_new = PFM_U_MAX;
  if (u_new < PFM_U_MAX * PFM_DELTA_MIN)
    u_new = PFM_U_MAX * PFM_DELTA_MIN;

  float delta = u_new / PFM_U_MAX;
  float delta_d = (3.0f - 1.0f / delta) / 2.0f;
  if (delta_d < 0.0f)
    delta_d = 0.0f;
  if (delta_d > 1.0f)
    delta_d = 1.0f;

  uint8_t N1, N2;
  PFM_ComputeN1N2(delta_d, &N1, &N2);

  pfm2_delta = delta;
  pfm2_delta_d = delta_d;
  pfm2_u_new = u_new;
  pfm2_N1 = N1;
  pfm2_N2 = N2;

#endif
}

static void PFM_Update_3(float i_set, float i_meas) {

#ifdef PFM_N1N2_FIXED
  (void)i_set;
  (void)i_meas;
  pfm3_N1 = DBG_N1_3;
  pfm3_N2 = DBG_N2_3;

#else
  float32_t error = (float32_t)(i_set - i_meas);
  float32_t du = arm_pid_f32(&pid_3, error);

  if (du > PFM_DU_MAX)
    du = PFM_DU_MAX;
  if (du < PFM_DU_MIN)
    du = PFM_DU_MIN;
  pfm3_du = du;

  float u_new = PFM_U_BASE + (float)du;
  if (u_new > PFM_U_MAX)
    u_new = PFM_U_MAX;
  if (u_new < PFM_U_MAX * PFM_DELTA_MIN)
    u_new = PFM_U_MAX * PFM_DELTA_MIN;

  float delta = u_new / PFM_U_MAX;
  float delta_d = (3.0f - 1.0f / delta) / 2.0f;
  if (delta_d < 0.0f)
    delta_d = 0.0f;
  if (delta_d > 1.0f)
    delta_d = 1.0f;

  uint8_t N1, N2;
  PFM_ComputeN1N2(delta_d, &N1, &N2);

  pfm3_delta = delta;
  pfm3_delta_d = delta_d;
  pfm3_u_new = u_new;
  pfm3_N1 = N1;
  pfm3_N2 = N2;

#endif
}

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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

  /* ================================================================
   * BO PFM #1 va #2: TIM3 IRQ (cung ARR, cung tan so)
   * #1: PC6 (A) / PB5 (B)  |  #2: PC8 (A) / PC9 (B)
   * ================================================================ */
  if (htim->Instance == TIM3) {

    /* ---- BO #1: PC6/PB5 ---- */
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

    /* ---- BO #2: PC8/PC9 ---- */
#if 0 // Tam thoi tat PFM 2
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
   * BO PFM #3: TIM1 IRQ
   * PE9 (A) / PE11 (B)
   * ================================================================ */
#if 0 // Tam thoi tat PFM 3
  if (htim->Instance == TIM1) {

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
  }
#endif
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART3) /* UART3 nhan du lieu tu LabVIEW */
  {
    rx_byte_count++; /* debug: dem byte nhan duoc trong ISR */

    uint8_t ch = rx_data;

    if (ch == '\n' || ch == '\r') {
      if (rx_index > 0) {
        rx_buffer[rx_index] = '\0';
        data_ready = 1;
        data_ready_count++; /* debug: dem packet hoan chinh */
      }
    } else {
      /* Luu ky tu vao buffer (tranh tran mang) */
      if (rx_index < (uint8_t)(sizeof(rx_buffer) - 1U)) {
        rx_buffer[rx_index] = ch;
        rx_index++;
      }
    }

    /* Kich hoat lai ngat nhan byte tiep theo */
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
