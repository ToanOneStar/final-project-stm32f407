#include "pfm_control.h"
#include "arm_math.h"
#include <math.h>

/* Private Defines -----------------------------------------------------------*/
#define TIM3_CLK_HZ 84000000UL /* TIM3 input clock = 84 MHz      */
#define PWM_PRESCALER 0U       /* Prescaler = 0                  */
#define PWM_F_KEY 50512L /* Tan so co ban f_key = ~50 kHz   */
#define PWM_ARR_HALF ((uint32_t)(TIM3_CLK_HZ / (2UL * PWM_F_KEY)) - 1U)

#define TIM1_CLK_HZ 84000000UL
#define TIM1_ARR_FKEY ((uint32_t)(TIM1_CLK_HZ / PWM_F_KEY) - 1U)

#define PFM_U_MAX 10.8f     /* U_max (V): 2*sqrt(2)*E/pi, E=12V          */
#define PFM_U_BASE 10.8f    /* U_base khi delta=1 (V)                    */
#define PFM_DU_MIN (-10.8f) /* Gioi han duoi PID output (V) = -U_max   */
#define PFM_DU_MAX 0.0f     /* Gioi han tren: khong che vuot U_max       */
#define PFM_DELTA_MIN 0.3f  /* delta toi thieu (tranh 1/delta -> inf)    */
#define PFM_PID_TS 0.001f   /* Sampling time PID = 1 ms                 */
#define PFM_MAX_TOTAL 12U   /* Gioi han N1+N2 toi da (tuy chinh)         */

#define PFM_N1N2_FIXED /* Comment dong nay de bat lai PID/PFM tu dong */

#define DBG_N1_1 2U
#define DBG_N2_1 1U
#define DBG_N1_2 2U
#define DBG_N2_2 1U
#define DBG_N1_3 2U
#define DBG_N2_3 1U

/* Private Variables ---------------------------------------------------------*/
volatile uint8_t pwm1_group = 0U; /* 0=nhom A, 1=nhom B                   */
volatile uint16_t pwm1_cnt = 0U;  /* dem nua chu ky trong nhom             */
volatile uint8_t pwm1_pol = 0U;   /* cuc hien tai PC6 (0=LOW, 1=HIGH)      */
volatile uint8_t pfm1_N1 = 2U;    /* cap nhat boi PFM_Update_1()           */
volatile uint8_t pfm1_N2 = 1U;

volatile uint8_t pwm2_group = 0U;
volatile uint16_t pwm2_cnt = 0U;
volatile uint8_t pwm2_pol = 0U; /* cuc hien tai PC8 (0=LOW, 1=HIGH)      */
volatile uint8_t pfm2_N1 = 2U;  /* cap nhat boi PFM_Update_2()           */
volatile uint8_t pfm2_N2 = 1U;

volatile uint8_t pwm3_group = 0U;
volatile uint16_t pwm3_cnt = 0U;
volatile uint8_t pwm3_pol = 0U; /* cuc hien tai PE9 (0=LOW, 1=HIGH)      */
volatile uint8_t pfm3_N1 = 2U;  /* cap nhat boi PFM_Update_3()           */
volatile uint8_t pfm3_N2 = 1U;

arm_pid_instance_f32 pid_1; /* PID bo 1: dieu khien I_1 */
arm_pid_instance_f32 pid_2; /* PID bo 2: dieu khien I_2 */
arm_pid_instance_f32 pid_3; /* PID bo 3: dieu khien I_3 */

volatile float pfm1_delta = 1.0f;
volatile float pfm1_delta_d = 1.0f;
volatile float pfm1_u_new = PFM_U_BASE;
volatile float pfm1_du = 0.0f;

volatile float pfm2_delta = 1.0f;
volatile float pfm2_delta_d = 1.0f;
volatile float pfm2_u_new = PFM_U_BASE;
volatile float pfm2_du = 0.0f;

volatile float pfm3_delta = 1.0f;
volatile float pfm3_delta_d = 1.0f;
volatile float pfm3_u_new = PFM_U_BASE;
volatile float pfm3_du = 0.0f;

volatile uint32_t bench_pid_cy = 0U;   
volatile uint32_t bench_pfm_cy = 0U;   
volatile uint32_t bench_total_cy = 0U; 
volatile float bench_pid_us = 0.0f;    
volatile float bench_total_us = 0.0f;  

#define DWT_START()                                                            \
  do {                                                                         \
    DWT->CYCCNT = 0U;                                                          \
  } while (0)
#define DWT_STOP() (DWT->CYCCNT)
#define DWT_US(cy) ((float)(cy) / 168.0f) /* cycles -> microseconds */

/* Private Functions ---------------------------------------------------------*/

static void PFM_ComputeN1N2(float delta_d, uint8_t *pN1, uint8_t *pN2) {
  uint8_t best_N1 = 1, best_N2 = 2;
  float best_err = 1e9f;

  for (uint8_t total = 3; total <= PFM_MAX_TOTAL; total++) {
    for (uint8_t n1 = 1; n1 < total; n1 += 2) {
      uint8_t n2 = total - n1;
      if ((n2 % 2) != 0)
        continue;

      float d_test = (float)n1 / (float)total;
      float err = d_test - delta_d;
      if (err < 0.0f)
        err = -err;

      if (err < best_err) {
        best_err = err;
        best_N1 = n1;
        best_N2 = n2;
      }
    }
    if (best_err < 1e-6f)
      break;
  }

  *pN1 = best_N1;
  *pN2 = best_N2;
}

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

  if (du > PFM_DU_MAX) du = PFM_DU_MAX;
  if (du < PFM_DU_MIN) du = PFM_DU_MIN;
  pfm1_du = du;

  float u_new = PFM_U_BASE + (float)du;
  if (u_new > PFM_U_MAX) u_new = PFM_U_MAX;
  if (u_new < PFM_U_MAX * PFM_DELTA_MIN) u_new = PFM_U_MAX * PFM_DELTA_MIN;

  float delta = u_new / PFM_U_MAX;
  float delta_d = (3.0f - 1.0f / delta) / 2.0f;
  if (delta_d < 0.0f) delta_d = 0.0f;
  if (delta_d > 1.0f) delta_d = 1.0f;

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

  if (du > PFM_DU_MAX) du = PFM_DU_MAX;
  if (du < PFM_DU_MIN) du = PFM_DU_MIN;
  pfm2_du = du;

  float u_new = PFM_U_BASE + (float)du;
  if (u_new > PFM_U_MAX) u_new = PFM_U_MAX;
  if (u_new < PFM_U_MAX * PFM_DELTA_MIN) u_new = PFM_U_MAX * PFM_DELTA_MIN;

  float delta = u_new / PFM_U_MAX;
  float delta_d = (3.0f - 1.0f / delta) / 2.0f;
  if (delta_d < 0.0f) delta_d = 0.0f;
  if (delta_d > 1.0f) delta_d = 1.0f;

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

  if (du > PFM_DU_MAX) du = PFM_DU_MAX;
  if (du < PFM_DU_MIN) du = PFM_DU_MIN;
  pfm3_du = du;

  float u_new = PFM_U_BASE + (float)du;
  if (u_new > PFM_U_MAX) u_new = PFM_U_MAX;
  if (u_new < PFM_U_MAX * PFM_DELTA_MIN) u_new = PFM_U_MAX * PFM_DELTA_MIN;

  float delta = u_new / PFM_U_MAX;
  float delta_d = (3.0f - 1.0f / delta) / 2.0f;
  if (delta_d < 0.0f) delta_d = 0.0f;
  if (delta_d > 1.0f) delta_d = 1.0f;

  uint8_t N1, N2;
  PFM_ComputeN1N2(delta_d, &N1, &N2);

  pfm3_delta = delta;
  pfm3_delta_d = delta_d;
  pfm3_u_new = u_new;
  pfm3_N1 = N1;
  pfm3_N2 = N2;
#endif
}

/* Public Functions ----------------------------------------------------------*/

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

void PFM_Control_Init(void) {
  /* Initialize PID coefficients */
  pid_1.Kp = 2.0f;
  pid_1.Ki = 0.1f;
  pid_1.Kd = 0.0f;
  arm_pid_init_f32(&pid_1, 1);

  pid_2.Kp = 2.0f;
  pid_2.Ki = 0.1f;
  pid_2.Kd = 0.0f;
  arm_pid_init_f32(&pid_2, 1);

  pid_3.Kp = 2.0f;
  pid_3.Ki = 0.1f;
  pid_3.Kd = 0.0f;
  arm_pid_init_f32(&pid_3, 1);

  /* Override TIM AF mode - Configure PFM pins as Output Push-Pull */
  GPIO_InitTypeDef gpio = {0};
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  /* GPIOC: PC6, PC8, PC9 */
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_9;
  HAL_GPIO_Init(GPIOC, &gpio);

  /* GPIOB: PB5 */
  gpio.Pin = GPIO_PIN_5;
  HAL_GPIO_Init(GPIOB, &gpio);

  /* GPIOE: PE9, PE11 */
  gpio.Pin = GPIO_PIN_9 | GPIO_PIN_11;
  HAL_GPIO_Init(GPIOE, &gpio);

  /* Set initial states */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
  
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, GPIO_PIN_SET);

  /* Start PFM Timers to trigger HAL_TIM_PeriodElapsedCallback */
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);
}

void PFM_Control_Update(float i1_set, float i1_meas, 
                        float i2_set, float i2_meas, 
                        float i3_set, float i3_meas) {
  static uint32_t pid_tick = 0;
  uint32_t now_ms = HAL_GetTick();

  if ((now_ms - pid_tick) >= 1U) {
    pid_tick = now_ms;

    DWT_START();
    PFM_Update_1(i1_set, i1_meas);
    bench_total_cy = DWT_STOP();
    bench_total_us = DWT_US(bench_total_cy);

    PFM_Update_2(i2_set, i2_meas);
    PFM_Update_3(i3_set, i3_meas);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* BO PFM #1 & #2: Dung chung TIM3 */
  if (htim->Instance == TIM3) {
    // ---- BO PFM #1: PC6/PB5 ----
    if (pwm1_group == 0) {
      pwm1_pol ^= 1U;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, pwm1_pol ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, pwm1_pol ? GPIO_PIN_RESET : GPIO_PIN_SET);
      pwm1_cnt++;
      if (pwm1_cnt >= pfm1_N1) {
        pwm1_group = 1;
        pwm1_cnt = 0;
      }
    } else {
      pwm1_cnt++;
      if ((pwm1_cnt % 3U) == 0U) {
        pwm1_pol ^= 1U;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, pwm1_pol ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, pwm1_pol ? GPIO_PIN_RESET : GPIO_PIN_SET);
      }
      if (pwm1_cnt >= (pfm1_N2 * 3U)) {
        pwm1_group = 0;
        pwm1_cnt = 0;
      }
    }

    // ---- BO PFM #2: PC8/PC9 ----
    if (pwm2_group == 0) {
      pwm2_pol ^= 1U;
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, pwm2_pol ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, pwm2_pol ? GPIO_PIN_RESET : GPIO_PIN_SET);
      pwm2_cnt++;
      if (pwm2_cnt >= pfm2_N1) {
        pwm2_group = 1;
        pwm2_cnt = 0;
      }
    } else {
      pwm2_cnt++;
      if ((pwm2_cnt % 3U) == 0U) {
        pwm2_pol ^= 1U;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, pwm2_pol ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, pwm2_pol ? GPIO_PIN_RESET : GPIO_PIN_SET);
      }
      if (pwm2_cnt >= (pfm2_N2 * 3U)) {
        pwm2_group = 0;
        pwm2_cnt = 0;
      }
    }
  }

  /* BO PFM #3: TIM1 -> PE9/PE11 */
  if (htim->Instance == TIM1) {
    if (pwm3_group == 0) {
      pwm3_pol ^= 1U;
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, pwm3_pol ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, pwm3_pol ? GPIO_PIN_RESET : GPIO_PIN_SET);
      pwm3_cnt++;
      if (pwm3_cnt >= pfm3_N1) {
        pwm3_group = 1;
        pwm3_cnt = 0;
      }
    } else {
      pwm3_cnt++;
      if ((pwm3_cnt % 3U) == 0U) {
        pwm3_pol ^= 1U;
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, pwm3_pol ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11, pwm3_pol ? GPIO_PIN_RESET : GPIO_PIN_SET);
      }
      if (pwm3_cnt >= (pfm3_N2 * 3U)) {
        pwm3_group = 0;
        pwm3_cnt = 0;
      }
    }
  }
}
