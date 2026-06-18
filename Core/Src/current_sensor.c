#include "current_sensor.h"
#include <math.h>

/* Private defines -----------------------------------------------------------*/
#define ACS_SENS_MV_A_1 225.0f
#define CURRENT_MEAS_MS 10U
#define SW_RMS_N 200U

/* Private variables ---------------------------------------------------------*/
static ADC_HandleTypeDef* p_hadc = NULL;
static float ix1_meas_amp = 0.0f;
static float ix2_meas_amp = 0.0f;
static float ix3_meas_amp = 0.0f;
static float g_current_A = 0.0f;
static uint32_t adc_tick = 0;

/* Public functions ----------------------------------------------------------*/

void CurrentSensor_Init(ADC_HandleTypeDef* hadc) {
    p_hadc = hadc;
}

void CurrentSensor_Process(void) {
    if (p_hadc == NULL) return;

    if ((HAL_GetTick() - adc_tick) >= CURRENT_MEAS_MS) {
        adc_tick = HAL_GetTick();

        float sw_sum = 0.0f;
        float sw_sum_sq = 0.0f;

        for (uint16_t i = 0; i < SW_RMS_N; i++) {
            HAL_ADC_Start(p_hadc);
            if (HAL_ADC_PollForConversion(p_hadc, 5U) == HAL_OK) {
                float v = (float)HAL_ADC_GetValue(p_hadc) * 3300.0f / 4095.0f;
                sw_sum += v;
                sw_sum_sq += v * v;
            }
            HAL_ADC_Stop(p_hadc);
        }

        float sw_mean = sw_sum / (float)SW_RMS_N;
        /* Variance of AC component: Var = mean(x^2) - mean(x)^2 */
        float sw_var = (sw_sum_sq / (float)SW_RMS_N) - (sw_mean * sw_mean);
        float sw_vrms = (sw_var > 0.0f) ? sqrtf(sw_var) : 0.0f; /* mV AC RMS */

        float new_current = sw_vrms / ACS_SENS_MV_A_1;

        /* Deadband: if RMS noise < 5mV then 0A */
        if (sw_vrms < 5.0f) {
            new_current = 0.0f;
        }

        /* EMA filter (alpha=0.2) */
        g_current_A = (g_current_A * 0.8f) + (new_current * 0.2f);
        ix1_meas_amp = g_current_A;
        
        /* Currently only 1 sensor is implemented with ADC1. 
         * Mocking I2 and I3 for the rest. */
        ix2_meas_amp = 0.0f;
        ix3_meas_amp = 0.0f;
    }
}

float CurrentSensor_GetI1(void) {
    return ix1_meas_amp;
}

float CurrentSensor_GetI2(void) {
    return ix2_meas_amp;
}

float CurrentSensor_GetI3(void) {
    return ix3_meas_amp;
}
