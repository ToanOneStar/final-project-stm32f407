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

/* DMA Buffer: Since MX configured NbrOfConversion=3 for CH1, we need 3x size */
static uint16_t adc_dma_buf[SW_RMS_N * 3];
static volatile uint8_t adc_dma_complete = 0;

/* Public functions ----------------------------------------------------------*/

void CurrentSensor_Init(ADC_HandleTypeDef* hadc) {
    p_hadc = hadc;
    /* Start the first DMA transfer */
    HAL_ADC_Start_DMA(p_hadc, (uint32_t*)adc_dma_buf, SW_RMS_N * 3);
}

void CurrentSensor_Process(void) {
    if (p_hadc == NULL) return;

    if ((HAL_GetTick() - adc_tick) >= CURRENT_MEAS_MS) {
        adc_tick = HAL_GetTick();

        /* Only process if DMA has finished collecting SW_RMS_N samples */
        if (adc_dma_complete) {
            adc_dma_complete = 0; /* Reset flag */

            float sw_sum = 0.0f;
            float sw_sum_sq = 0.0f;

            /* Process the buffer. Since NbrOfConversion=3, we step by 3 */
            for (uint16_t i = 0; i < SW_RMS_N * 3; i += 3) {
                float v = (float)adc_dma_buf[i] * 3300.0f / 4095.0f;
                sw_sum += v;
                sw_sum_sq += v * v;
            }

            float sw_mean = sw_sum / (float)SW_RMS_N;
            float sw_var = (sw_sum_sq / (float)SW_RMS_N) - (sw_mean * sw_mean);
            float sw_vrms = (sw_var > 0.0f) ? sqrtf(sw_var) : 0.0f;

            float new_current = sw_vrms / ACS_SENS_MV_A_1;

            if (sw_vrms < 5.0f) {
                new_current = 0.0f;
            }

            g_current_A = (g_current_A * 0.8f) + (new_current * 0.2f);
            ix1_meas_amp = g_current_A;
            
            ix2_meas_amp = 0.0f;
            ix3_meas_amp = 0.0f;

            /* Restart DMA for the next batch */
            HAL_ADC_Start_DMA(p_hadc, (uint32_t*)adc_dma_buf, SW_RMS_N * 3);
        }
    }
}

/* DMA Complete Callback */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
    if (hadc == p_hadc) {
        adc_dma_complete = 1;
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
