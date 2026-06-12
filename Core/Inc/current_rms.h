/**
 * @file    current_rms.h
 * @brief   Software AC RMS current measurement using ACS70331 Hall-effect
 * sensor
 *
 * The ACS70331 output is a DC-biased AC signal:
 *   Vout = 1.5V_DC + Sensitivity * i(t)
 *
 * RMS of the AC component is computed using the online variance formula
 * (no sample array needed):
 *   Var(x) = mean(x^2) - mean(x)^2
 *   Vrms_AC = sqrt(Var(x))
 *   I_rms   = Vrms_AC / Sensitivity
 *
 * Wiring:
 *   ACS70331 Vout -> STM32 ADC pin (e.g. PA1 = ADC1_CH1)
 *   ACS70331 GND  -> STM32 GND
 *   ACS70331 VCC  -> 3.3V
 */

#ifndef CURRENT_RMS_H
#define CURRENT_RMS_H

#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdint.h>

/* Number of ADC samples per RMS measurement (~200 x 3.7 us = 0.74 ms) */
#define CRMS_SAMPLES 200U

/* ADC reference voltage in mV */
#define CRMS_VREF_MV 3300.0f

/* 12-bit ADC full scale */
#define CRMS_ADC_FULLSCALE 4095.0f

/* Noise floor threshold (mV): ACS70331 idle noise ~12 mV RMS.
 * When Vrms_AC < CRMS_DEADBAND_MV the output is hard-reset to 0 A. */
#define CRMS_DEADBAND_MV 15.0f

/* EMA smoothing factor: 0 = no update, 1 = raw, 0.2 = balanced */
#define CRMS_EMA_ALPHA 0.2f

/** State for one current measurement channel. */
typedef struct {
  float current_A;     /* Filtered RMS current (A)                   */
  float vrms_mV;       /* AC RMS voltage from ADC (mV)               */
  uint16_t dc_mean_mV; /* DC mean (~1500 mV = 1.5 V offset), debug   */
  uint8_t valid;       /* 1 = measurement OK, 0 = ADC timeout        */
} CRMS_Channel_t;

/**
 * @brief Reset a channel struct to safe initial values.
 * @param ch  Pointer to the channel to initialise.
 */
void CRMS_Init(CRMS_Channel_t *ch);

/**
 * @brief Update one channel from interleaved DMA buffer.
 *
 * @param ch                  Channel state struct.
 * @param dma_buf             Pointer to the start of the DMA buffer.
 * @param total_channels      Total number of interleaved channels in the buffer.
 * @param channel_offset      Offset index for this specific channel.
 * @param samples_per_channel Number of samples to read for this channel.
 * @param sensitivity_mV_A    Sensor sensitivity in mV/A.
 */
void CRMS_Update(CRMS_Channel_t *ch, const uint16_t *dma_buf,
                 uint16_t total_channels, uint16_t channel_offset,
                 uint16_t samples_per_channel, float sensitivity_mV_A);

#endif /* CURRENT_RMS_H */
