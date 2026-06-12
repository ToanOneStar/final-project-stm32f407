/**
 * @file    current_rms.c
 * @brief   Software AC RMS current measurement using ACS70331 and STM32 ADC.
 *
 * See current_rms.h for algorithm description and wiring guide.
 */

#include "current_rms.h"

void CRMS_Init(CRMS_Channel_t *ch) {
  ch->current_A = 0.0f;
  ch->vrms_mV = 0.0f;
  ch->dc_mean_mV = 0U;
  ch->valid = 0U;
}

void CRMS_Update(CRMS_Channel_t *ch, const uint16_t *dma_buf,
                 uint16_t total_channels, uint16_t channel_offset,
                 uint16_t samples_per_channel, float sensitivity_mV_A) {

  /* Accumulate sum and sum-of-squares for the online variance formula. */
  float sum = 0.0f;
  float sum_sq = 0.0f;

  for (uint16_t i = 0; i < samples_per_channel; i++) {
    float v = (float)dma_buf[i * total_channels + channel_offset] * CRMS_VREF_MV / CRMS_ADC_FULLSCALE;
    sum += v;
    sum_sq += v * v;
  }

  uint16_t n = samples_per_channel;

  if (n == 0U) {
    /* ADC did not return any valid samples — keep previous output. */
    ch->valid = 0U;
    return;
  }

  ch->valid = 1U;

  /* Variance = mean(x^2) - mean(x)^2  →  this equals the squared AC RMS. */
  float fn = (float)n;
  float mean = sum / fn;
  float var = (sum_sq / fn) - (mean * mean);
  float vrms = (var > 0.0f) ? sqrtf(var) : 0.0f;

  ch->vrms_mV = vrms;
  ch->dc_mean_mV = (uint16_t)mean; /* ~1500 mV when ACS70331 is idle */

  /* Deadband: ACS70331 idle noise floor is ~12 mV RMS.
   * Hard-reset to 0 A when below threshold to avoid EMA holdover. */
  if (vrms < CRMS_DEADBAND_MV) {
    ch->current_A = 0.0f;
    return;
  }

  /* EMA low-pass filter applied only when a real signal is present. */
  float new_i = vrms / sensitivity_mV_A;
  ch->current_A =
      ch->current_A * (1.0f - CRMS_EMA_ALPHA) + new_i * CRMS_EMA_ALPHA;
}
