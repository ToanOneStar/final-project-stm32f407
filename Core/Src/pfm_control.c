/**
 * @file    pfm_control.c
 * @brief   PFM controller implementation
 */

#include "pfm_control.h"

/**
 * @brief Compute best rational approximation for delta_d
 *
 * Finds integers N1, N2 such that N1/(N1+N2) approximates delta_d.
 *
 * @param delta_d Target duty ratio
 * @param pN1 Pointer to store N1
 * @param pN2 Pointer to store N2
 */
static void PFM_ComputeN1N2(float delta_d, uint8_t *pN1, uint8_t *pN2) {
  if (delta_d >= 1.0f) {
    *pN1 = 1U;
    *pN2 = 0U;
    return;
  }

  uint8_t best_N1 = 1U;
  uint8_t best_N2 = 1U;
  float best_err = 1.0f;

  for (uint8_t total = 1; total <= PFM_MAX_TOTAL; total++) {
    float fp = delta_d * (float)total;
    uint8_t p = (uint8_t)(fp + 0.5f);
    
    if (p == 0) {
      p = 1;
    }
    if (p > total) {
      p = total;
    }

    float err = fp - (float)p;
    if (err < 0.0f) {
      err = -err;
    }

    if (err < best_err) {
      best_err = err;
      best_N1 = p;
      best_N2 = total - p;
    }

    if (best_err < 1e-6f) {
      break;
    }
  }

  *pN1 = best_N1;
  *pN2 = best_N2;
}

void PFM_Init(PFM_Controller_t *pfm, float kp, float ki, float kd) {
  pfm->pid.Kp = kp;
  pfm->pid.Ki = ki;
  pfm->pid.Kd = kd;
  arm_pid_init_f32(&pfm->pid, 1);
  
  pfm->u_new = PFM_U_BASE;
  pfm->du = 0.0f;
  pfm->delta = 1.0f;
  pfm->delta_d = 1.0f;
  pfm->n1 = 2U;
  pfm->n2 = 1U;
}

void PFM_Update(PFM_Controller_t *pfm, float i_set, float i_meas) {
  float32_t error = (float32_t)(i_set - i_meas);
  float32_t du = arm_pid_f32(&pfm->pid, error);

  if (du > PFM_DU_MAX) {
    du = PFM_DU_MAX;
  }
  if (du < PFM_DU_MIN) {
    du = PFM_DU_MIN;
  }
  pfm->du = du;

  float u_new = PFM_U_BASE + (float)du;
  if (u_new > PFM_U_MAX) {
    u_new = PFM_U_MAX;
  }
  if (u_new < PFM_U_MAX * PFM_DELTA_MIN) {
    u_new = PFM_U_MAX * PFM_DELTA_MIN;
  }
  pfm->u_new = u_new;

  float delta = u_new / PFM_U_MAX;
  float delta_d = (3.0f - 1.0f / delta) / 2.0f;
  
  if (delta_d < 0.0f) {
    delta_d = 0.0f;
  }
  if (delta_d > 1.0f) {
    delta_d = 1.0f;
  }

  pfm->delta = delta;
  pfm->delta_d = delta_d;

  PFM_ComputeN1N2(delta_d, &pfm->n1, &pfm->n2);
}
