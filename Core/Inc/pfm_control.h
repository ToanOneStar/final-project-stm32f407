/**
 * @file    pfm_control.h
 * @brief   PFM (Pulse Frequency Modulation) controller with PID loop
 */

#ifndef PFM_CONTROL_H
#define PFM_CONTROL_H

#include "arm_math.h"
#include <stdint.h>

/* =================================================================
 * PFM System Constants
 * =================================================================
 * U_max = 2*sqrt(2)*E/pi (E = DC source voltage, e.g., 12V)
 * delta = U_new / U_max (in range 0 to 1]
 * delta_d = (3 - 1/delta) / 2
 * ================================================================= */
#define PFM_U_MAX      10.8f    /* Max voltage (V) */
#define PFM_U_BASE     10.8f    /* Base voltage when delta=1 (V) */
#define PFM_DU_MIN    (-10.8f)  /* Min PID output (V) */
#define PFM_DU_MAX     0.0f     /* Max PID output (V) */
#define PFM_DELTA_MIN  0.3f     /* Min delta to prevent 1/delta -> infinity */
#define PFM_MAX_TOTAL  12U      /* Max N1+N2 limit */

/** 
 * @brief PFM Controller State 
 */
typedef struct {
  arm_pid_instance_f32 pid; /* CMSIS-DSP PID instance */
  float u_new;              /* Computed control voltage */
  float du;                 /* PID output */
  float delta;              /* Voltage ratio */
  float delta_d;            /* Duty ratio for N1/N2 */
  uint8_t n1;               /* Number of fast cycles (group A) */
  uint8_t n2;               /* Number of slow cycles (group B) */
} PFM_Controller_t;

/**
 * @brief Initialize PFM controller
 * @param pfm Pointer to controller instance
 * @param kp Proportional gain
 * @param ki Integral gain
 * @param kd Derivative gain
 */
void PFM_Init(PFM_Controller_t *pfm, float kp, float ki, float kd);

/**
 * @brief Update PFM controller (PID + N1/N2 calculation)
 * @param pfm Pointer to controller instance
 * @param i_set Setpoint current (A)
 * @param i_meas Measured current (A)
 */
void PFM_Update(PFM_Controller_t *pfm, float i_set, float i_meas);

#endif /* PFM_CONTROL_H */
