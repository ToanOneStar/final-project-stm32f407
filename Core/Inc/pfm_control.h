#ifndef PFM_CONTROL_H
#define PFM_CONTROL_H

#include "main.h"

/**
 * @brief Initialize the PFM control module (e.g. initialize PID instances).
 */
void PFM_Control_Init(void);

/**
 * @brief Update the PFM parameters based on current measurements and setpoints.
 * @param i1_set Setpoint for current 1
 * @param i1_meas Measured current 1
 * @param i2_set Setpoint for current 2
 * @param i2_meas Measured current 2
 * @param i3_set Setpoint for current 3
 * @param i3_meas Measured current 3
 */
void PFM_Control_Update(float i1_set, float i1_meas, 
                        float i2_set, float i2_meas, 
                        float i3_set, float i3_meas);

#endif /* PFM_CONTROL_H */
