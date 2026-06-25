#ifndef CURRENT_SENSOR_H
#define CURRENT_SENSOR_H

#include "main.h"

/**
 * @brief Initialize the Current Sensor module.
 * @param hadc Pointer to the ADC handle used for measurements.
 */
void CurrentSensor_Init(ADC_HandleTypeDef* hadc);

/**
 * @brief Non-blocking process loop for measuring SW RMS current.
 */
void CurrentSensor_Process(void);

/**
 * @brief Get the measured current for channel 1.
 * @return Current in Amperes.
 */
float CurrentSensor_GetI1(void);

/**
 * @brief Get the measured current for channel 2.
 * @return Current in Amperes.
 */
float CurrentSensor_GetI2(void);

/**
 * @brief Get the measured current for channel 3.
 * @return Current in Amperes.
 */
float CurrentSensor_GetI3(void);

#endif /* CURRENT_SENSOR_H */
