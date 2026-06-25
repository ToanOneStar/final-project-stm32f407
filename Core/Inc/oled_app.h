#ifndef OLED_APP_H
#define OLED_APP_H

#include "main.h"
#include <stdint.h>

/**
 * @brief Initialize the OLED display.
 */
void OLED_App_Init(void);

/**
 * @brief Non-blocking process loop for polling buttons and updating the OLED screen.
 * @param i1 Measured current 1 (A)
 * @param i2 Measured current 2 (A)
 * @param i3 Measured current 3 (A)
 */
void OLED_App_Process(float i1, float i2, float i3);

#endif /* OLED_APP_H */
