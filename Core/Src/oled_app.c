#include "oled_app.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static uint8_t current_page = 0;
static float Rx = 45.1234f, Ry = 90.5678f, Rz = 180.9012f;
static uint32_t last_btn_press = 0;
static uint8_t last_btn_state = 0;
static uint32_t oled_tick = 0;

/* Private functions ---------------------------------------------------------*/
/**
 * @brief Convert float to string with 4 decimal places.
 */
static void float_to_str(float val, char* buf) {
    if (val < 0) {
        *buf++ = '-';
        val = -val;
    }
    int int_part = (int)val;
    int frac_part = (int)((val - int_part) * 10000);
    
    char int_str[10];
    int i = 0;
    if (int_part == 0) {
        int_str[i++] = '0';
    } else {
        while (int_part > 0) {
            int_str[i++] = (int_part % 10) + '0';
            int_part /= 10;
        }
    }
    while (i > 0) {
        *buf++ = int_str[--i];
    }
    
    *buf++ = '.';
    buf[3] = (frac_part % 10) + '0'; frac_part /= 10;
    buf[2] = (frac_part % 10) + '0'; frac_part /= 10;
    buf[1] = (frac_part % 10) + '0'; frac_part /= 10;
    buf[0] = (frac_part % 10) + '0';
    buf += 4;
    *buf = '\0';
}

/* Public functions ----------------------------------------------------------*/

void OLED_App_Init(void) {
    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();
}

void OLED_App_Process(float i1, float i2, float i3) {
    /* Non-blocking button read with debounce */
    uint8_t current_btn_state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
    if (current_btn_state == GPIO_PIN_SET && last_btn_state == GPIO_PIN_RESET) {
        if (HAL_GetTick() - last_btn_press > 200) {
            current_page = (current_page + 1) % 3;
            last_btn_press = HAL_GetTick();
        }
    }
    last_btn_state = current_btn_state;

    /* Non-blocking OLED screen update */
    if (HAL_GetTick() - oled_tick > 100) {
        oled_tick = HAL_GetTick();
        char buf1[32]; 
        char buf2[32]; 
        char val_str[16];
        
        ssd1306_Fill(Black);
        
        if (current_page == 0) {
            float_to_str(i1, val_str);
            strcpy(buf1, "I1: "); strcat(buf1, val_str); strcat(buf1, " A");
            float_to_str(i2, val_str);
            strcpy(buf2, "I2: "); strcat(buf2, val_str); strcat(buf2, " A");
        } 
        else if (current_page == 1) {
            float_to_str(i3, val_str);
            strcpy(buf1, "I3: "); strcat(buf1, val_str); strcat(buf1, " A");
            float_to_str(Rx, val_str);
            strcpy(buf2, "Rx: "); strcat(buf2, val_str); strcat(buf2, " deg");
        } 
        else {
            float_to_str(Ry, val_str);
            strcpy(buf1, "Ry: "); strcat(buf1, val_str); strcat(buf1, " deg");
            float_to_str(Rz, val_str);
            strcpy(buf2, "Rz: "); strcat(buf2, val_str); strcat(buf2, " deg");
        }
        
        ssd1306_SetCursor(5, 15);
        ssd1306_WriteString(buf1, Font_7x10, White);
        
        ssd1306_SetCursor(5, 40);
        ssd1306_WriteString(buf2, Font_7x10, White);
        
        ssd1306_UpdateScreen();
        
        /* Mock angle rotation */
        Rx += 0.0005f; 
        Ry += 0.0005f; 
        Rz += 0.0005f;
    }
}
