#ifndef __SSD1306_CONF_H__
#define __SSD1306_CONF_H__

// Choose a microcontroller family
#define STM32F4

// Choose a bus
#define SSD1306_USE_SPI

// SPI Configuration
#define SSD1306_SPI_PORT        hspi1
#define SSD1306_CS_Port         GPIOB
#define SSD1306_CS_Pin          GPIO_PIN_12
#define SSD1306_DC_Port         GPIOB
#define SSD1306_DC_Pin          GPIO_PIN_14
#define SSD1306_Reset_Port      GPIOA
#define SSD1306_Reset_Pin       GPIO_PIN_8

// Include only needed fonts
#define SSD1306_INCLUDE_FONT_6x8
#define SSD1306_INCLUDE_FONT_7x10
#define SSD1306_INCLUDE_FONT_11x18
#define SSD1306_INCLUDE_FONT_16x26

#endif /* __SSD1306_CONF_H__ */
