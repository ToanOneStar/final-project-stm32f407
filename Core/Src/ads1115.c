#include "ads1115.h"
#include "stm32f4xx_hal.h" // Could be changed for a specific processor.
#include "string.h"

/* Variables */
uint8_t ADS1115_devAddress = 0b1001000; // 7 bit address, without R/W' bit.

I2C_HandleTypeDef ADS1115_I2C_Handler; // HAL I2C handler store variable.

uint16_t ADS1115_dataRate = ADS1115_DATA_RATE_128; // Default
uint16_t ADS1115_pga = ADS1115_PGA_TWO;            // Default
uint16_t ADS1115_port = ADS1115_MUX_AIN0;          // Default

uint8_t ADS1115_config[2];
uint8_t ADS1115_rawValue[2];

/* Function definitions. */
HAL_StatusTypeDef ADS1115_Init(I2C_HandleTypeDef *handler, uint16_t setDataRate,
                               uint16_t setPGA) {
  memcpy(&ADS1115_I2C_Handler, handler, sizeof(*handler));

  ADS1115_dataRate = setDataRate;
  ADS1115_pga = setPGA;

  if (HAL_I2C_IsDeviceReady(&ADS1115_I2C_Handler,
                            (uint16_t)(ADS1115_devAddress << 1), 5,
                            ADS1115_TIMEOUT) == HAL_OK) {
    return HAL_OK;
  } else {
    return HAL_ERROR;
  }
}

HAL_StatusTypeDef ADS1115_readSingleEnded(uint16_t muxPort,
                                          uint16_t *rawValue) {

  ADS1115_config[0] = ADS1115_OS | muxPort | ADS1115_pga | ADS1115_MODE;
  ADS1115_config[1] = ADS1115_dataRate | ADS1115_COMP_MODE | ADS1115_COMP_POL |
                      ADS1115_COMP_LAT | ADS1115_COMP_QUE;
  uint8_t waiting = 1;
  uint16_t cnt = 0;

  if (HAL_I2C_Mem_Write(&ADS1115_I2C_Handler,
                        (uint16_t)(ADS1115_devAddress << 1), ADS1115_CONFIG_REG,
                        1, ADS1115_config, 2, ADS1115_TIMEOUT) == HAL_OK) {
    while (waiting) // Checking Data Ready
    {
      if (HAL_I2C_Mem_Read(&ADS1115_I2C_Handler,
                           (uint16_t)((ADS1115_devAddress << 1) | 0x1),
                           ADS1115_CONFIG_REG, 1, ADS1115_config, 2,
                           ADS1115_TIMEOUT) == HAL_OK) {
        if (ADS1115_config[0] & ADS1115_OS)
          waiting = 0;
      } else
        return HAL_ERROR;
      if (++cnt == 100)
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Read(&ADS1115_I2C_Handler,
                         (uint16_t)((ADS1115_devAddress << 1) | 0x1),
                         ADS1115_CONVER_REG, 1, ADS1115_rawValue, 2,
                         ADS1115_TIMEOUT) == HAL_OK) {
      /* Ghep 2 byte thanh gia tri uint16_t khong dau (0 - 65535) */
      *rawValue = (uint16_t)((ADS1115_rawValue[0] << 8) | ADS1115_rawValue[1]);
      return HAL_OK;
    }
  }

  return HAL_ERROR;
}