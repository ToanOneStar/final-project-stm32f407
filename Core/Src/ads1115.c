#include "ads1115.h"
#include "stm32f4xx_hal.h" // Could be changed for a specific processor.
#include "string.h"

/* Variables */
uint8_t ADS1115_devAddress = 0b1001000; // 7 bit address, without R/W' bit.

uint16_t ADS1115_dataRate = ADS1115_DATA_RATE_128; // Default
uint16_t ADS1115_pga = ADS1115_PGA_TWO;            // Default

/* Function definitions. */
HAL_StatusTypeDef ADS1115_Init(I2C_HandleTypeDef *hi2c, uint16_t setDataRate,
                               uint16_t setPGA) {
  ADS1115_dataRate = setDataRate;
  ADS1115_pga = setPGA;

  if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(ADS1115_devAddress << 1), 5,
                            ADS1115_TIMEOUT) == HAL_OK) {
    return HAL_OK;
  } else {
    return HAL_ERROR;
  }
}

HAL_StatusTypeDef ADS1115_readSingleEnded(I2C_HandleTypeDef *hi2c,
                                          uint16_t muxPort,
                                          uint16_t *rawValue) {
  uint8_t cfg[2];
  uint8_t raw[2];
  uint16_t addr_w = (uint16_t)(ADS1115_devAddress << 1);
  uint16_t addr_r = (uint16_t)((ADS1115_devAddress << 1) | 0x1);

  cfg[0] = ADS1115_OS | muxPort | ADS1115_pga | ADS1115_MODE;
  cfg[1] = ADS1115_dataRate | ADS1115_COMP_MODE | ADS1115_COMP_POL |
           ADS1115_COMP_LAT | ADS1115_COMP_QUE;

  /* Ghi Config Register */
  if (HAL_I2C_Mem_Write(hi2c, addr_w, ADS1115_CONFIG_REG, 1, cfg, 2,
                        ADS1115_TIMEOUT) != HAL_OK) {
    return HAL_ERROR;
  }

  /* Doi conversion hoan tat: 128SPS => ~8ms/conv; them delay truoc khi poll */
  HAL_Delay(10); /* An toan: cho ADS1115 hoan thanh conversion */
  uint16_t cnt = 0;
  uint8_t waiting = 1;
  while (waiting) {
    if (HAL_I2C_Mem_Read(hi2c, addr_r, ADS1115_CONFIG_REG, 1, cfg, 2,
                         ADS1115_TIMEOUT) == HAL_OK) {
      if (cfg[0] & ADS1115_OS)
        waiting = 0;
    } else {
      return HAL_ERROR;
    }
    if (++cnt == 100)
      return HAL_ERROR;
  }

  /* Doc Conversion Register */
  if (HAL_I2C_Mem_Read(hi2c, addr_r, ADS1115_CONVER_REG, 1, raw, 2,
                       ADS1115_TIMEOUT) == HAL_OK) {
    *rawValue = (uint16_t)((raw[0] << 8) | raw[1]);
    return HAL_OK;
  }

  return HAL_ERROR;
}