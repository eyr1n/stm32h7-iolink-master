#pragma once

#include <cstdint>

#include "main.h"

#include "ltc2874/ltc2874.hpp"

extern SPI_HandleTypeDef hspi3;

namespace driver {

class LTC2874Driver {
public:
  void spi_cs_low() {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
  }

  void spi_cs_high() {
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
  }

  uint8_t spi_transmit(uint8_t data) {
    uint8_t res;
    HAL_SPI_TransmitReceive(&hspi3, &data, &res, 1, 10);
    return res;
  }
};

inline LTC2874Driver ltc2874_driver;
inline ltc2874::LTC2874 ltc2874{ltc2874_driver};

} // namespace driver
