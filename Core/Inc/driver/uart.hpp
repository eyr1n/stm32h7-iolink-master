#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "main.h"

#include "driver/ltc2874.hpp"
#include "iolink/iolink_util.hpp"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

namespace driver {

class UART {
public:
  UART(ltc2874::Port port, iolink::COM com) {
    switch (port) {
      case ltc2874::Port::PORT1:
        huart_ = &huart1;
        break;
      case ltc2874::Port::PORT2:
        huart_ = &huart2;
        break;
      case ltc2874::Port::PORT3:
        huart_ = &huart3;
        break;
      case ltc2874::Port::PORT4:
        huart_ = &huart6;
        break;
    }

    switch (com) {
      case iolink::COM::COM1:
        huart_->Init.BaudRate = 4800;
        break;
      case iolink::COM::COM2:
        huart_->Init.BaudRate = 38400;
        break;
      case iolink::COM::COM3:
        huart_->Init.BaudRate = 230400;
        break;
    }
  }

  void init() {
    HAL_UART_Init(huart_);
    HAL_UART_Receive_DMA(huart_, buf_.data(), buf_.size());
  }

  void deinit() {
    HAL_UART_DeInit(huart_);
  }

  size_t available() {
    size_t write_idx = buf_.size() - __HAL_DMA_GET_COUNTER(huart_->hdmarx);
    return (buf_.size() + write_idx - read_idx_) % buf_.size();
  }

  void transmit(const uint8_t *data, size_t len) {
    HAL_UART_Transmit_DMA(huart_, data, len);
  }

  void receive(uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
      data[i] = buf_[read_idx_];
      advance(1);
    }
  }

  void advance(size_t len) {
    read_idx_ = (read_idx_ + len) % buf_.size();
  }

  void flush() {
    advance(available());
  }

private:
  UART_HandleTypeDef *huart_;
  std::array<uint8_t, 64> buf_;
  size_t read_idx_ = 0;
};

} // namespace driver
