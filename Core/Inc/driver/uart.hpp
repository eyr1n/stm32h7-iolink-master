#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "main.h"

namespace driver {

class UART {
public:
  void init(UART_HandleTypeDef *huart) {
    huart_ = huart;
    HAL_UART_Receive_DMA(huart_, buf_.data(), buf_.size());
  }

  void transmit(const uint8_t *data, size_t size) {
    HAL_UART_Transmit(huart_, data, size, 10);
  }

  size_t available() {
    size_t write_idx = buf_.size() - __HAL_DMA_GET_COUNTER(huart_->hdmarx);
    size_t res = (write_idx < read_idx_) ? (write_idx + buf_.size() - read_idx_) : (write_idx - read_idx_);
    return res;
  }

  uint8_t read() {
    uint8_t res = buf_[read_idx_];
    read_idx_++;
    if (read_idx_ >= buf_.size()) {
      read_idx_ = 0;
    }
    return res;
  }

  void flush() {
    while (available() > 0) {
      read();
    }
  }

private:
  UART_HandleTypeDef *huart_;
  std::array<uint8_t, 64> buf_;
  size_t read_idx_ = 0;
};

} // namespace driver
