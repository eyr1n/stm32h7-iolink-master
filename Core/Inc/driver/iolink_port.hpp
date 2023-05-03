#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "main.h"

#include "driver/ltc2874.hpp"
#include "driver/uart.hpp"
#include "iolink/iolink_util.hpp"

extern SPI_HandleTypeDef hspi3;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;

namespace driver {

class IOLinkPortDriver {
public:
  IOLinkPortDriver(ltc2874::Port port, iolink::COM com) : port_(port), com_(com) {
    switch (port_) {
      case ltc2874::Port::PORT1:
        huart_ = &huart1;
        txen_port_ = TXEN1_GPIO_Port;
        txen_pin_ = TXEN1_Pin;
        break;
      case ltc2874::Port::PORT2:
        huart_ = &huart2;
        txen_port_ = TXEN2_GPIO_Port;
        txen_pin_ = TXEN2_Pin;
        break;
      case ltc2874::Port::PORT3:
        huart_ = &huart3;
        txen_port_ = TXEN3_GPIO_Port;
        txen_pin_ = TXEN3_Pin;
        break;
      case ltc2874::Port::PORT4:
        huart_ = &huart6;
        txen_port_ = TXEN4_GPIO_Port;
        txen_pin_ = TXEN4_Pin;
        break;
    }

    switch (com_) {
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
    switch (port_) {
      case ltc2874::Port::PORT1:
        driver::ltc2874.write_bit_raise(ltc2874::Register::CTRL2, ltc2874::CTRL2_Bit::ENLP1);
        switch (com_) {
          case iolink::COM::COM1:
            driver::ltc2874.write_bit_lower(ltc2874::Register::MODE2, ltc2874::MODE2_Bit::SLEW1);
            driver::ltc2874.write_value(ltc2874::Register::NSF, ltc2874::NSF_Bit::NSF1, 1);
            break;
          case iolink::COM::COM2:
            driver::ltc2874.write_bit_lower(ltc2874::Register::MODE2, ltc2874::MODE2_Bit::SLEW1);
            driver::ltc2874.write_value(ltc2874::Register::NSF, ltc2874::NSF_Bit::NSF1, 2);
            break;
          case iolink::COM::COM3:
            driver::ltc2874.write_value(ltc2874::Register::NSF, ltc2874::NSF_Bit::NSF1, 3);
            break;
        }
        break;

      case ltc2874::Port::PORT2:
        driver::ltc2874.write_bit_raise(ltc2874::Register::CTRL2, ltc2874::CTRL2_Bit::ENLP2);
        switch (com_) {
          case iolink::COM::COM1:
            driver::ltc2874.write_bit_lower(ltc2874::Register::MODE2, ltc2874::MODE2_Bit::SLEW2);
            driver::ltc2874.write_value(ltc2874::Register::NSF, ltc2874::NSF_Bit::NSF2, 1);
            break;
          case iolink::COM::COM2:
            driver::ltc2874.write_bit_lower(ltc2874::Register::MODE2, ltc2874::MODE2_Bit::SLEW2);
            driver::ltc2874.write_value(ltc2874::Register::NSF, ltc2874::NSF_Bit::NSF2, 2);
            break;
          case iolink::COM::COM3:
            driver::ltc2874.write_value(ltc2874::Register::NSF, ltc2874::NSF_Bit::NSF2, 3);
            break;
        }
        break;

      case ltc2874::Port::PORT3:
        driver::ltc2874.write_bit_raise(ltc2874::Register::CTRL2, ltc2874::CTRL2_Bit::ENLP3);
        switch (com_) {
          case iolink::COM::COM1:
            driver::ltc2874.write_bit_lower(ltc2874::Register::MODE2, ltc2874::MODE2_Bit::SLEW3);
            driver::ltc2874.write_value(ltc2874::Register::NSF, ltc2874::NSF_Bit::NSF3, 1);
            break;
          case iolink::COM::COM2:
            driver::ltc2874.write_bit_lower(ltc2874::Register::MODE2, ltc2874::MODE2_Bit::SLEW3);
            driver::ltc2874.write_value(ltc2874::Register::NSF, ltc2874::NSF_Bit::NSF3, 2);
            break;
          case iolink::COM::COM3:
            driver::ltc2874.write_value(ltc2874::Register::NSF, ltc2874::NSF_Bit::NSF3, 3);
            break;
        }
        break;

      case ltc2874::Port::PORT4:
        driver::ltc2874.write_bit_raise(ltc2874::Register::CTRL2, ltc2874::CTRL2_Bit::ENLP4);
        switch (com_) {
          case iolink::COM::COM1:
            driver::ltc2874.write_bit_lower(ltc2874::Register::MODE2, ltc2874::MODE2_Bit::SLEW4);
            driver::ltc2874.write_value(ltc2874::Register::NSF, ltc2874::NSF_Bit::NSF4, 1);
            break;
          case iolink::COM::COM2:
            driver::ltc2874.write_bit_lower(ltc2874::Register::MODE2, ltc2874::MODE2_Bit::SLEW4);
            driver::ltc2874.write_value(ltc2874::Register::NSF, ltc2874::NSF_Bit::NSF4, 2);
            break;
          case iolink::COM::COM3:
            driver::ltc2874.write_value(ltc2874::Register::NSF, ltc2874::NSF_Bit::NSF4, 3);
            break;
        }
        break;
    }
  }

  void wakeup_request() {
    HAL_UART_DeInit(huart_);
    HAL_GPIO_WritePin(txen_port_, txen_pin_, GPIO_PIN_RESET);
    switch (port_) {
      case ltc2874::Port::PORT1:
        driver::ltc2874.write_bit_lower(ltc2874::Register::EVENT3, ltc2874::EVENT3_Bit::TOC_CQ1, true);
        driver::ltc2874.write_bit_raise(ltc2874::Register::CTRL1, ltc2874::CTRL1_Bit::WKUP1, true);
        break;
      case ltc2874::Port::PORT2:
        driver::ltc2874.write_bit_lower(ltc2874::Register::EVENT3, ltc2874::EVENT3_Bit::TOC_CQ2, true);
        driver::ltc2874.write_bit_raise(ltc2874::Register::CTRL1, ltc2874::CTRL1_Bit::WKUP2, true);
        break;
      case ltc2874::Port::PORT3:
        driver::ltc2874.write_bit_lower(ltc2874::Register::EVENT3, ltc2874::EVENT3_Bit::TOC_CQ3, true);
        driver::ltc2874.write_bit_raise(ltc2874::Register::CTRL1, ltc2874::CTRL1_Bit::WKUP3, true);
        break;
      case ltc2874::Port::PORT4:
        driver::ltc2874.write_bit_lower(ltc2874::Register::EVENT3, ltc2874::EVENT3_Bit::TOC_CQ4, true);
        driver::ltc2874.write_bit_raise(ltc2874::Register::CTRL1, ltc2874::CTRL1_Bit::WKUP1, true);
        break;
    }
    HAL_Delay(1);
    HAL_UART_Init(huart_);
    uart_.init(huart_);
  }

  template <size_t N> void mseq_transmit(const std::array<uint8_t, N> &mseq) {
    uart_.flush();
    HAL_GPIO_WritePin(txen_port_, txen_pin_, GPIO_PIN_SET);
    uart_.transmit(mseq.data(), N);
    HAL_GPIO_WritePin(txen_port_, txen_pin_, GPIO_PIN_RESET);
    latest_mseq_len_ = N;
  }

  template <size_t N> void mseq_receive(std::array<uint8_t, N> &res) {
    if (uart_.available() == latest_mseq_len_ + N) {
      for (size_t i = 0; i < latest_mseq_len_; ++i) {
        uart_.read();
      }
      for (size_t i = 0; i < N; ++i) {
        res[i] = uart_.read();
      }
    }
  }

  void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
  }

private:
  ltc2874::Port port_;
  iolink::COM com_;

  UART_HandleTypeDef *huart_;
  GPIO_TypeDef *txen_port_;
  uint32_t txen_pin_;
  UART uart_;

  size_t latest_mseq_len_;
};

} // namespace driver
