#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "main.h"

#include "driver/ltc2874.hpp"
#include "driver/uart.hpp"
#include "iolink/iolink_util.hpp"

namespace driver {

class IOLinkPortDriver {
public:
  IOLinkPortDriver(ltc2874::Port port, iolink::COM com) : port_(port), com_(com), uart_(port, com) {}

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
    uart_.deinit();
    switch (port_) {
      case ltc2874::Port::PORT1:
        HAL_GPIO_WritePin(TXEN1_GPIO_Port, TXEN1_Pin, GPIO_PIN_RESET);
        driver::ltc2874.write_bit_lower(ltc2874::Register::EVENT3, ltc2874::EVENT3_Bit::TOC_CQ1, true);
        driver::ltc2874.write_bit_raise(ltc2874::Register::CTRL1, ltc2874::CTRL1_Bit::WKUP1, true);
        break;
      case ltc2874::Port::PORT2:
        // HAL_GPIO_WritePin(TXEN2_GPIO_Port, TXEN2_Pin, GPIO_PIN_RESET);
        driver::ltc2874.write_bit_lower(ltc2874::Register::EVENT3, ltc2874::EVENT3_Bit::TOC_CQ2, true);
        driver::ltc2874.write_bit_raise(ltc2874::Register::CTRL1, ltc2874::CTRL1_Bit::WKUP2, true);
        break;
      case ltc2874::Port::PORT3:
        HAL_GPIO_WritePin(TXEN3_GPIO_Port, TXEN3_Pin, GPIO_PIN_RESET);
        driver::ltc2874.write_bit_lower(ltc2874::Register::EVENT3, ltc2874::EVENT3_Bit::TOC_CQ3, true);
        driver::ltc2874.write_bit_raise(ltc2874::Register::CTRL1, ltc2874::CTRL1_Bit::WKUP3, true);
        break;
      case ltc2874::Port::PORT4:
        HAL_GPIO_WritePin(TXEN4_GPIO_Port, TXEN4_Pin, GPIO_PIN_RESET);
        driver::ltc2874.write_bit_lower(ltc2874::Register::EVENT3, ltc2874::EVENT3_Bit::TOC_CQ4, true);
        driver::ltc2874.write_bit_raise(ltc2874::Register::CTRL1, ltc2874::CTRL1_Bit::WKUP4, true);
        break;
    }
    delay_ms(1);
    uart_.init();
  }

  template <size_t N> void mseq_transmit(const std::array<uint8_t, N> &mseq) {
    uart_.flush();
    switch (port_) {
      case ltc2874::Port::PORT1:
        HAL_GPIO_WritePin(TXEN1_GPIO_Port, TXEN1_Pin, GPIO_PIN_SET);
        break;
      case ltc2874::Port::PORT2:
        // HAL_GPIO_WritePin(TXEN2_GPIO_Port, TXEN2_Pin, GPIO_PIN_SET);
        break;
      case ltc2874::Port::PORT3:
        HAL_GPIO_WritePin(TXEN3_GPIO_Port, TXEN3_Pin, GPIO_PIN_SET);
        break;
      case ltc2874::Port::PORT4:
        HAL_GPIO_WritePin(TXEN4_GPIO_Port, TXEN4_Pin, GPIO_PIN_SET);
        break;
    }
    uart_.transmit(mseq.data(), N);
    latest_mseq_len_ = N;
  }

  template <size_t N> void mseq_receive(std::array<uint8_t, N> &res) {
    if (uart_.available() == latest_mseq_len_ + N) {
      uart_.advance(latest_mseq_len_);
      uart_.receive(res.data(), N);
    } else {
      res.fill(0);
    }
  }

  void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
  }

private:
  ltc2874::Port port_;
  iolink::COM com_;
  UART uart_;
  size_t latest_mseq_len_;
};

} // namespace driver
