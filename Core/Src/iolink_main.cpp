#include <array>
#include <cstddef>
#include <cstdint>

#include "main.h"

#include "driver/iolink_port.hpp"
#include "driver/ltc2874.hpp"
#include "iolink/iolink_port.hpp"

extern TIM_HandleTypeDef htim1;
extern FDCAN_HandleTypeDef hfdcan1;

template <class Driver, size_t N, size_t... Seq>
auto make_iolink_ports(std::array<Driver, N> &drivers, std::index_sequence<Seq...>) {
  return std::array{iolink::IOLinkPort{drivers[Seq]}...};
}

template <class Driver, size_t N> auto make_iolink_ports(std::array<Driver, N> &drivers) {
  return make_iolink_ports(drivers, std::make_index_sequence<N>());
}

extern "C" void iolink_main() {
  // IOLink Config
  std::array iolink_port_drivers{
      driver::IOLinkPortDriver{ltc2874::Port::PORT1, iolink::COM::COM2},
      // driver::IOLinkPortDriver{ltc2874::Port::PORT2, iolink::COM::COM2},
      // driver::IOLinkPortDriver{ltc2874::Port::PORT3, iolink::COM::COM2},
      // driver::IOLinkPortDriver{ltc2874::Port::PORT4, iolink::COM::COM2},
  };
  std::array iolink_ports = make_iolink_ports(iolink_port_drivers);
  uint32_t master_cycle_time = 3000;

  // Initialize STM32 Peripherals
  HAL_TIM_Base_Start(&htim1);
  HAL_FDCAN_Start(&hfdcan1);

  // Initialize IOLink Port Drivers
  HAL_Delay(10);
  driver::ltc2874.reset();
  HAL_Delay(10);
  for (auto &driver : iolink_port_drivers) {
    driver.init();
  }
  HAL_Delay(10);
  driver::ltc2874.update();

  // Initialize IOLink Ports
  HAL_Delay(10);
  for (auto &port : iolink_ports) {
    while (!port.wakeup_request())
      ;
    while (!port.set_master_ident())
      ;
    while (!port.pre_operate_mode())
      ;
    while (!port.set_master_cycle_time(master_cycle_time))
      ;
    while (!port.operate_mode())
      ;
  }

  // M-Sequence for exchanging Process Data
  constexpr auto mseq_pd =
      iolink::create_mseq(iolink::MSeqRW::READ, iolink::MSeqChannel::PROCESS, 0x00, iolink::MSeqType::TYPE_2);
  std::array<uint8_t, 4> mseq_pd_response;

  // Process Data
  std::array<uint16_t, 4> process_data = {};

  // Classic CAN Header
  FDCAN_TxHeaderTypeDef tx_header;
  tx_header.Identifier = 0x1;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = FDCAN_DLC_BYTES_8;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;

  // Exchange Process Data
  while (true) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    for (auto &port : iolink_ports) {
      port.mseq_transmit(mseq_pd);
    }
    while (static_cast<int32_t>(master_cycle_time) - static_cast<int32_t>(__HAL_TIM_GET_COUNTER(&htim1)) > 0)
      ;
    for (size_t i = 0; i < iolink_ports.size(); ++i) {
      if (iolink_ports[i].mseq_receive(mseq_pd_response)) {
        process_data[i] = (mseq_pd_response[1] << 8) | mseq_pd_response[2];
      }
    }
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, reinterpret_cast<uint8_t *>(process_data.data()));
  }
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART1) {
    HAL_GPIO_WritePin(TXEN1_GPIO_Port, TXEN1_Pin, GPIO_PIN_RESET);
  } else if (huart->Instance == USART2) {
    HAL_GPIO_WritePin(TXEN2_GPIO_Port, TXEN2_Pin, GPIO_PIN_RESET);
  } else if (huart->Instance == USART3) {
    HAL_GPIO_WritePin(TXEN3_GPIO_Port, TXEN3_Pin, GPIO_PIN_RESET);
  } else if (huart->Instance == USART6) {
    HAL_GPIO_WritePin(TXEN4_GPIO_Port, TXEN4_Pin, GPIO_PIN_RESET);
  }
}
