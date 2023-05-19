#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "iolink/iolink_util.hpp"

namespace iolink {

template <class Driver> class IOLinkPort {
public:
  IOLinkPort(Driver &driver) : driver_(driver) {}

  bool wakeup_request() {
    constexpr auto mseq = create_mseq(MSeqRW::READ, MSeqChannel::PAGE, 0xA2, MSeqType::TYPE_0);
    std::array<uint8_t, 2> res;
    driver_.wakeup_request();
    mseq_transmit(mseq);
    driver_.delay_ms(30);
    return mseq_receive(res);
  }

  bool set_master_ident() {
    constexpr auto mseq = create_mseq(MSeqRW::WRITE, MSeqChannel::PAGE, 0x00, MSeqType::TYPE_0, 0x95);
    std::array<uint8_t, 1> res;
    mseq_transmit(mseq);
    driver_.delay_ms(30);
    return mseq_receive(res);
  }

  bool pre_operate_mode() {
    constexpr auto mseq = create_mseq(MSeqRW::WRITE, MSeqChannel::PAGE, 0x00, MSeqType::TYPE_0, 0x9A);
    std::array<uint8_t, 1> res;
    mseq_transmit(mseq);
    driver_.delay_ms(30);
    return mseq_receive(res);
  }

  bool set_master_cycle_time(uint32_t us) {
    auto mseq = create_mseq(MSeqRW::WRITE, MSeqChannel::PAGE, 0x01, MSeqType::TYPE_0, iolink::us_to_cycle_time(us));
    std::array<uint8_t, 1> res;
    mseq_transmit(mseq);
    driver_.delay_ms(30);
    return mseq_receive(res);
  }

  bool operate_mode() {
    constexpr auto mseq = create_mseq(MSeqRW::WRITE, MSeqChannel::PAGE, 0x00, MSeqType::TYPE_0, 0x99);
    std::array<uint8_t, 1> res;
    mseq_transmit(mseq);
    driver_.delay_ms(30);
    return mseq_receive(res);
  }

  template <size_t N> void mseq_transmit(const std::array<uint8_t, N> &mseq) {
    driver_.mseq_transmit(mseq);
  }

  template <size_t N> bool mseq_receive(std::array<uint8_t, N> &res) {
    driver_.mseq_receive(res);
    uint8_t checksum = res[N - 1];
    res[N - 1] = 0;
    return mseq_checksum(res) == checksum;
  }

private:
  Driver &driver_;
};

template <class Driver, size_t N, size_t... Seq>
auto make_iolink_ports(std::array<Driver, N> &drivers, std::index_sequence<Seq...>) {
  return std::array{iolink::IOLinkPort{drivers[Seq]}...};
}

template <class Driver, size_t N> auto make_iolink_ports(std::array<Driver, N> &drivers) {
  return make_iolink_ports(drivers, std::make_index_sequence<N>());
}

} // namespace iolink
