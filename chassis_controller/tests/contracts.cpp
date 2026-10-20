/* SPDX-License-Identifier: Apache-2.0 */
#include "chassis_config.h"
#include "chassis_controller.h"
#include "motor_commands.hpp"
#include "sensor_decoder.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <protocols/imu/hi91_protocol.h>
#include <vector>
#define CHECK(condition)                                                       \
  do {                                                                         \
    if (!(condition)) {                                                        \
      std::cerr << "line " << __LINE__ << '\n';                                \
      return 1;                                                                \
    }                                                                          \
  } while (false)
using namespace wbr;
namespace {
bridge::Packet Uart(unsigned port, uint32_t sequence, const uint8_t *data,
                    size_t size, uint32_t time = 1000) {
  bridge::Packet packet{};
  bridge::Header(packet.data(), bridge::Kind::Uart, time, 7);
  packet[10] = port;
  packet[11] = size;
  bridge::Put32(packet.data() + 16, sequence);
  bridge::Put32(packet.data() + 20, time);
  std::copy_n(data, size, packet.data() + 24);
  return packet;
}
std::array<uint8_t, 25> Sbus(bool enabled) {
  std::array<uint8_t, 25> bytes{};
  bytes[0] = 15;
  for (unsigned channel = 0; channel < 16; ++channel) {
    uint16_t value = 1024;
    if (channel == 4)
      value = enabled ? 1024 : 300;
    if (channel == 5)
      value = 300;
    for (unsigned bit = 0; bit < 11; ++bit)
      if (value & (1U << bit))
        bytes[1 + (channel * 11 + bit) / 8] |=
            uint8_t(1U << ((channel * 11 + bit) % 8));
  }
  return bytes;
}
} // namespace
int main() {
  bridge::CommandLease lease;
  CHECK(!lease.Expired(100000, false));
  CHECK(lease.CanAccept(0xffffffffU));
  lease.Renew(0xffffffffU, 0xfffffff0U);
  CHECK(!lease.CanAccept(0xffffffffU));
  CHECK(lease.CanAccept(0));
  CHECK(!lease.Expired(0x10U, true));
  CHECK(lease.Expired(0x10U, false));
  CHECK(lease.Expired(uint32_t(0xfffffff0U + bridge::kCommandTimeoutUs), true));
  lease.Renew(0, 0); // zero is not used as a sentinel
  CHECK(!lease.Expired(bridge::kCommandTimeoutUs - 1, true));
  CHECK(lease.Expired(bridge::kCommandTimeoutUs, true));
  lease.Reset();
  CHECK(!lease.Expired(0, false));

  CHECK(bridge::NewSequence(0, 0xffffffffU));
  CHECK(!bridge::NewSequence(9, 10));
  host::MotorCommands commands;
  commands.Begin();
  uint8_t zero[8]{};
  CHECK(commands.Submit(platform::CanTxSlot::kLeftWheel, 3, 0x200, zero, 8) ==
        0);
  auto packet = commands.packet();
  bridge::Header(packet.data(), bridge::Kind::Commands, 1, 7);
  CHECK(bridge::ValidCommands(packet, 7));
  CHECK(!bridge::ValidCommands(packet, 8));
  packet[10] = 2;
  std::copy_n(packet.data() + 32, 16, packet.data() + 48);
  CHECK(!bridge::ValidCommands(packet, 7));
  packet[10] = 7;
  CHECK(!bridge::ValidCommands(packet, 7));
  commands.Begin();
  modules::ChassisController controller(commands);
  modules::ControllerInputs sources{};
  sources.remote.robot_enable = true;
  sources.remote.sequence = 1;
  controller.Tick(sources, 1000, false);
  CHECK(controller.state() == modules::ChassisControlState::kDisabled);
  auto disabled = commands.packet();
  CHECK(disabled[10] == 6);
  for (unsigned i = 0; i < 6; ++i) {
    auto *record = disabled.data() + 32 + 16 * i;
    if (record[12] < 2)
      for (unsigned j = 0; j < 8; ++j)
        CHECK(record[4 + j] == 0);
    else
      CHECK(record[11] == 0xfd); // exit command, never enter or nonzero torque
  }
  modules::ChassisStateMachine states;
  modules::ChassisCycleInput input{};
  input.dt = .001;
  input.requested_enable = true;
  input.remote_fresh = true;
  input.imu_fresh = true;
  CHECK(states.Update(input).to == modules::ChassisControlState::kDmArming);
  input.imu_fresh = false;
  CHECK(states.Update(input).to == modules::ChassisControlState::kSafetyStop);
  input.requested_enable = false;
  CHECK(states.Update(input).to == modules::ChassisControlState::kDisabled);
  host::SensorStore store;
  host::SensorDecoder decoder(store, true);
  auto sbus = Sbus(true);
  CHECK(decoder.Consume(Uart(1, 0, sbus.data(), 10), 2000));
  CHECK(!store.Read().control.remote.robot_enable);
  CHECK(decoder.Consume(Uart(1, 2, sbus.data() + 10, 15),
                        2000)); // gap rejects partial frame
  CHECK(!store.Read().control.remote.robot_enable);
  CHECK(decoder.Consume(Uart(1, 3, sbus.data(), 25), 2000));
  CHECK(store.Read().control.remote.robot_enable);
  auto safe = Sbus(false);
  CHECK(decoder.Consume(Uart(1, 4, safe.data(), 25), 2000));
  CHECK(store.Read().control.remote_disable_pose);
  safe[23] = 8;
  CHECK(decoder.Consume(Uart(1, 5, safe.data(), 25), 2000));
  CHECK(!store.Read().control.remote_disable_pose &&
        !store.Read().control.remote.robot_enable);
  // A reconnect clears both published state and an unfinished SBUS frame.
  decoder.Reset();
  CHECK(decoder.Consume(Uart(1, 0, sbus.data(), 10), 2000));
  decoder.Reset();
  CHECK(!store.Read().control.remote.robot_enable);
  CHECK(decoder.Consume(Uart(1, 0, sbus.data() + 10, 15), 2000));
  CHECK(!store.Read().control.remote.robot_enable);
  CHECK(decoder.Consume(Uart(1, 1, sbus.data(), 25), 2000));
  CHECK(store.Read().control.remote.robot_enable);
  std::array<uint8_t, 82> hi91{};
  hi91[0] = 0x5a;
  hi91[1] = 0xa5;
  bridge::Put16(hi91.data() + 2, 76);
  auto *payload = hi91.data() + 6;
  payload[0] = 0x91;
  bridge::Put32(payload + 8, 1);
  float pitch = 3.0F;
  std::memcpy(payload + 52, &pitch, 4);
  bridge::Put16(hi91.data() + 4, protocols::CalculateHi91FrameCrc(76, payload));
  CHECK(decoder.Consume(Uart(2, 0, hi91.data(), 40), 2000));
  CHECK(decoder.Consume(Uart(2, 1, hi91.data() + 40, 40), 2000));
  CHECK(decoder.Consume(Uart(2, 2, hi91.data() + 80, 2), 2000));
  CHECK(store.Read().control.imu.valid &&
        store.Read().control.imu.pitch_deg == 3.0F);
  auto sequence = store.Read().control.imu.sequence;
  CHECK(decoder.Consume(Uart(2, 3, hi91.data(), 40), 3000));
  CHECK(decoder.Consume(Uart(2, 4, hi91.data() + 40, 40), 3000));
  CHECK(decoder.Consume(Uart(2, 5, hi91.data() + 80, 2), 3000));
  CHECK(store.Read().control.imu.sequence ==
        sequence); // duplicate sensor time cannot refresh state
  bridge::Packet can{};
  bridge::Header(can.data(), bridge::Kind::Can, 10, 7);
  can[10] = 1;
  bridge::CanRecord(can.data() + 16, 3, 0x201, zero, 8, 0xfffffff0U);
  CHECK(decoder.Consume(can, 1000));
  CHECK(store.Read().control.motors[0].precise_timestamp_us == 974);
  bridge::Put32(can.data() + 4, 50000);
  CHECK(decoder.Consume(can, 60000));
  CHECK(!store.Read().control.motors[0].valid);
  // Real balance controller runs with fixed frozen geometry; all outputs must
  // remain finite across repeated stateful updates and a changed speed target.
  modules::BalanceController balance;
  input = {};
  input.dt = .001;
  input.imu_fresh = input.feedback_valid = true;
  input.leg.left.length = input.leg.right.length = .20;
  input.leg.left.angle = input.leg.right.angle = 1.5707963267948966;
  input.leg.left.jacobian[0][0] = input.leg.right.jacobian[0][0] = .1;
  input.leg.left.jacobian[1][1] = input.leg.right.jacobian[1][1] = .1;
  modules::ChassisControlOutput output{};
  for (unsigned tick = 0; tick < 1000; ++tick) {
    input.remote.chassis_x = tick > 500 ? .2 : 0;
    balance.Update(input, output);
    CHECK(std::isfinite(output.physical_wheel_torque.left));
    CHECK(std::isfinite(output.joint_torque.left.b));
  }
  std::cout << "Session framing, disabled output, stale input, UART gaps, HI91 "
               "CRC, clock wrap and real control math passed\n";
}
