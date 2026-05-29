#include "hooke2_message_manager/hooke2_message_manager.hpp"

namespace hooke2 {

Hooke2MessageManager::Hooke2MessageManager() {
  // Control Messages
  AddSendProtocolData<hooke2::common::Throttlecommand100, true>();
  // AddSendProtocolData<Brakecommand101, true>();
  // AddSendProtocolData<Gearcommand103, true>();
  // AddSendProtocolData<Parkcommand104, true>();
  // AddSendProtocolData<Steeringcommand102, true>();
  // AddSendProtocolData<Vehiclemodecommand105, true>();
}

Hooke2MessageManager::~Hooke2MessageManager() {}

}  // namespace hooke2
