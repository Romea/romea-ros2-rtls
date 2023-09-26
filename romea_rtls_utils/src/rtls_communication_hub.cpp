// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


// std
#include <map>
#include <memory>
#include <string>
#include <vector>

// local
#include "romea_rtls_utils/rtls_parameters.hpp"
#include "romea_rtls_utils/rtls_communication_hub.hpp"
#include "romea_rtls_transceiver_utils/rtls_transceiver_data_conversions.hpp"


namespace romea
{

//-----------------------------------------------------------------------------
RTLSCommunicationHub::RTLSCommunicationHub(
  std::shared_ptr<rclcpp::Node> node,
  RangeCallback range_callback,
  PayloadCallback payload_callback)
: initiators_names_(get_initiators_names(node)),
  initiators_ids_(get_initiators_ids(node)),
  responders_names_(get_responders_names(node)),
  responders_ids_(get_responders_ids(node)),
  initiators_interfaces_(initiators_names_.size()),
  range_callback_(range_callback),
  payload_callback_(payload_callback)
{
  for (size_t n = 0; n < initiators_names_.size(); ++n) {
    auto ranging_result_msg_callback = std::bind(
      &RTLSCommunicationHub::ranging_result_msg_callback_,
      this, std::placeholders::_1, n);

    RTLSTransceiverInterfaceClient::PayloadCallback payload_msg_callback;
    if (payload_callback_) {
      payload_msg_callback = std::bind(
        &RTLSCommunicationHub::payload_msg_callback_, this, std::placeholders::_1);
    }

    initiators_interfaces_[n] = std::make_unique<RTLSTransceiverInterfaceClient>(
      node, initiators_names_[n], ranging_result_msg_callback, payload_msg_callback);
  }
}

//-----------------------------------------------------------------------------
void RTLSCommunicationHub::send_ranging_request(
  const size_t & initiatorIndex,
  const size_t & responderIndex,
  const PayloadMsg & payload,
  const double & timeout)
{
  romea_rtls_transceiver_msgs::msg::RangingRequest request_msg;
  request_msg.responder_id = responders_ids_[responderIndex];
  request_msg.payload = payload;
  request_msg.timeout = timeout;

  initiators_interfaces_[initiatorIndex]->send_ranging_request(request_msg);
}

//-----------------------------------------------------------------------------
void RTLSCommunicationHub::send_ranging_request(
  const size_t & initiatorIndex,
  const size_t & responderIndex,
  const double & timeout)
{
  romea_rtls_transceiver_msgs::msg::RangingRequest request_msg;
  request_msg.responder_id = responders_ids_[responderIndex];
  request_msg.timeout = timeout;

  initiators_interfaces_[initiatorIndex]->send_ranging_request(request_msg);
}


//-----------------------------------------------------------------------------
void RTLSCommunicationHub::ranging_result_msg_callback_(
  RangingResultMsg::ConstSharedPtr msg,
  size_t initiator_index)
{
  assert(initiators_ids_[initiator_index] == msg->initiator_id);
  if (range_callback_) {
    range_callback_(
      initiator_index,
      responder_index_(msg->responder_id),
      msg->range);
  }
}

//-----------------------------------------------------------------------------
void RTLSCommunicationHub::payload_msg_callback_(
  PayloadMsg::ConstSharedPtr msg)
{
  payload_callback_(*msg);
}

//-----------------------------------------------------------------------------
size_t RTLSCommunicationHub::responder_index_(const uint16_t & responder_id)
{
  auto it = std::find(responders_ids_.begin(), responders_ids_.end(), responder_id);
  return std::distance(responders_ids_.begin(), it);
}


}  // namespace romea
