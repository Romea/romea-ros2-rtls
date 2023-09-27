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


#ifndef ROMEA_RTLS_UTILS__RTLS_COMMUNICATION_HUB_HPP_
#define ROMEA_RTLS_UTILS__RTLS_COMMUNICATION_HUB_HPP_

// std
#include <memory>
#include <string>
#include <vector>
#include <functional>

// romea
#include "romea_rtls_transceiver_utils/rtls_transceiver_interface_client.hpp"
#include "romea_core_rtls_transceiver/RTLSTransceiverRangingResult.hpp"

// local
#include "romea_rtls_utils/visibility_control.h"

namespace romea
{

class RTLSCommunicationHub
{
public:
  // using RangingResult = RTLSTransceiverRangingResult;
  using RangeMsg = romea_rtls_transceiver_msgs::msg::Range;
  using PayloadMsg = romea_rtls_transceiver_msgs::msg::Payload;
  using RangingResultMsg = romea_rtls_transceiver_msgs::msg::RangingResult;
  using TransceiverInterface = std::shared_ptr<RTLSTransceiverInterfaceClient>;

  using RangeCallback = std::function<void (
        const size_t & /*initiatorIndex*/,
        const size_t & /*responderIndex*/,
        const RangeMsg & /*result*/)>;

  using PayloadCallback = std::function<void (
        const PayloadMsg & /*payload*/)>;

public:
  RTLSCommunicationHub(
    std::shared_ptr<rclcpp::Node> node,
    RangeCallback range_callback = {},
    PayloadCallback payload_callback = {});

  RTLSCommunicationHub(
    std::shared_ptr<rclcpp::Node> node,
    const std::vector<std::string> & initiators_names,
    const std::vector<uint16_t> & initiators_ids,
    const std::vector<std::string> & responders_names,
    const std::vector<uint16_t> & responders_ids,
    RangeCallback range_callback = {},
    PayloadCallback payload_callback = {});

  void send_ranging_request(
    const size_t & initiatorIndex,
    const size_t & responderIndex,
    const PayloadMsg & payload,
    const double & timeout);

  void send_ranging_request(
    const size_t & initiatorIndex,
    const size_t & responderIndex,
    const double & timeout);

private:
  void ranging_result_msg_callback_(
    RangingResultMsg::ConstSharedPtr msg,
    size_t initiator_index);

  void payload_msg_callback_(
    PayloadMsg::ConstSharedPtr msg);

  size_t responder_index_(const uint16_t & responder_id);

private:
  std::vector<std::string> initiators_names_;
  std::vector<uint16_t> initiators_ids_;
  std::vector<std::string> responders_names_;
  std::vector<uint16_t> responders_ids_;

  std::vector<TransceiverInterface> initiators_interfaces_;
  RangeCallback range_callback_;
  PayloadCallback payload_callback_;
};

}  // namespace romea

#endif   // ROMEA_RTLS_UTILS__RTLS_COMMUNICATION_HUB_HPP_
