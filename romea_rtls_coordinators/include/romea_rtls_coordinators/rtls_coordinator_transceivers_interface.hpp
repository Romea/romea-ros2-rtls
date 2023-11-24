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


#ifndef ROMEA_RTLS_COORDINATORS__RTLS_COORDINATOR_TRANSCEIVERS_INTERFACE_HPP_
#define ROMEA_RTLS_COORDINATORS__RTLS_COORDINATOR_TRANSCEIVERS_INTERFACE_HPP_

// std
#include <map>
#include <memory>
#include <string>
#include <vector>

// ros
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_rtls_transceiver_utils/transceiver_interface_client.hpp"
#include "romea_rtls_transceiver_utils/rtls_transceiver_data_conversions.hpp"
#include "romea_rtls_transceiver_msgs/msg/transceiver_range_stamped.hpp"

// local
#include "romea_rtls_coordinators/visibility_control.h"

namespace romea
{
namespace ros2
{

class RTLSCoordinatorTransceiversInterface
{
public:
  using RangingResult = RTLSTransceiverRangingResult;
  using TransceiverRangeStamped = romea_rtls_transceiver_msgs::msg::TransceiverRangeStamped;
  using TransceiverRangePublisher = rclcpp::Publisher<TransceiverRangeStamped>;

public:
  explicit RTLSCoordinatorTransceiversInterface(const std::shared_ptr<rclcpp::Node> node);

  RTLSTransceiverRangingResult do_ranging(
    const size_t & initiatorIndex,
    const size_t & responderIndex,
    const Duration & timeout);

private:
  std::vector<std::string> initiators_names_;
  std::vector<uint16_t> initiators_ids_;
  std::vector<std::string> responders_names_;
  std::vector<uint16_t> responders_ids_;

  std::vector<std::unique_ptr<TransceiverInterfaceClient>> initiators_interfaces_;
  std::vector<std::shared_ptr<TransceiverRangePublisher>> initiators_range_pubs_;

  // TODO(JEAN) add payload support
};

}  // namespace ros2
}  // namespace romea

#endif   // ROMEA_RTLS_COORDINATORS__RTLS_COORDINATOR_TRANSCEIVERS_INTERFACE_HPP_
