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

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_rtls_coordinators/rtls_coordinator_parameters.hpp"
#include "romea_rtls_coordinators/rtls_coordinator_transceivers_interface.hpp"


namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
RTLSCoordinatorTransceiversInterface::RTLSCoordinatorTransceiversInterface(
  const std::shared_ptr<rclcpp::Node> node)
: initiators_names_(get_initiators_names(node)),
  initiators_ids_(get_initiators_ids(node)),
  responders_names_(get_responders_names(node)),
  responders_ids_(get_responders_ids(node)),
  initiators_interfaces_(initiators_names_.size()),
  initiators_range_pubs_(initiators_names_.size())
{
  for (size_t i = 0; i < initiators_names_.size(); ++i) {
    initiators_interfaces_[i] = std::make_unique<TransceiverInterfaceClient>(
      node, initiators_names_[i]);

    initiators_range_pubs_[i] = node->create_publisher<TransceiverRangeStamped>(
      initiators_names_[i] + "/range", sensor_data_qos());
  }
}

//-----------------------------------------------------------------------------
RTLSTransceiverRangingResult RTLSCoordinatorTransceiversInterface::do_ranging(
  const size_t & initiator_index,
  const size_t & responder_index,
  const Duration & timeout)
{
  RTLSTransceiverRangingResult ranging_result;
  auto range_msg = std::make_unique<TransceiverRangeStamped>();
  // range_msg->header->frame_id = initiatords_names[initiator_index]
  range_msg->range.responder_name = responders_names_[responder_index];
  bool success = initiators_interfaces_[initiator_index]->ranging(
    responders_ids_[responder_index],
    range_msg->range.result,
    timeout);

  if (success) {
    initiators_interfaces_[initiator_index]->get_payload(range_msg->range.payload);
  }

  to_romea(range_msg->range.result, ranging_result);
  initiators_range_pubs_[initiator_index]->publish(std::move(range_msg));
  return ranging_result;
}

}  // namespace ros2
}  // namespace romea
