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


// local
#include "romea_common_utils/qos.hpp"
#include "romea_rtls_coordinators/rtls_coordinator_parameters.hpp"
#include "romea_rtls_coordinators/rtls_coordinator_scheduler_interface.hpp"
#include "romea_rtls_coordinators/visibility_control.h"

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
RTLSCoordinatorSchedulerInterface<RTLSSimpleCoordinatorScheduler>::RTLSCoordinatorSchedulerInterface(
  std::shared_ptr<rclcpp::Node> node,
  std::function<RTLSTransceiverRangingResult(
    const size_t &, const size_t &, const Duration &)> callback)
: scheduler(
    get_poll_rate(node),
    get_initiators_names(node),
    get_responders_names(node),
    callback)
{
}

//-----------------------------------------------------------------------------
RTLSCoordinatorSchedulerInterface<RTLSGeoreferencedCoordinatorScheduler>::
RTLSCoordinatorSchedulerInterface(
  std::shared_ptr<rclcpp::Node> node,
  std::function<RTLSTransceiverRangingResult(
    const size_t &, const size_t &, const Duration &)>
  callback)
: scheduler(
    get_poll_rate(node),
    get_maximal_range(node),
    get_initiators_names(node),
    get_initiators_positions(node),
    get_responders_names(node),
    get_responders_positions(node),
    callback)
{
  using ShedulerInterface =
    RTLSCoordinatorSchedulerInterface<RTLSGeoreferencedCoordinatorScheduler>;

  auto odom_callback = std::bind(
    &ShedulerInterface::odom_callback, this, std::placeholders::_1);

  odom_sub = node->create_subscription<Odometry>(
    "filtered_odom", best_effort(1), odom_callback);
}

//-----------------------------------------------------------------------------
void RTLSCoordinatorSchedulerInterface<RTLSGeoreferencedCoordinatorScheduler>::odom_callback(
  Odometry::ConstSharedPtr msg)
{
  Eigen::Vector3d position;
  position.x() = msg->pose.pose.position.x;
  position.y() = msg->pose.pose.position.y;
  position.z() = msg->pose.pose.position.z;
  scheduler.updateRobotPosition(position);
}

}  // namespace ros2
}  // namespace romea
