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


#ifndef ROMEA_RTLS_COORDINATORS__RTLS_COORDINATOR_SCHEDULER_INTERFACE_HPP_
#define ROMEA_RTLS_COORDINATORS__RTLS_COORDINATOR_SCHEDULER_INTERFACE_HPP_

// std
#include <memory>

// ros
#include "rclcpp/rclcpp.hpp"

// romea
#include "nav_msgs/msg/odometry.hpp"
#include "romea_core_rtls/coordination/RTLSSimpleCoordinatorScheduler.hpp"
#include "romea_core_rtls/coordination/RTLSGeoreferencedCoordinatorScheduler.hpp"

// local
#include "romea_rtls_coordinators/visibility_control.h"

namespace romea
{

template<typename SchedulerType>
struct RTLSCoordinatorSchedulerInterface;

template<>
struct RTLSCoordinatorSchedulerInterface<RTLSSimpleCoordinatorScheduler>
{
  ROMEA_RTLS_COORDINATORS_PUBLIC
  RTLSCoordinatorSchedulerInterface(
    std::shared_ptr<rclcpp::Node> node,
    std::function<RTLSTransceiverRangingResult(
      const size_t &, const size_t &, const Duration &)> callback);

  RTLSSimpleCoordinatorScheduler scheduler;
};

template<>
struct RTLSCoordinatorSchedulerInterface<RTLSGeoreferencedCoordinatorScheduler>
{
  using Odometry = nav_msgs::msg::Odometry;

  ROMEA_RTLS_COORDINATORS_PUBLIC
  RTLSCoordinatorSchedulerInterface(
    std::shared_ptr<rclcpp::Node> node,
    std::function<RTLSTransceiverRangingResult(
      const size_t &, const size_t &, const Duration &)> callback);

  void odom_callback(Odometry::ConstSharedPtr msg);

  RTLSGeoreferencedCoordinatorScheduler scheduler;
  std::shared_ptr<rclcpp::Subscription<Odometry>> odom_sub;
};

}  // namespace romea

#endif   // ROMEA_RTLS_COORDINATORS__RTLS_COORDINATOR_SCHEDULER_INTERFACE_HPP_
