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


#ifndef ROMEA_RTLS_COORDINATORS__RTLS_GEOREFERENCED_COORDINATOR_HPP_
#define ROMEA_RTLS_COORDINATORS__RTLS_GEOREFERENCED_COORDINATOR_HPP_

// std
#include <map>
#include <memory>
#include <string>
#include <vector>

// ros
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_common_utils/publishers/diagnostic_publisher.hpp"
#include "romea_rtls_coordinators/rtls_coordinator_scheduler_interface.hpp"
#include "romea_rtls_coordinators/rtls_coordinator_transceivers_interface.hpp"
#include "romea_rtls_coordinators/visibility_control.h"

namespace romea
{


template<typename SchedulerType>
class RTLSCoordinator
{
public:
  using SchedulerInterface = RTLSCoordinatorSchedulerInterface<SchedulerType>;
  using TransceiversInterface = RTLSCoordinatorTransceiversInterface;

public:
  ROMEA_RTLS_COORDINATORS_PUBLIC
  explicit RTLSCoordinator(const rclcpp::NodeOptions & options);

  ROMEA_RTLS_COORDINATORS_PUBLIC
  virtual ~RTLSCoordinator() = default;

  ROMEA_RTLS_COORDINATORS_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

private:
  void declare_parameters_();

  void init_scheduler_interface();

  void init_transceivers_interface();

  void init_diagnostics_publisher_();

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::unique_ptr<SchedulerInterface> scheduler_interface_;
  std::unique_ptr<TransceiversInterface> transceivers_interface_;
  std::shared_ptr<StampedPublisherBase<DiagnosticReport>> diagnostics_publisher_;
};

using RTLSSimpleCoordinator = RTLSCoordinator<RTLSSimpleCoordinatorScheduler>;
using RTLSGeoreferencedCoordinator = RTLSCoordinator<RTLSGeoreferencedCoordinatorScheduler>;

}  // namespace romea

#endif   // ROMEA_RTLS_COORDINATORS__RTLS_GEOREFERENCED_COORDINATOR_HPP_
