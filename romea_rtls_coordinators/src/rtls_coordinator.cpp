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


// romea
#include "romea_rtls_coordinators/rtls_coordinator.hpp"
#include "romea_rtls_coordinators/rtls_coordinator_parameters.hpp"

namespace romea
{

//-----------------------------------------------------------------------------
template<typename SchedulerType>
RTLSCoordinator<SchedulerType>::RTLSCoordinator(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("rtls_coordinator", options))
{
  declare_parameters_();
  init_scheduler_interface();
  init_transceivers_interface();
  init_diagnostics_publisher_();
  scheduler_interface_->scheduler.start();
}

//-----------------------------------------------------------------------------
template<typename SchedulerType>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
RTLSCoordinator<SchedulerType>::get_node_base_interface()const
{
  return node_->get_node_base_interface();
}

//-----------------------------------------------------------------------------
template<typename SchedulerType>
void RTLSCoordinator<SchedulerType>::declare_parameters_()
{
  declare_poll_rate(node_);
  declare_initiators_names(node_);
  declare_initiators_ids(node_);
  declare_responders_names(node_);
  declare_responders_ids(node_);


  if (std::is_same<SchedulerType, RTLSGeoreferencedCoordinatorScheduler>::value) {
    declare_maximal_range(node_);
    declare_initiators_positions(node_);
    declare_responders_positions(node_);
  }
}

//-----------------------------------------------------------------------------
template<typename SchedulerType>
void RTLSCoordinator<SchedulerType>::init_scheduler_interface()
{
  auto callback = std::bind(
    &RTLSCoordinatorTransceiversInterface::do_ranging, transceivers_interface_.get(),
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  scheduler_interface_ = std::make_unique<SchedulerInterface>(node_, callback);
}

//-----------------------------------------------------------------------------
template<typename SchedulerType>
void RTLSCoordinator<SchedulerType>::init_transceivers_interface()
{
  transceivers_interface_ = std::make_unique<TransceiversInterface>(node_);
}

//-----------------------------------------------------------------------------
template<typename SchedulerType>
void RTLSCoordinator<SchedulerType>::init_diagnostics_publisher_()
{
  diagnostics_publisher_ =
    make_diagnostic_publisher<DiagnosticReport>(node_, node_->get_name(), 1.0);
}

template class RTLSCoordinator<RTLSSimpleCoordinatorScheduler>;
template class RTLSCoordinator<RTLSGeoreferencedCoordinatorScheduler>;


}  // namespace romea

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(romea::RTLSSimpleCoordinator);
RCLCPP_COMPONENTS_REGISTER_NODE(romea::RTLSGeoreferencedCoordinator);
