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
#include <memory>
#include <string>
#include <vector>


// romea
#include "romea_common_utils/qos.hpp"
#include "romea_common_utils/params/node_parameters.hpp"
#include "romea_common_utils/conversions/pose3d_conversions.hpp"
#include "romea_common_utils/conversions/twist3d_conversions.hpp"
#include "romea_rtls_data_broadcaster/rtls_data_broadcaster.hpp"

namespace romea
{


//-----------------------------------------------------------------------------
template<typename DataType>
RTLSDataBroadcaster<DataType>::RTLSDataBroadcaster(const rclcpp::NodeOptions & options)
: node_(std::make_shared<rclcpp::Node>("rtls_data_broadscater", options)),
  transceivers_(),
  odom_sub_()
{
  init_transceivers_interfaces_();
  init_odom_sub_();
}

//-----------------------------------------------------------------------------
template<typename DataType>
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
RTLSDataBroadcaster<DataType>::get_node_base_interface() const
{
  return node_->get_node_base_interface();
}

//-----------------------------------------------------------------------------
template<typename DataType>
void RTLSDataBroadcaster<DataType>::init_odom_sub_()
{
  auto callback = std::bind(
    &RTLSDataBroadcaster::odom_callback_, this, std::placeholders::_1);

  odom_sub_ = node_->create_subscription<Odometry>(
    "odom", best_effort(1), callback);
}

//-----------------------------------------------------------------------------
template<typename DataType>
void RTLSDataBroadcaster<DataType>::init_transceivers_interfaces_()
{
  declare_vector_parameter<std::string>(node_, "transceivers_names");
  auto transceivers_names = get_vector_parameter<std::string>(node_, "transceivers_names");

  transceivers_.resize(transceivers_names.size());
  for (size_t i = 0; i < transceivers_names.size(); ++i) {
    transceivers_[i] = std::make_unique<RTLSTransceiverInterfaceClient>(
      node_, transceivers_names[i]);
  }
}

//-----------------------------------------------------------------------------
template<typename DataType>
void RTLSDataBroadcaster<DataType>::send_payload_(const std::vector<uint8_t> & payload)
{
  Payload payload_msg;
  payload_msg.data = payload;
  for (size_t i = 0; i < transceivers_.size(); ++i) {
    transceivers_[i]->send_payload(payload_msg);
  }
}

//-----------------------------------------------------------------------------
template<>
void RTLSDataBroadcaster<Twist2D>::odom_callback_(Odometry::ConstSharedPtr msg)
{
  send_payload_(serializeTwist2D(toTwist2D(to_romea(msg->twist))));
}
//-----------------------------------------------------------------------------
template<>
void RTLSDataBroadcaster<Pose2D>::odom_callback_(Odometry::ConstSharedPtr msg)
{
  send_payload_(serializePose2D(toPose2D(to_romea(msg->pose))));
}

template class RTLSDataBroadcaster<Pose2D>;
template class RTLSDataBroadcaster<Twist2D>;

}  // namespace romea
