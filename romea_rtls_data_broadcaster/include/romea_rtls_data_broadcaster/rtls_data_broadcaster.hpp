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


#ifndef ROMEA_RTLS_DATA_BROADCASTER__RTLS_DATA_BROADCASTER_HPP_
#define ROMEA_RTLS_DATA_BROADCASTER__RTLS_DATA_BROADCASTER_HPP_

// std
#include <memory>
#include <string>
#include <vector>

// ros
#include "nav_msgs/msg/odometry.hpp"

// romea
#include "romea_core_rtls/serialization/Pose2DSerialization.hpp"
#include "romea_core_rtls/serialization/Twist2DSerialization.hpp"
#include "romea_rtls_transceiver_utils/rtls_transceiver_interface_client.hpp"
#include "romea_rtls_data_broadcaster/visibility_control.h"


namespace romea
{
namespace ros2
{

template<typename DataType>
class RTLSDataBroadcaster
{
public:
  using Payload = romea_rtls_transceiver_msgs::msg::Payload;
  using Odometry = nav_msgs::msg::Odometry;

public:
  ROMEA_RTLS_DATA_BROADCASTER_PUBLIC
  explicit RTLSDataBroadcaster(const rclcpp::NodeOptions & options);

  ROMEA_RTLS_DATA_BROADCASTER_PUBLIC
  virtual ~RTLSDataBroadcaster() = default;

  ROMEA_RTLS_DATA_BROADCASTER_PUBLIC
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr
  get_node_base_interface() const;

private:
  void init_odom_sub_();

  void init_transceivers_interfaces_();

  void odom_callback_(Odometry::ConstSharedPtr msg);

  void send_payload_(const std::vector<uint8_t> & payload);

private:
  std::shared_ptr<rclcpp::Node> node_;
  std::vector<std::unique_ptr<RTLSTransceiverInterfaceClient>> transceivers_;
  std::shared_ptr<rclcpp::Subscription<Odometry>> odom_sub_;
};

using RTLSPose2DBroadcaster = RTLSDataBroadcaster<core::Pose2D>;
using RTLSTwist2DBroadcaster = RTLSDataBroadcaster<core::Twist2D>;

}  // namespace ros2
}  // namespace romea

#endif   // ROMEA_RTLS_DATA_BROADCASTER__RTLS_DATA_BROADCASTER_HPP_
