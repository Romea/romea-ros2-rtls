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
#include <string>
#include <vector>

// romea
#include "romea_rtls_utils/rtls_parameters.hpp"
#include "romea_common_utils/params/eigen_parameters.hpp"

const char poll_rate_param_name[] = "poll_rate";
const char range_std_param_name[] = "range_std";
const char minimal_range_param_name[] = "minimal_range";
const char maximal_range_param_name[] = "maximal_range";
const char enable_sheduler_param_name[] = "enable_sheduler";
const char initiator_id_param_name[] = "initiators_id";
const char initiator_name_param_name[] = "initiators_name";
const char initiator_position_param_name[] = "initiators_position";
const char initiators_ids_param_name[] = "initiators_ids";
const char initiators_names_param_name[] = "initiators_names";
const char initiators_positions_param_name[] = "initiators_positions";
const char responder_id_param_name[] = "responder_id";
const char responder_name_param_name[] = "responder_name";
const char responder_position_param_name[] = "responder_position";
const char responders_ids_param_name[] = "responders_ids";
const char responders_names_param_name[] = "responders_names";
const char responders_positions_param_name[] = "responders_positions";

namespace romea
{
namespace ros2
{

//-----------------------------------------------------------------------------
void declare_poll_rate(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, poll_rate_param_name);
}

//-----------------------------------------------------------------------------
void declare_range_std(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, range_std_param_name);
}

//-----------------------------------------------------------------------------
void declare_minimal_range(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, minimal_range_param_name);
}

//-----------------------------------------------------------------------------
void declare_maximal_range(rclcpp::Node::SharedPtr node)
{
  declare_parameter<double>(node, maximal_range_param_name);
}

//-----------------------------------------------------------------------------
void declare_initiator_name(rclcpp::Node::SharedPtr node)
{
  declare_parameter<std::string>(node, initiator_name_param_name);
}

//-----------------------------------------------------------------------------
void declare_initiator_id(rclcpp::Node::SharedPtr node)
{
  declare_vector_parameter<int64_t>(node, initiator_id_param_name);
}

//-----------------------------------------------------------------------------
void declare_initiator_position(rclcpp::Node::SharedPtr node)
{
  declare_eigen_vector_parameter<Eigen::Vector3d>(
    node, initiators_positions_param_name);
}

//-----------------------------------------------------------------------------
void declare_responder_name(rclcpp::Node::SharedPtr node)
{
  declare_parameter<std::string>(node, responder_name_param_name);
}

//-----------------------------------------------------------------------------
void declare_responder_id(rclcpp::Node::SharedPtr node)
{
  declare_parameter<int64_t>(node, responder_id_param_name);
}

//-----------------------------------------------------------------------------
void declare_responder_position(rclcpp::Node::SharedPtr node)
{
  declare_eigen_vector_parameter<Eigen::Vector3d>(
    node, responder_position_param_name);
}


//-----------------------------------------------------------------------------
void declare_initiators_names(rclcpp::Node::SharedPtr node)
{
  declare_vector_parameter<std::string>(node, initiators_names_param_name);
}

//-----------------------------------------------------------------------------
void declare_initiators_ids(rclcpp::Node::SharedPtr node)
{
  declare_vector_parameter<int64_t>(node, initiators_ids_param_name);
}

//-----------------------------------------------------------------------------
void declare_initiators_positions(rclcpp::Node::SharedPtr node)
{
  declare_eigen_vector_parameters<Eigen::Vector3d>(
    node, initiators_positions_param_name, get_initiators_names(node));
}

//-----------------------------------------------------------------------------
void declare_responders_names(rclcpp::Node::SharedPtr node)
{
  declare_vector_parameter<std::string>(node, responders_names_param_name);
}

//-----------------------------------------------------------------------------
void declare_responders_ids(rclcpp::Node::SharedPtr node)
{
  declare_vector_parameter<int64_t>(node, responders_ids_param_name);
}

//-----------------------------------------------------------------------------
void declare_responders_positions(rclcpp::Node::SharedPtr node)
{
  declare_eigen_vector_parameters<Eigen::Vector3d>(
    node, responders_positions_param_name, get_responders_names(node));
}

//-----------------------------------------------------------------------------
void declare_enable_scheduler(rclcpp::Node::SharedPtr node)
{
  declare_parameter_with_default<bool>(node, enable_sheduler_param_name, true);
}

//-----------------------------------------------------------------------------
double get_poll_rate(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, poll_rate_param_name);
}

//-----------------------------------------------------------------------------
double get_range_std(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, range_std_param_name);
}

//-----------------------------------------------------------------------------
double get_minimal_range(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, minimal_range_param_name);
}

//-----------------------------------------------------------------------------
double get_maximal_range(rclcpp::Node::SharedPtr node)
{
  return get_parameter<double>(node, maximal_range_param_name);
}


//-----------------------------------------------------------------------------
std::string get_initiator_name(rclcpp::Node::SharedPtr node)
{
  return get_parameter<std::string>(node, initiator_name_param_name);
}

//-----------------------------------------------------------------------------
uint16_t get_initiator_id(rclcpp::Node::SharedPtr node)
{
  return get_parameter<int64_t>(node, initiator_id_param_name);
}

//-----------------------------------------------------------------------------
Eigen::Vector3d get_initiator_position(rclcpp::Node::SharedPtr node)
{
  return get_eigen_vector_parameter<Eigen::Vector3d>(node, initiator_position_param_name);
}

//-----------------------------------------------------------------------------
std::string get_responder_name(rclcpp::Node::SharedPtr node)
{
  return get_parameter<std::string>(node, responder_name_param_name);
}

//-----------------------------------------------------------------------------
uint16_t get_responder_id(rclcpp::Node::SharedPtr node)
{
  return get_parameter<int64_t>(node, responder_id_param_name);
}

//-----------------------------------------------------------------------------
Eigen::Vector3d get_responder_position(rclcpp::Node::SharedPtr node)
{
  return get_eigen_vector_parameter<Eigen::Vector3d>(node, responder_position_param_name);
}

//-----------------------------------------------------------------------------
std::vector<std::string> get_initiators_names(rclcpp::Node::SharedPtr node)
{
  return get_vector_parameter<std::string>(node, initiators_names_param_name);
}

//-----------------------------------------------------------------------------
std::vector<uint16_t> get_initiators_ids(rclcpp::Node::SharedPtr node)
{
  std::vector<uint16_t> uint16_ids;
  for (const auto int64_id : get_vector_parameter<int64_t>(node, initiators_ids_param_name)) {
    assert(int64_id >= 0 && int64_id <= std::numeric_limits<uint16_t>::max());
    uint16_ids.push_back(static_cast<uint16_t>(int64_id));
  }
  return uint16_ids;
}

//-----------------------------------------------------------------------------
core::VectorOfEigenVector<Eigen::Vector3d> get_initiators_positions(rclcpp::Node::SharedPtr node)
{
  return get_eigen_vector_parameters<Eigen::Vector3d>(
    node, initiators_positions_param_name,
    get_initiators_names(node));
}

//-----------------------------------------------------------------------------
std::vector<std::string> get_responders_names(rclcpp::Node::SharedPtr node)
{
  return get_vector_parameter<std::string>(node, responders_names_param_name);
}

//-----------------------------------------------------------------------------
std::vector<uint16_t> get_responders_ids(rclcpp::Node::SharedPtr node)
{
  std::vector<uint16_t> uint16_ids;
  for (const auto int64_id : get_vector_parameter<int64_t>(node, responders_ids_param_name)) {
    assert(int64_id >= 0 && int64_id <= std::numeric_limits<uint16_t>::max());
    uint16_ids.push_back(static_cast<uint16_t>(int64_id));
  }
  return uint16_ids;
}

//-----------------------------------------------------------------------------
core::VectorOfEigenVector<Eigen::Vector3d> get_responders_positions(rclcpp::Node::SharedPtr node)
{
  return get_eigen_vector_parameters<Eigen::Vector3d>(
    node, responders_positions_param_name,
    get_responders_names(node));
}

//-----------------------------------------------------------------------------
bool get_enable_scheduler(rclcpp::Node::SharedPtr node)
{
  return get_parameter<bool>(node, enable_sheduler_param_name);
}

}  // namespace ros2
}  // namespace romea
