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


#ifndef ROMEA_RTLS_COORDINATORS__RTLS_COORDINATORS_PARAMETERS_HPP_
#define ROMEA_RTLS_COORDINATORS__RTLS_COORDINATORS_PARAMETERS_HPP_

// std
//#include <map>
#include <string>
#include <vector>

// ros
#include "rclcpp/rclcpp.hpp"

// romea
#include "romea_core_common/containers/Eigen/VectorOfEigenVector.hpp"

namespace romea
{
void declare_poll_rate(rclcpp::Node::SharedPtr node);
void declare_minimal_range(rclcpp::Node::SharedPtr node);
void declare_maximal_range(rclcpp::Node::SharedPtr node);
void declare_initiators_names(rclcpp::Node::SharedPtr node);
void declare_initiators_ids(rclcpp::Node::SharedPtr node);
void declare_initiators_positions(rclcpp::Node::SharedPtr node);
void declare_responders_names(rclcpp::Node::SharedPtr node);
void declare_responders_ids(rclcpp::Node::SharedPtr node);
void declare_responders_positions(rclcpp::Node::SharedPtr node);

double get_poll_rate(rclcpp::Node::SharedPtr node);
double get_minimal_range(rclcpp::Node::SharedPtr node);
double get_maximal_range(rclcpp::Node::SharedPtr node);
std::vector<std::string> get_initiators_names(rclcpp::Node::SharedPtr node);
std::vector<uint16_t> get_initiators_ids(rclcpp::Node::SharedPtr node);
VectorOfEigenVector<Eigen::Vector3d> get_initiators_positions(rclcpp::Node::SharedPtr node);
std::vector<std::string> get_responders_names(rclcpp::Node::SharedPtr node);
std::vector<uint16_t> get_responders_ids(rclcpp::Node::SharedPtr node);
VectorOfEigenVector<Eigen::Vector3d> get_responders_positions(rclcpp::Node::SharedPtr node);

}  // namespace romea

#endif   // ROMEA_RTLS_COORDINATORS__RTLS_COORDINATORS_PARAMETERS_HPP_
