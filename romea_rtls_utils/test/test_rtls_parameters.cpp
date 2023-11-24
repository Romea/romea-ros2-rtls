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
#include <memory>

// gtest
#include "gtest/gtest.h"

// local
#include "romea_rtls_utils/rtls_parameters.hpp"

#include "../test/test_helper.h"

class TestTransceiverParameters : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }

  void SetUp() override
  {
    rclcpp::NodeOptions no;

    no.arguments(
      {"--ros-args",
        "--params-file",
        std::string(TEST_DIR) + "/test_rtls_parameters.yaml"});

    node = std::make_shared<rclcpp::Node>("test_rtls_parameters", "ns", no);
  }

  std::shared_ptr<rclcpp::Node> node;
};

TEST_F(TestTransceiverParameters, getPollRate) {
  romea::ros2::declare_poll_rate(node);
  EXPECT_DOUBLE_EQ(romea::ros2::get_poll_rate(node), 20.0);
}

TEST_F(TestTransceiverParameters, getMinimalRange) {
  romea::ros2::declare_minimal_range(node);
  EXPECT_DOUBLE_EQ(romea::ros2::get_minimal_range(node), 0.5);
}

TEST_F(TestTransceiverParameters, getMaximalRange) {
  romea::ros2::declare_maximal_range(node);
  EXPECT_DOUBLE_EQ(romea::ros2::get_maximal_range(node), 20.0);
}

TEST_F(TestTransceiverParameters, getInitiatorName) {
  romea::ros2::declare_initiator_name(node);
  EXPECT_STREQ(romea::ros2::get_initiator_name(node).c_str(), "i1");
}

TEST_F(TestTransceiverParameters, getInitiatorId) {
  romea::ros2::declare_initiator_id(node);
  EXPECT_EQ(romea::ros2::get_initiator_id(node), 1);
}

TEST_F(TestTransceiverParameters, getInitiatorPosition) {
  romea::ros2::declare_initiator_position(node);
  auto initiator_position = romea::ros2::get_initiator_position(node);
  EXPECT_EQ(initiator_position[0], 1);
  EXPECT_EQ(initiator_position[1], 2);
  EXPECT_EQ(initiator_position[2], 3);
}

TEST_F(TestTransceiverParameters, getResponderName) {
  romea::ros2::declare_responder_name(node);
  EXPECT_STREQ(romea::ros2::get_responder_name(node).c_str(), "i4");
}

TEST_F(TestTransceiverParameters, getResponderId) {
  romea::ros2::declare_responder_id(node);
  EXPECT_EQ(romea::ros2::get_responder_id(node), 4);
}

TEST_F(TestTransceiverParameters, getResponderPosition) {
  romea::ros2::declare_responder_position(node);
  auto responder_position = romea::ros2::get_responder_position(node);
  EXPECT_EQ(responder_position[0], 11);
  EXPECT_EQ(responder_position[1], 12);
  EXPECT_EQ(responder_position[2], 13);
}

TEST_F(TestTransceiverParameters, getInitiatorsNames) {
  romea::ros2::declare_initiators_names(node);
  auto initiators_names = romea::ros2::get_initiators_names(node);
  EXPECT_STREQ(initiators_names[0].c_str(), "i1");
  EXPECT_STREQ(initiators_names[1].c_str(), "i2");
  EXPECT_STREQ(initiators_names[2].c_str(), "i3");
}

TEST_F(TestTransceiverParameters, getInitiatorsIds) {
  romea::ros2::declare_initiators_ids(node);
  auto initiators_ids = romea::ros2::get_initiators_ids(node);
  EXPECT_EQ(initiators_ids[0], 1);
  EXPECT_EQ(initiators_ids[1], 2);
  EXPECT_EQ(initiators_ids[2], 3);
}

TEST_F(TestTransceiverParameters, getInitiatorsPositions) {
  romea::ros2::declare_initiators_names(node);
  romea::ros2::declare_initiators_positions(node);
  auto initiators_positions = romea::ros2::get_initiators_positions(node);
  EXPECT_EQ(initiators_positions[0][0], 1);
  EXPECT_EQ(initiators_positions[0][1], 2);
  EXPECT_EQ(initiators_positions[0][2], 3);
  EXPECT_EQ(initiators_positions[1][0], 4);
  EXPECT_EQ(initiators_positions[1][1], 5);
  EXPECT_EQ(initiators_positions[1][2], 6);
  EXPECT_EQ(initiators_positions[2][0], 7);
  EXPECT_EQ(initiators_positions[2][1], 8);
  EXPECT_EQ(initiators_positions[2][2], 9);
}

TEST_F(TestTransceiverParameters, getRespondersNames) {
  romea::ros2::declare_responders_names(node);
  auto responders_names = romea::ros2::get_responders_names(node);
  EXPECT_STREQ(responders_names[0].c_str(), "r1");
  EXPECT_STREQ(responders_names[1].c_str(), "r2");
  EXPECT_STREQ(responders_names[2].c_str(), "r3");
}

TEST_F(TestTransceiverParameters, getRespondersIds) {
  romea::ros2::declare_responders_ids(node);
  auto responders_ids = romea::ros2::get_responders_ids(node);
  EXPECT_EQ(responders_ids[0], 4);
  EXPECT_EQ(responders_ids[1], 5);
  EXPECT_EQ(responders_ids[2], 6);
}

TEST_F(TestTransceiverParameters, getRespondersPositions) {
  romea::ros2::declare_responders_names(node);
  romea::ros2::declare_responders_positions(node);
  auto responders_positions = romea::ros2::get_responders_positions(node);
  EXPECT_EQ(responders_positions[0][0], 11);
  EXPECT_EQ(responders_positions[0][1], 12);
  EXPECT_EQ(responders_positions[0][2], 13);
  EXPECT_EQ(responders_positions[1][0], 14);
  EXPECT_EQ(responders_positions[1][1], 15);
  EXPECT_EQ(responders_positions[1][2], 16);
  EXPECT_EQ(responders_positions[2][0], 17);
  EXPECT_EQ(responders_positions[2][1], 18);
  EXPECT_EQ(responders_positions[2][2], 19);
}


//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
