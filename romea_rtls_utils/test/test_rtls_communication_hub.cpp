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
#include <limits>
#include <string>
#include <memory>
#include <vector>
#include <thread>

// gtest
#include "gtest/gtest.h"

// romea
#include "romea_common_utils/qos.hpp"
#include "romea_common_utils/publishers/data_publisher.hpp"
#include "romea_common_utils/services/service_client_async.hpp"
#include "romea_rtls_transceiver_utils/transceiver_interface_server.hpp"

// local
#include "../test/test_helper.h"
#include "romea_rtls_communication_hub/rtls_communication_hub.hpp"
#include "romea_rtls_transceiver_msgs/srv/set_payload.hpp"

class Tag
{
public:
  using RangingRequest = romea_rtls_transceiver_msgs::msg::RangingRequest;
  using RangingResult = romea_rtls_transceiver_msgs::msg::RangingResult;

public:
  Tag(
    std::shared_ptr<rclcpp::Node> node,
    const uint16_t & transceiver_id)
  : transceiver_id_(transceiver_id)
  {
  }

private:
  void process_request(RangingRequest::ConstSharedPtr msg)
  {
    RangingResult result;
    result.initiator_id = uint16_t transceiver_id_;
    result.responder_id = msg->responder_id;
    interface_.send_result(result)
  }

private:
  uint16_t transceiver_id_;
  std::vector<unsigned char> payload_;
  std::unique_ptr<romea::RTLSTransceiverInterfaceServer> interface_;
};


class TestRTLSCommunicationHub : public ::testing::Test
{
public:
  using Hub = romea::RTLSCommunicationHub;
  using Poll = romea_rtls_msgs::msg::Poll;

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
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_thread_ = std::thread([this]() {this->executor_->spin();});

    node_hub = std::make_shared<rclcpp::Node>("test_rtls_transceiver_hub");
    node_tag0 = std::make_shared<rclcpp::Node>("test_rtls_transceiver_tag0", "tag0");
    node_tag1 = std::make_shared<rclcpp::Node>("test_rtls_transceiver_tag1", "tag1");

    executor_->add_node(node_tag0);
    tag0_ = std::make_shared<Tag>(node_tag0, 0);

    executor_->add_node(node_tag1);
    tag1_ = std::make_shared<Tag>(node_tag1, 1);

    rclcpp::NodeOptions no;
    no.arguments(
      {"--ros-args",
        "--params-file",
        std::string(TEST_DIR) + "/test_rtls_hub.yaml"});

    hub_ = std::make_shared<Hub>(no);
    executor_->add_node(hub_->get_node_base_interface());
    executor_->add_node(node_master);
    master_ = std::make_shared<Master>(node_master);
  }


  void TearDown() override
  {
    executor_->cancel();
    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }
  }

  std::shared_ptr<rclcpp::Node> node_tag0;
  std::shared_ptr<rclcpp::Node> node_tag1;
  std::shared_ptr<rclcpp::Node> node_hub;
  rclcpp::Executor::SharedPtr executor_;
  std::thread executor_thread_;

  std::shared_ptr<Tag> tag0_;
  std::shared_ptr<Tag> tag1_;
  std::shared < RTLSComm
  std::shared_ptr<Hub> hub_;
  std::shared_ptr<Master> master_;
};

TEST_F(TestRTLSTransceiverHub, check_poll_tag2_anchor_0) {
  Poll poll;
  poll.transceivers.initiator_name = "tag2";
  poll.transceivers.responder_name = "anchor0";
  master_->poll(poll);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(master_->range, nullptr);
}

TEST_F(TestRTLSTransceiverHub, check_poll_tag0_anchor_2) {
  Poll poll;
  poll.transceivers.initiator_name = "tag0";
  poll.transceivers.responder_name = "anchor2";
  master_->poll(poll);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_EQ(master_->range, nullptr);
}

TEST_F(TestRTLSTransceiverHub, check_poll_tag0_anchor_0) {
  Poll poll;
  poll.transceivers.initiator_name = "tag0";
  poll.transceivers.responder_name = "anchor0";
  master_->poll(poll);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_DOUBLE_EQ(master_->range->range.range, 255);
  EXPECT_EQ(master_->range->payload.data.size(), 0u);
}

TEST_F(TestRTLSTransceiverHub, check_poll_tag1_anchor_0) {
  Poll poll;
  poll.payload.data = {1, 2, 3, 4};
  poll.transceivers.initiator_name = "tag1";
  poll.transceivers.responder_name = "anchor0";
  master_->poll(poll);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_DOUBLE_EQ(master_->range->range.range, 65811);
  EXPECT_EQ(master_->range->payload.data.size(), 4u);
  EXPECT_EQ(master_->range->payload.data[0], 1);
  EXPECT_EQ(master_->range->payload.data[1], 2);
  EXPECT_EQ(master_->range->payload.data[2], 3);
  EXPECT_EQ(master_->range->payload.data[3], 4);
}

TEST_F(TestRTLSTransceiverHub, check_poll_tag0_anchor_1) {
  Poll poll;
  poll.transceivers.initiator_name = "tag0";
  poll.transceivers.responder_name = "anchor1";
  master_->poll(poll);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_DOUBLE_EQ(master_->range->range.range, 256);
  EXPECT_EQ(master_->range->payload.data.size(), 0u);
}

TEST_F(TestRTLSTransceiverHub, check_poll_tag1_anchor_1) {
  Poll poll;
  poll.payload.data = {1, 2, 3, 4};
  poll.transceivers.initiator_name = "tag1";
  poll.transceivers.responder_name = "anchor1";
  master_->poll(poll);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  EXPECT_DOUBLE_EQ(master_->range->range.range, 65812);
  EXPECT_EQ(master_->range->payload.data.size(), 4u);
  EXPECT_EQ(master_->range->payload.data[0], 1);
  EXPECT_EQ(master_->range->payload.data[1], 2);
  EXPECT_EQ(master_->range->payload.data[2], 3);
  EXPECT_EQ(master_->range->payload.data[3], 4);
}

//-----------------------------------------------------------------------------
int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
