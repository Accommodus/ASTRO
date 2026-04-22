// Copyright 2026 ASTRO
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

#include <chrono>
#include <memory>
#include <string>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "distributed_satellite_sim/circular_buffer.hpp"
#include "distributed_satellite_sim/env_node.hpp"
#include "distributed_satellite_sim/msg/actuation_sample.hpp"
#include "distributed_satellite_sim/msg/state_sample.hpp"
#include "distributed_satellite_sim/srv/actuation_cmd.hpp"
#include "distributed_satellite_sim/srv/get_recent_actuation_history.hpp"
#include "distributed_satellite_sim/srv/get_recent_env_history.hpp"
#include "distributed_satellite_sim/srv/get_recent_log_history.hpp"
#include "distributed_satellite_sim/telemetry_buffer_node.hpp"

using distributed_satellite_sim::CircularBuffer;
using ActuationCmd = distributed_satellite_sim::srv::ActuationCmd;
using ActuationSample = distributed_satellite_sim::msg::ActuationSample;
using GetRecentEnvHistory = distributed_satellite_sim::srv::GetRecentEnvHistory;
using GetRecentActuationHistory = distributed_satellite_sim::srv::GetRecentActuationHistory;
using GetRecentLogHistory = distributed_satellite_sim::srv::GetRecentLogHistory;

// ============================================================
// CircularBuffer unit tests (no ROS required)
// ============================================================

TEST(CircularBufferTest, PushBelowCapacity)
{
  CircularBuffer<int> buf(5);
  buf.push(1);
  buf.push(2);
  buf.push(3);
  EXPECT_EQ(buf.size(), 3u);
  EXPECT_EQ(buf.capacity(), 5u);
}

TEST(CircularBufferTest, RolloverKeepsLastN)
{
  CircularBuffer<int> buf(3);
  for (int i = 1; i <= 6; ++i) {
    buf.push(i);
  }
  EXPECT_EQ(buf.size(), 3u);
  auto entries = buf.get_recent(0);
  ASSERT_EQ(entries.size(), 3u);
  EXPECT_EQ(entries[0], 4);
  EXPECT_EQ(entries[1], 5);
  EXPECT_EQ(entries[2], 6);
}

TEST(CircularBufferTest, GetRecentLimitZeroReturnsAll)
{
  CircularBuffer<int> buf(10);
  for (int i = 0; i < 7; ++i) {
    buf.push(i);
  }
  auto entries = buf.get_recent(0);
  EXPECT_EQ(entries.size(), 7u);
}

TEST(CircularBufferTest, GetRecentLimitExceedingSize)
{
  CircularBuffer<int> buf(10);
  buf.push(42);
  buf.push(43);
  auto entries = buf.get_recent(100);
  ASSERT_EQ(entries.size(), 2u);
  EXPECT_EQ(entries[0], 42);
  EXPECT_EQ(entries[1], 43);
}

TEST(CircularBufferTest, GetRecentOldestToNewest)
{
  CircularBuffer<int> buf(5);
  for (int i = 10; i <= 50; i += 10) {
    buf.push(i);
  }
  auto entries = buf.get_recent(3);
  ASSERT_EQ(entries.size(), 3u);
  EXPECT_EQ(entries[0], 30);
  EXPECT_EQ(entries[1], 40);
  EXPECT_EQ(entries[2], 50);
}

TEST(CircularBufferTest, ExactCapacityNoRollover)
{
  CircularBuffer<int> buf(4);
  for (int i = 1; i <= 4; ++i) {
    buf.push(i);
  }
  EXPECT_EQ(buf.size(), 4u);
  auto entries = buf.get_recent(0);
  ASSERT_EQ(entries.size(), 4u);
  EXPECT_EQ(entries[0], 1);
  EXPECT_EQ(entries[3], 4);
}

// ============================================================
// TelemetryBufferNode integration tests (with ROS)
// ============================================================

class TelemetryBufferNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    rclcpp::NodeOptions opts;
    opts.append_parameter_override("env_buffer_capacity", 3);
    opts.append_parameter_override("actuation_buffer_capacity", 3);
    opts.append_parameter_override("log_buffer_capacity", 3);
    buf_node_ = std::make_shared<TelemetryBufferNode>(opts);

    helper_node_ = rclcpp::Node::make_shared("test_helper_buf");

    env_pub_ = helper_node_->create_publisher<std_msgs::msg::Float64MultiArray>(
      "env_data", 10);
    actuation_pub_ = helper_node_->create_publisher<ActuationSample>(
      "actuation_applied", 10);
    log_pub_ = helper_node_->create_publisher<rcl_interfaces::msg::Log>(
      "/rosout", 10);

    env_client_ = helper_node_->create_client<GetRecentEnvHistory>(
      "/telemetry_buffer/get_recent_env_history");
    actuation_client_ = helper_node_->create_client<GetRecentActuationHistory>(
      "/telemetry_buffer/get_recent_actuation_history");
    log_client_ = helper_node_->create_client<GetRecentLogHistory>(
      "/telemetry_buffer/get_recent_log_history");

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(buf_node_);
    executor_->add_node(helper_node_);
  }

  void TearDown() override
  {
    executor_->cancel();
    executor_->remove_node(helper_node_);
    executor_->remove_node(buf_node_);
    env_pub_.reset();
    actuation_pub_.reset();
    log_pub_.reset();
    env_client_.reset();
    actuation_client_.reset();
    log_client_.reset();
    helper_node_.reset();
    buf_node_.reset();
  }

  void spin_for(std::chrono::milliseconds duration)
  {
    auto start = std::chrono::steady_clock::now();
    while ((std::chrono::steady_clock::now() - start) < duration) {
      executor_->spin_some(std::chrono::milliseconds(10));
    }
  }

  template<typename SrvT>
  typename SrvT::Response::SharedPtr call_service(
    typename rclcpp::Client<SrvT>::SharedPtr client,
    typename SrvT::Request::SharedPtr request)
  {
    auto future = client->async_send_request(request);
    auto start = std::chrono::steady_clock::now();
    while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready &&
      (std::chrono::steady_clock::now() - start) < std::chrono::seconds(3))
    {
      executor_->spin_some(std::chrono::milliseconds(10));
    }
    if (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      return nullptr;
    }
    return future.get();
  }

  std::shared_ptr<TelemetryBufferNode> buf_node_;
  rclcpp::Node::SharedPtr helper_node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr env_pub_;
  rclcpp::Publisher<ActuationSample>::SharedPtr actuation_pub_;
  rclcpp::Publisher<rcl_interfaces::msg::Log>::SharedPtr log_pub_;
  rclcpp::Client<GetRecentEnvHistory>::SharedPtr env_client_;
  rclcpp::Client<GetRecentActuationHistory>::SharedPtr actuation_client_;
  rclcpp::Client<GetRecentLogHistory>::SharedPtr log_client_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(TelemetryBufferNodeTest, EnvBufferRolloverRetainsLastN)
{
  // Publish 5 messages into a capacity-3 buffer
  for (int i = 0; i < 5; ++i) {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {static_cast<double>(i), 0.0, 0.0, 0.0, 0.0, 0.0};
    env_pub_->publish(msg);
    spin_for(std::chrono::milliseconds(30));
  }

  auto req = std::make_shared<GetRecentEnvHistory::Request>();
  req->limit = 0;
  auto resp = call_service<GetRecentEnvHistory>(env_client_, req);

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->capacity, 3u);
  EXPECT_EQ(resp->total_buffered, 3u);
  ASSERT_EQ(resp->entries.size(), 3u);
  // Oldest retained entry should have state[0] == 2.0
  EXPECT_NEAR(resp->entries[0].state[0], 2.0, 1e-9);
  EXPECT_NEAR(resp->entries[2].state[0], 4.0, 1e-9);
}

TEST_F(TelemetryBufferNodeTest, EnvQueryLimitZeroReturnsAll)
{
  for (int i = 0; i < 2; ++i) {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {static_cast<double>(i), 0.0, 0.0, 0.0, 0.0, 0.0};
    env_pub_->publish(msg);
    spin_for(std::chrono::milliseconds(30));
  }

  auto req = std::make_shared<GetRecentEnvHistory::Request>();
  req->limit = 0;
  auto resp = call_service<GetRecentEnvHistory>(env_client_, req);

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->total_buffered, 2u);
  EXPECT_EQ(resp->entries.size(), 2u);
}

TEST_F(TelemetryBufferNodeTest, EnvQueryLimitExceedsAvailable)
{
  auto msg = std_msgs::msg::Float64MultiArray();
  msg.data = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  env_pub_->publish(msg);
  spin_for(std::chrono::milliseconds(50));

  auto req = std::make_shared<GetRecentEnvHistory::Request>();
  req->limit = 99;
  auto resp = call_service<GetRecentEnvHistory>(env_client_, req);

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->total_buffered, 1u);
  ASSERT_EQ(resp->entries.size(), 1u);
  EXPECT_NEAR(resp->entries[0].state[0], 1.0, 1e-9);
}

TEST_F(TelemetryBufferNodeTest, EnvQueryChronologicalOrder)
{
  for (int i = 1; i <= 3; ++i) {
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data = {static_cast<double>(i * 10), 0.0, 0.0, 0.0, 0.0, 0.0};
    env_pub_->publish(msg);
    spin_for(std::chrono::milliseconds(30));
  }

  auto req = std::make_shared<GetRecentEnvHistory::Request>();
  req->limit = 0;
  auto resp = call_service<GetRecentEnvHistory>(env_client_, req);

  ASSERT_NE(resp, nullptr);
  ASSERT_EQ(resp->entries.size(), 3u);
  EXPECT_LT(resp->entries[0].state[0], resp->entries[1].state[0]);
  EXPECT_LT(resp->entries[1].state[0], resp->entries[2].state[0]);
}

TEST_F(TelemetryBufferNodeTest, ActuationHistoryFromDirectPublish)
{
  ActuationSample sample;
  sample.stamp = helper_node_->now();
  sample.actuation = {1.1, 2.2, 3.3};
  actuation_pub_->publish(sample);
  spin_for(std::chrono::milliseconds(50));

  auto req = std::make_shared<GetRecentActuationHistory::Request>();
  req->limit = 0;
  auto resp = call_service<GetRecentActuationHistory>(actuation_client_, req);

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->capacity, 3u);
  EXPECT_EQ(resp->total_buffered, 1u);
  ASSERT_EQ(resp->entries.size(), 1u);
  EXPECT_NEAR(resp->entries[0].actuation[0], 1.1, 1e-9);
  EXPECT_NEAR(resp->entries[0].actuation[1], 2.2, 1e-9);
  EXPECT_NEAR(resp->entries[0].actuation[2], 3.3, 1e-9);
}

TEST_F(TelemetryBufferNodeTest, ActuationHistoryRollover)
{
  for (int i = 1; i <= 5; ++i) {
    ActuationSample sample;
    sample.stamp = helper_node_->now();
    sample.actuation = {static_cast<double>(i), 0.0, 0.0};
    actuation_pub_->publish(sample);
    spin_for(std::chrono::milliseconds(30));
  }

  auto req = std::make_shared<GetRecentActuationHistory::Request>();
  req->limit = 0;
  auto resp = call_service<GetRecentActuationHistory>(actuation_client_, req);

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->total_buffered, 3u);
  ASSERT_EQ(resp->entries.size(), 3u);
  EXPECT_NEAR(resp->entries[0].actuation[0], 3.0, 1e-9);
  EXPECT_NEAR(resp->entries[2].actuation[0], 5.0, 1e-9);
}

TEST_F(TelemetryBufferNodeTest, LogHistoryFromSyntheticRosout)
{
  rcl_interfaces::msg::Log log_msg;
  log_msg.level = rcl_interfaces::msg::Log::INFO;
  log_msg.name = "test_node";
  log_msg.msg = "hello from test";
  log_msg.stamp = helper_node_->now();
  log_pub_->publish(log_msg);
  spin_for(std::chrono::milliseconds(50));

  auto req = std::make_shared<GetRecentLogHistory::Request>();
  req->limit = 0;
  auto resp = call_service<GetRecentLogHistory>(log_client_, req);

  ASSERT_NE(resp, nullptr);
  EXPECT_EQ(resp->capacity, 3u);
  EXPECT_GE(resp->total_buffered, 1u);
  bool found = false;
  for (const auto & entry : resp->entries) {
    if (entry.msg == "hello from test") {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "Expected log message not found in log history";
}

TEST_F(TelemetryBufferNodeTest, ActuationHistoryFromActuationCmdPath)
{
  // Use env_node so the real actuation_cmd -> actuation_applied path is exercised
  auto env_node = std::make_shared<EnvNode>();
  executor_->add_node(env_node);

  auto actuation_client = helper_node_->create_client<ActuationCmd>("actuation_cmd");
  ASSERT_TRUE(actuation_client->wait_for_service(std::chrono::seconds(2)));

  auto req = std::make_shared<ActuationCmd::Request>();
  req->thrust = {1e-6, -2e-6, 3e-6};
  auto future = actuation_client->async_send_request(req);
  auto start = std::chrono::steady_clock::now();
  while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready &&
    (std::chrono::steady_clock::now() - start) < std::chrono::seconds(2))
  {
    executor_->spin_some(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  EXPECT_TRUE(future.get()->success);

  spin_for(std::chrono::milliseconds(100));

  auto hist_req = std::make_shared<GetRecentActuationHistory::Request>();
  hist_req->limit = 0;
  auto resp = call_service<GetRecentActuationHistory>(actuation_client_, hist_req);

  ASSERT_NE(resp, nullptr);
  ASSERT_GE(resp->entries.size(), 1u);
  EXPECT_NEAR(resp->entries.back().actuation[0], 1e-6, 1e-12);
  EXPECT_NEAR(resp->entries.back().actuation[1], -2e-6, 1e-12);
  EXPECT_NEAR(resp->entries.back().actuation[2], 3e-6, 1e-12);

  executor_->remove_node(env_node);
}
