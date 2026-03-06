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

#include <memory>

#include "gtest/gtest.h"
#include "Eigen/Dense"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "distributed_satellite_sim/env_node.hpp"
#include "distributed_satellite_sim/srv/actuation_cmd.hpp"

using ActuationCmd = distributed_satellite_sim::srv::ActuationCmd;

static Eigen::Matrix<double, 6, 6> test_Ad()
{
  Eigen::Matrix<double, 6, 6> Ad;
  Ad <<
    1.25645279151274, 0, 0, 349.682205848334, 147.7805051, 0,
    -0.0716204647063059, 1, 0, -147.7805051, 318.728823393334, 0,
    0, 0, 0.914515736162421, 0, 0, 349.682205848334,
    0.0014040831838806, 0, 0, 0.914515736162421, 0.809100657054006, 0,
    -0.00059338484671504, 0, 0, -0.809100657054006, 0.658062944649682, 0,
    0, 0, -0.000468027727960198, 0, 0, 0.914515736162421;
  return Ad;
}

static Eigen::Matrix<double, 6, 3> test_Bd()
{
  Eigen::Matrix<double, 6, 3> Bd;
  Bd <<
    63868.7072544296, 17836.8364281426, 0,
    -17836.8364281426, 61074.8290177183, 0,
    0, 0, 63868.7072544296,
    349.682205848334, 147.7805051, 0,
    -147.7805051, 318.728823393334, 0,
    0, 0, 349.682205848334;
  return Bd;
}

static Eigen::Matrix<double, 6, 1> test_x0()
{
  Eigen::Matrix<double, 6, 1> x;
  x << 20, 20, 20, 0.00930458, -0.0467472, 0.00798343;
  return x;
}

class EnvNodeTest : public ::testing::Test
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
    env_node_ = std::make_shared<EnvNode>();

    helper_node_ = rclcpp::Node::make_shared("test_helper");
    client_ = helper_node_->create_client<ActuationCmd>("actuation_cmd");
    received_msg_.reset();
    sub_ = helper_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "env_data", 10,
      [this](std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        received_msg_ = msg;
      });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(env_node_);
    executor_->add_node(helper_node_);
  }

  void TearDown() override
  {
    executor_->cancel();
    executor_->remove_node(helper_node_);
    executor_->remove_node(env_node_);
    sub_.reset();
    client_.reset();
    helper_node_.reset();
    env_node_.reset();
  }

  void spin_until_message(
    std::chrono::milliseconds timeout = std::chrono::milliseconds(2000))
  {
    auto start = std::chrono::steady_clock::now();
    while (!received_msg_ &&
      (std::chrono::steady_clock::now() - start) < timeout)
    {
      executor_->spin_some(std::chrono::milliseconds(10));
    }
  }

  bool call_actuation(double t0, double t1, double t2)
  {
    auto request = std::make_shared<ActuationCmd::Request>();
    request->thrust = {t0, t1, t2};
    auto future = client_->async_send_request(request);

    auto start = std::chrono::steady_clock::now();
    while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready &&
      (std::chrono::steady_clock::now() - start) < std::chrono::seconds(2))
    {
      executor_->spin_some(std::chrono::milliseconds(10));
    }
    if (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      return false;
    }
    return future.get()->success;
  }

  std::shared_ptr<EnvNode> env_node_;
  rclcpp::Node::SharedPtr helper_node_;
  rclcpp::Client<ActuationCmd>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  std_msgs::msg::Float64MultiArray::SharedPtr received_msg_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
};

TEST_F(EnvNodeTest, ZeroThrust)
{
  spin_until_message();
  ASSERT_NE(received_msg_, nullptr) << "No message received on env_data";

  auto Ad = test_Ad();
  auto Bd = test_Bd();
  auto x0 = test_x0();
  Eigen::Matrix<double, 6, 1> x_expected = Ad * x0 + Bd * Eigen::Vector3d::Zero();

  ASSERT_EQ(received_msg_->data.size(), 6u);
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(received_msg_->data[i], x_expected(i), 1e-6)
      << "Mismatch at element " << i;
  }
}

TEST_F(EnvNodeTest, NonZeroThrust)
{
  Eigen::Vector3d u(1e-7, -2e-7, 5e-8);

  // Service completes before the first timer tick (100ms period),
  // so the very first published state already uses the new thrust.
  ASSERT_TRUE(call_actuation(u(0), u(1), u(2)));

  spin_until_message();
  ASSERT_NE(received_msg_, nullptr) << "No message received on env_data";

  auto Ad = test_Ad();
  auto Bd = test_Bd();
  auto x0 = test_x0();
  Eigen::Matrix<double, 6, 1> x_expected = Ad * x0 + Bd * u;

  ASSERT_EQ(received_msg_->data.size(), 6u);
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(received_msg_->data[i], x_expected(i), 1e-6)
      << "Mismatch at element " << i;
  }
}

TEST_F(EnvNodeTest, DefaultInitialState)
{
  spin_until_message();
  ASSERT_NE(received_msg_, nullptr) << "No message received on env_data";

  auto Ad = test_Ad();
  auto Bd = test_Bd();
  auto x0 = test_x0();
  Eigen::Matrix<double, 6, 1> x_expected = Ad * x0 + Bd * Eigen::Vector3d::Zero();

  ASSERT_EQ(received_msg_->data.size(), 6u);
  for (int i = 0; i < 6; ++i) {
    EXPECT_NEAR(received_msg_->data[i], x_expected(i), 1e-6)
      << "Mismatch at element " << i;
  }
}

TEST_F(EnvNodeTest, ServiceReturnsSuccess)
{
  ASSERT_TRUE(client_->wait_for_service(std::chrono::seconds(2)))
    << "actuation_cmd service not available";
  EXPECT_TRUE(call_actuation(0.0, 0.0, 0.0));
}

TEST_F(EnvNodeTest, TopicPublishesCorrectSize)
{
  spin_until_message();
  ASSERT_NE(received_msg_, nullptr) << "No message received on env_data";
  EXPECT_EQ(received_msg_->data.size(), 6u);
}
