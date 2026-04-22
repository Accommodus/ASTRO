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

#ifndef DISTRIBUTED_SATELLITE_SIM__TELEMETRY_BUFFER_NODE_HPP_
#define DISTRIBUTED_SATELLITE_SIM__TELEMETRY_BUFFER_NODE_HPP_

#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/log.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "distributed_satellite_sim/circular_buffer.hpp"
#include "distributed_satellite_sim/msg/actuation_sample.hpp"
#include "distributed_satellite_sim/msg/state_sample.hpp"
#include "distributed_satellite_sim/srv/get_recent_actuation_history.hpp"
#include "distributed_satellite_sim/srv/get_recent_env_history.hpp"
#include "distributed_satellite_sim/srv/get_recent_log_history.hpp"

using StateSample = distributed_satellite_sim::msg::StateSample;
using ActuationSample = distributed_satellite_sim::msg::ActuationSample;
using GetRecentEnvHistory = distributed_satellite_sim::srv::GetRecentEnvHistory;
using GetRecentActuationHistory = distributed_satellite_sim::srv::GetRecentActuationHistory;
using GetRecentLogHistory = distributed_satellite_sim::srv::GetRecentLogHistory;

class TelemetryBufferNode : public rclcpp::Node
{
public:
  explicit TelemetryBufferNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("telemetry_buffer", options)
  {
    declare_parameter("env_buffer_capacity", 100);
    declare_parameter("actuation_buffer_capacity", 100);
    declare_parameter("log_buffer_capacity", 100);

    int env_cap = get_parameter("env_buffer_capacity").as_int();
    int act_cap = get_parameter("actuation_buffer_capacity").as_int();
    int log_cap = get_parameter("log_buffer_capacity").as_int();

    if (env_cap <= 0 || act_cap <= 0 || log_cap <= 0) {
      throw std::invalid_argument(
        "Buffer capacities must be > 0. Got: env=" + std::to_string(env_cap) +
        " actuation=" + std::to_string(act_cap) +
        " log=" + std::to_string(log_cap));
    }

    env_buf_ = std::make_unique<distributed_satellite_sim::CircularBuffer<StateSample>>(
      static_cast<std::size_t>(env_cap));
    actuation_buf_ =
      std::make_unique<distributed_satellite_sim::CircularBuffer<ActuationSample>>(
      static_cast<std::size_t>(act_cap));
    log_buf_ =
      std::make_unique<distributed_satellite_sim::CircularBuffer<rcl_interfaces::msg::Log>>(
      static_cast<std::size_t>(log_cap));

    env_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "env_data", 10,
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        StateSample sample;
        sample.stamp = now();
        sample.state = msg->data;
        std::lock_guard<std::mutex> lock(env_mutex_);
        env_buf_->push(sample);
      });

    actuation_sub_ = create_subscription<ActuationSample>(
      "actuation_applied", 10,
      [this](const ActuationSample::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(actuation_mutex_);
        actuation_buf_->push(*msg);
      });

    log_sub_ = create_subscription<rcl_interfaces::msg::Log>(
      "/rosout", 10,
      [this](const rcl_interfaces::msg::Log::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(log_mutex_);
        log_buf_->push(*msg);
      });

    env_hist_srv_ = create_service<GetRecentEnvHistory>(
      "~/get_recent_env_history",
      [this](
        const GetRecentEnvHistory::Request::SharedPtr req,
        GetRecentEnvHistory::Response::SharedPtr res)
      {
        std::lock_guard<std::mutex> lock(env_mutex_);
        res->entries = env_buf_->get_recent(req->limit);
        res->total_buffered = static_cast<uint32_t>(env_buf_->size());
        res->capacity = static_cast<uint32_t>(env_buf_->capacity());
      });

    actuation_hist_srv_ = create_service<GetRecentActuationHistory>(
      "~/get_recent_actuation_history",
      [this](
        const GetRecentActuationHistory::Request::SharedPtr req,
        GetRecentActuationHistory::Response::SharedPtr res)
      {
        std::lock_guard<std::mutex> lock(actuation_mutex_);
        res->entries = actuation_buf_->get_recent(req->limit);
        res->total_buffered = static_cast<uint32_t>(actuation_buf_->size());
        res->capacity = static_cast<uint32_t>(actuation_buf_->capacity());
      });

    log_hist_srv_ = create_service<GetRecentLogHistory>(
      "~/get_recent_log_history",
      [this](
        const GetRecentLogHistory::Request::SharedPtr req,
        GetRecentLogHistory::Response::SharedPtr res)
      {
        std::lock_guard<std::mutex> lock(log_mutex_);
        res->entries = log_buf_->get_recent(req->limit);
        res->total_buffered = static_cast<uint32_t>(log_buf_->size());
        res->capacity = static_cast<uint32_t>(log_buf_->capacity());
      });

    RCLCPP_INFO(get_logger(),
      "Telemetry buffer started (env_cap=%d, act_cap=%d, log_cap=%d)",
      env_cap, act_cap, log_cap);
  }

private:
  std::unique_ptr<distributed_satellite_sim::CircularBuffer<StateSample>> env_buf_;
  std::unique_ptr<distributed_satellite_sim::CircularBuffer<ActuationSample>> actuation_buf_;
  std::unique_ptr<distributed_satellite_sim::CircularBuffer<rcl_interfaces::msg::Log>> log_buf_;

  std::mutex env_mutex_;
  std::mutex actuation_mutex_;
  std::mutex log_mutex_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr env_sub_;
  rclcpp::Subscription<ActuationSample>::SharedPtr actuation_sub_;
  rclcpp::Subscription<rcl_interfaces::msg::Log>::SharedPtr log_sub_;

  rclcpp::Service<GetRecentEnvHistory>::SharedPtr env_hist_srv_;
  rclcpp::Service<GetRecentActuationHistory>::SharedPtr actuation_hist_srv_;
  rclcpp::Service<GetRecentLogHistory>::SharedPtr log_hist_srv_;
};

#endif  // DISTRIBUTED_SATELLITE_SIM__TELEMETRY_BUFFER_NODE_HPP_
