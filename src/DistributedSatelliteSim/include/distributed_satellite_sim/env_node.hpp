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

#ifndef DISTRIBUTED_SATELLITE_SIM__ENV_NODE_HPP_
#define DISTRIBUTED_SATELLITE_SIM__ENV_NODE_HPP_
 
#include <memory>
#include <vector>
 
#include "Eigen/Dense"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "distributed_satellite_sim/msg/actuation_sample.hpp"
#include "distributed_satellite_sim/srv/actuation_cmd.hpp"

using ActuationCmd = distributed_satellite_sim::srv::ActuationCmd;
using ActuationSample = distributed_satellite_sim::msg::ActuationSample;


class EnvNode : public rclcpp::Node
{
public:
  EnvNode()
  : Node("env_node")
  {
    // ---- DLQR defaults 
    std::vector<double> Ad_default = {
      1.25645279151274, 0, 0, 349.682205848334, 147.7805051, 0,
      -0.0716204647063059, 1, 0, -147.7805051, 318.728823393334, 0,
      0, 0, 0.914515736162421, 0, 0, 349.682205848334,
      0.0014040831838806, 0, 0, 0.914515736162421, 0.809100657054006, 0,
      -0.00059338484671504, 0, 0, -0.809100657054006, 0.658062944649682, 0,
      0, 0, -0.000468027727960198, 0, 0, 0.914515736162421
    };
 
    std::vector<double> Bd_default = {
      63868.7072544296, 17836.8364281426, 0,
      -17836.8364281426, 61074.8290177183, 0,
      0, 0, 63868.7072544296,
      349.682205848334, 147.7805051, 0,
      -147.7805051, 318.728823393334, 0,
      0, 0, 349.682205848334
    };
 
    std::vector<double> X0_default = {20, 20, 20, 0.00930458, -0.0467472, 0.00798343};
 
    declare_parameter("max_steps", 91);
    declare_parameter("min_subscribers", 0);
    declare_parameter("timer_period_ms", 100);
    declare_parameter("Ad", Ad_default);
    declare_parameter("Bd", Bd_default);
    declare_parameter("X0", X0_default);
    declare_parameter("tol_pos", 1e-4);
    declare_parameter("tol_vel", 1e-4);
    declare_parameter("enable_docking_check", false);
 
    max_steps_ = get_parameter("max_steps").as_int();
    min_subscribers_ = get_parameter("min_subscribers").as_int();
    int timer_ms = get_parameter("timer_period_ms").as_int();
    tol_pos_ = get_parameter("tol_pos").as_double();
    tol_vel_ = get_parameter("tol_vel").as_double();
    enable_docking_check_ = get_parameter("enable_docking_check").as_bool();
 
    auto Ad_vec = get_parameter("Ad").as_double_array();
    auto Bd_vec = get_parameter("Bd").as_double_array();
    auto X0_vec = get_parameter("X0").as_double_array();
 
    Ad_ = Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(Ad_vec.data());
    Bd_ = Eigen::Map<Eigen::Matrix<double, 6, 3, Eigen::RowMajor>>(Bd_vec.data());
    x_now_ = Eigen::Map<Eigen::VectorXd>(X0_vec.data(), 6);
    u_now_ = Eigen::Vector3d::Zero();
 
    // Output matrices for docking termination
    C_pos_ = Eigen::Matrix<double, 3, 6>::Zero();
    C_pos_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    C_vel_ = Eigen::Matrix<double, 3, 6>::Zero();
    C_vel_.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    state_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("env_data", 10);
    actuation_applied_pub_ = create_publisher<ActuationSample>("actuation_applied", 10);
 
    cmd_srv_ = create_service<ActuationCmd>(
      "actuation_cmd",
      [this](const std::shared_ptr<ActuationCmd::Request> req,
      std::shared_ptr<ActuationCmd::Response> res) {
        actuation_callback(req, res);
      });
 
    timer_ = create_wall_timer(
      std::chrono::milliseconds(timer_ms),
      std::bind(&EnvNode::timer_callback, this));
 
    RCLCPP_INFO(get_logger(),
      "Env node started (max_steps=%d, timer=%dms, docking_check=%s)",
      max_steps_, timer_ms, enable_docking_check_ ? "on" : "off");
  }
 
private:
  void timer_callback()
  {
    if (!sim_started_) {
      if (static_cast<int>(state_pub_->get_subscription_count()) < min_subscribers_) {
        return;
      }
      sim_started_ = true;
    }
 
    if (max_steps_ > 0 && step_ >= max_steps_) {
      timer_->cancel();
      RCLCPP_INFO(get_logger(), "Simulation complete after %d steps", step_);
      return;
    }
 
    x_now_ = Ad_ * x_now_ + Bd_ * u_now_;
 
    auto msg = std_msgs::msg::Float64MultiArray();
    msg.data.assign(x_now_.data(), x_now_.data() + x_now_.size());
    state_pub_->publish(msg);
 
    RCLCPP_INFO(get_logger(),
      "step %d: x = [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
      step_,
      x_now_(0), x_now_(1), x_now_(2),
      x_now_(3), x_now_(4), x_now_(5));
 
    ++step_;
 
    // Docking termination check
    if (enable_docking_check_) {
      double pos_norm = (C_pos_ * x_now_).norm();
      double vel_norm = (C_vel_ * x_now_).norm();
      if (pos_norm < tol_pos_ && vel_norm < tol_vel_) {
        timer_->cancel();
        RCLCPP_INFO(get_logger(),
          "=== DOCKING ACHIEVED at step %d  ||pos||=%.2e  ||vel||=%.2e ===",
          step_, pos_norm, vel_norm);
      }
    }
  }
 
  void actuation_callback(
    const std::shared_ptr<ActuationCmd::Request> req,
    std::shared_ptr<ActuationCmd::Response> res)
  {
    u_now_ << req->thrust[0], req->thrust[1], req->thrust[2];
    res->success = true;

    ActuationSample sample;
    sample.stamp = now();
    sample.actuation = {u_now_(0), u_now_(1), u_now_(2)};
    actuation_applied_pub_->publish(sample);
 
    RCLCPP_INFO(get_logger(),
      "actuation_cmd received: u = [%.6e, %.6e, %.6e]",
      u_now_(0), u_now_(1), u_now_(2));
  }
 
  Eigen::Matrix<double, 6, 1> x_now_;
  Eigen::Vector3d u_now_;
  int step_ = 0;
  int max_steps_;
  int min_subscribers_;
  bool sim_started_ = false;
 
  Eigen::Matrix<double, 6, 6> Ad_;
  Eigen::Matrix<double, 6, 3> Bd_;
 
  // Output matrices
  Eigen::Matrix<double, 3, 6> C_pos_;
  Eigen::Matrix<double, 3, 6> C_vel_;
  double tol_pos_;
  double tol_vel_;
  bool enable_docking_check_;
 
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  rclcpp::Publisher<ActuationSample>::SharedPtr actuation_applied_pub_;
  rclcpp::Service<ActuationCmd>::SharedPtr cmd_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};
 
#endif  // DISTRIBUTED_SATELLITE_SIM__ENV_NODE_HPP_
