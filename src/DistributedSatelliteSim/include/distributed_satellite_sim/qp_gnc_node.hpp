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

#ifndef DISTRIBUTED_SATELLITE_SIM__QP_GNC_NODE_HPP_
#define DISTRIBUTED_SATELLITE_SIM__QP_GNC_NODE_HPP_

#include <memory>

#include "Eigen/Dense"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "distributed_satellite_sim/srv/actuation_cmd.hpp"

// Pre-computed QP matrices (static const Eigen::Map objects)
#include "matrices/QQ.hpp"
#include "matrices/H1.hpp"
#include "matrices/Aineq.hpp"
#include "matrices/Bineq.hpp"

// Vendored QuadProg++
#include "QuadProg++.hh"

using ActuationCmd = distributed_satellite_sim::srv::ActuationCmd;

// Eigen <-> QuadProg++ conversion helpers 
namespace qp_convert
{

inline quadprogpp::Matrix<double> eigenToQP(const Eigen::MatrixXd & M)
{
  quadprogpp::Matrix<double> Q(M.rows(), M.cols());
  for (int i = 0; i < M.rows(); ++i) {
    for (int j = 0; j < M.cols(); ++j) {
      Q[i][j] = M(i, j);
    }
  }
  return Q;
}

inline quadprogpp::Vector<double> eigenToQP(const Eigen::VectorXd & v)
{
  quadprogpp::Vector<double> q(v.size());
  for (int i = 0; i < v.size(); ++i) {
    q[i] = v(i);
  }
  return q;
}

inline Eigen::VectorXd qpToEigen(const quadprogpp::Vector<double> & v)
{
  Eigen::VectorXd e(v.size());
  for (int i = 0; i < v.size(); ++i) {
    e(i) = v[i];
  }
  return e;
}

}  // namespace qp_convert

// QpGncNode
class QpGncNode : public rclcpp::Node
{
public:
  QpGncNode()
  : Node("qp_gnc_node")
  {
    // Subscribe to state published by env_node
    state_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
      "env_data", 10,
      std::bind(&QpGncNode::state_callback, this, std::placeholders::_1));

    // Service client matching env_node's actuation_cmd service
    cmd_client_ = create_client<ActuationCmd>("actuation_cmd");

    RCLCPP_INFO(get_logger(),
      "QP-MPC GNC node started (horizon=15, dim=45, ineq=90, u_max=0.01)");
  }

private:
  void state_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < 6) {
      RCLCPP_WARN(get_logger(), "env_data has < 6 elements, skipping");
      return;
    }

    // Map incoming state vector
    Eigen::VectorXd X = Eigen::Map<const Eigen::VectorXd>(msg->data.data(), 6);

    RCLCPP_INFO(get_logger(),
      "Received state: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
      X(0), X(1), X(2), X(3), X(4), X(5));

    // Build QP in Eigen  (identical to quadprogMPC_roundtrip.cpp)
    Eigen::MatrixXd G_e   = QQ;                     // 45x45 Hessian
    Eigen::VectorXd g0_e  = H1.transpose() * X;     // 45x1  linear cost
    Eigen::MatrixXd CI_e  = -Aineq.transpose();     // 45x90 (QuadProg++ sign)
    Eigen::VectorXd ci0_e = Bineq;                  // 90x1

    // Convert to QuadProg++ types  (cannot use Eigen directly)
    quadprogpp::Matrix<double> G   = qp_convert::eigenToQP(G_e);
    quadprogpp::Vector<double> g0  = qp_convert::eigenToQP(g0_e);
    quadprogpp::Matrix<double> CE(45, 0);            // no equality constraints
    quadprogpp::Vector<double> ce0(0);
    quadprogpp::Matrix<double> CI  = qp_convert::eigenToQP(CI_e);
    quadprogpp::Vector<double> ci0 = qp_convert::eigenToQP(ci0_e);
    quadprogpp::Vector<double> xqp(45);

    for (int i = 0; i < 45; ++i) {
      xqp[i] = 0.0;
    }

    // Solve
    double cost = quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, xqp);

    // Extract first control action from augmented decision vector
    Eigen::VectorXd Utot = qp_convert::qpToEigen(xqp);
    Eigen::Vector3d u = Utot.segment<3>(0);

    RCLCPP_INFO(get_logger(),
      "QP solved — u = [%.6e, %.6e, %.6e]  cost = %.4e",
      u(0), u(1), u(2), cost);

    // Send control via actuation_cmd service
    if (!cmd_client_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "actuation_cmd service not ready, skipping");
      return;
    }

    auto request = std::make_shared<ActuationCmd::Request>();
    request->thrust = {u(0), u(1), u(2)};

    cmd_client_->async_send_request(
      request,
      [this](rclcpp::Client<ActuationCmd>::SharedFuture future) {
        auto res = future.get();
        if (res->success) {
          RCLCPP_INFO(get_logger(), "Actuation command accepted");
        } else {
          RCLCPP_WARN(get_logger(), "Actuation command rejected");
        }
      });
  }

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr state_sub_;
  rclcpp::Client<ActuationCmd>::SharedPtr cmd_client_;
};

#endif  // DISTRIBUTED_SATELLITE_SIM__QP_GNC_NODE_HPP_