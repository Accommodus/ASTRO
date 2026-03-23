// Copyright 2026 Accommodus
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
//
// Unit tests for GncNode DLQR control law.

#include "test_gnc_node.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <optional>

#include "distributed_satellite_sim/gnc_node.hpp"

namespace
{

Eigen::Matrix<double, 3, 6> make_reference_K()
{
  Eigen::Matrix<double, 3, 6> K;
  K <<
    4.66705488313515e-06, -1.36363629843838e-06, 3.96098209388736e-22,
    0.00216637769262191, 0.000840447323685129, 7.35693727993496e-20,
    2.77991721268819e-06, 7.33444155253651e-07, 6.05699839509399e-22,
    0.000150571571893266, 0.00170682586161904, 1.18873969348727e-18,
    -3.71173845538396e-22, -1.12623951370299e-21, 6.24116221637017e-07,
    1.62911699574378e-18, 9.12299177441715e-19, 0.00135188823063242;
  return K;
}

Eigen::Vector3d compute_expected_u(const Eigen::Matrix<double, 6, 1> & x)
{
  static const Eigen::Matrix<double, 3, 6> K = make_reference_K();
  return -K * x;
}

void expect_vectors_near(const Eigen::Vector3d & a, const Eigen::Vector3d & b, double tol)
{
  EXPECT_NEAR(a(0), b(0), tol);
  EXPECT_NEAR(a(1), b(1), tol);
  EXPECT_NEAR(a(2), b(2), tol);
}

}  // namespace

// ---------- Direct compute_thrust tests ----------

TEST(GncNodeComputeThrustTest, ZeroState)
{
  auto node = std::make_shared<distributed_satellite_sim::GncNode>(false);

  Eigen::Matrix<double, 6, 1> x = Eigen::Matrix<double, 6, 1>::Zero();
  const Eigen::Vector3d u = node->compute_thrust(x);
  Eigen::Vector3d expected = compute_expected_u(x);

  expect_vectors_near(u, expected, 1e-12);
}

TEST(GncNodeComputeThrustTest, KnownState)
{
  auto node = std::make_shared<distributed_satellite_sim::GncNode>(false);

  Eigen::Matrix<double, 6, 1> x;
  x << 20.0, 20.0, 20.0, 0.00930458, -0.0467472, 0.00798343;

  const Eigen::Vector3d u = node->compute_thrust(x);
  Eigen::Vector3d expected = compute_expected_u(x);

  expect_vectors_near(u, expected, 1e-12);
}

TEST(GncNodeComputeThrustTest, LargeState)
{
  auto node = std::make_shared<distributed_satellite_sim::GncNode>(false);

  Eigen::Matrix<double, 6, 1> x;
  x << 1e4, 1e4, 1e4, 1e2, 1e2, 1e2;

  const Eigen::Vector3d u = node->compute_thrust(x);
  Eigen::Vector3d expected = compute_expected_u(x);

  expect_vectors_near(u, expected, 1e-8);
}

TEST(GncNodeComputeThrustTest, NegativeComponents)
{
  auto node = std::make_shared<distributed_satellite_sim::GncNode>(false);

  Eigen::Matrix<double, 6, 1> x;
  x << -10.0, -5.0, -20.0, -0.01, -0.05, -0.02;

  const Eigen::Vector3d u = node->compute_thrust(x);
  Eigen::Vector3d expected = compute_expected_u(x);

  expect_vectors_near(u, expected, 1e-10);
}

// Provide a custom main for gtest with rclcpp initialization.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
