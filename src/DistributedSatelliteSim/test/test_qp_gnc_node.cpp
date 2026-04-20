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
//
// Unit tests for QP-MPC GNC controller solve logic.

#include "test_qp_gnc_node.hpp"

#include <gtest/gtest.h>

#include <cmath>

#include "Eigen/Dense"

#include "distributed_satellite_sim/matrices/QQ.hpp"
#include "distributed_satellite_sim/matrices/H1.hpp"
#include "distributed_satellite_sim/matrices/Aineq.hpp"
#include "distributed_satellite_sim/matrices/Bineq.hpp"

#include "QuadProg++.hh"

// ---------------------------------------------------------------------------
// Helpers (same as in qp_gnc_node.hpp)
// ---------------------------------------------------------------------------
namespace
{

quadprogpp::Matrix<double> eigenToQP(const Eigen::MatrixXd & M)
{
  quadprogpp::Matrix<double> Q(M.rows(), M.cols());
  for (int i = 0; i < M.rows(); ++i) {
    for (int j = 0; j < M.cols(); ++j) {
      Q[i][j] = M(i, j);
    }
  }
  return Q;
}

quadprogpp::Vector<double> eigenToQP(const Eigen::VectorXd & v)
{
  quadprogpp::Vector<double> q(v.size());
  for (int i = 0; i < v.size(); ++i) {
    q[i] = v(i);
  }
  return q;
}

Eigen::VectorXd qpToEigen(const quadprogpp::Vector<double> & v)
{
  Eigen::VectorXd e(v.size());
  for (int i = 0; i < (int)v.size(); ++i) {
    e(i) = v[i];
  }
  return e;
}

Eigen::Vector3d solve_qp_for_state(const Eigen::VectorXd & X)
{
  Eigen::MatrixXd G_e = QQ;
  Eigen::VectorXd g0_e = H1.transpose() * X;
  Eigen::MatrixXd CI_e = -Aineq.transpose();
  Eigen::VectorXd ci0_e = Bineq;

  quadprogpp::Matrix<double> G = eigenToQP(G_e);
  quadprogpp::Vector<double> g0 = eigenToQP(g0_e);
  quadprogpp::Matrix<double> CE(45, 0);
  quadprogpp::Vector<double> ce0(0);
  quadprogpp::Matrix<double> CI = eigenToQP(CI_e);
  quadprogpp::Vector<double> ci0 = eigenToQP(ci0_e);
  quadprogpp::Vector<double> xqp(45);

  for (int i = 0; i < 45; ++i) {
    xqp[i] = 0.0;
  }

  quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, xqp);

  Eigen::VectorXd Utot = qpToEigen(xqp);
  return Utot.segment<3>(0);
}

constexpr double U_MAX = 0.01;

}  // namespace

// ---------- QP matrix dimension tests ----------

TEST(QpMatricesTest, QQDimensions)
{
  EXPECT_EQ(QQ.rows(), 45);
  EXPECT_EQ(QQ.cols(), 45);
}

TEST(QpMatricesTest, H1Dimensions)
{
  EXPECT_EQ(H1.rows(), 6);
  EXPECT_EQ(H1.cols(), 45);
}

TEST(QpMatricesTest, AineqDimensions)
{
  EXPECT_EQ(Aineq.rows(), 90);
  EXPECT_EQ(Aineq.cols(), 45);
}

TEST(QpMatricesTest, BineqDimensions)
{
  EXPECT_EQ(Bineq.size(), 90);
}

TEST(QpMatricesTest, QQSymmetric)
{
  Eigen::MatrixXd Q = QQ;
  double asym = (Q - Q.transpose()).norm();
  EXPECT_LT(asym, 1e-4) << "QQ should be symmetric, asymmetry norm = " << asym;
}

TEST(QpMatricesTest, BineqAllPositive)
{
  for (int i = 0; i < Bineq.size(); ++i) {
    EXPECT_GT(Bineq(i), 0.0) << "Bineq(" << i << ") should be positive";
  }
}

// ---------- QP solve tests ----------

TEST(QpSolveTest, InitialStateSaturatedControl)
{
  // At X0 = [20, 20, 20, 0, 0, 0], all axes are far from origin.
  // Controller should saturate at ±u_max on each axis.
  Eigen::VectorXd X(6);
  X << 20.0, 20.0, 20.0, 0.0, 0.0, 0.0;

  Eigen::Vector3d u = solve_qp_for_state(X);

  // All components should be at -u_max (driving toward origin)
  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(std::abs(u(i)), U_MAX, 1e-3)
      << "u(" << i << ") = " << u(i) << " should be saturated at ±" << U_MAX;
  }
}

TEST(QpSolveTest, ZeroStateZeroControl)
{
  // At the origin, optimal control should be zero.
  Eigen::VectorXd X = Eigen::VectorXd::Zero(6);

  Eigen::Vector3d u = solve_qp_for_state(X);

  for (int i = 0; i < 3; ++i) {
    EXPECT_NEAR(u(i), 0.0, 1e-8)
      << "u(" << i << ") should be ~0 at origin, got " << u(i);
  }
}

TEST(QpSolveTest, ControlRespectsBoxConstraints)
{
  // For an arbitrary state, all controls must be within [-u_max, u_max].
  Eigen::VectorXd X(6);
  X << 5.0, -3.0, 10.0, 0.001, -0.02, 0.005;

  Eigen::Vector3d u = solve_qp_for_state(X);

  for (int i = 0; i < 3; ++i) {
    EXPECT_LE(u(i), U_MAX + 1e-10)
      << "u(" << i << ") = " << u(i) << " exceeds upper bound";
    EXPECT_GE(u(i), -U_MAX - 1e-10)
      << "u(" << i << ") = " << u(i) << " exceeds lower bound";
  }
}

TEST(QpSolveTest, OppositeStateFlipsControl)
{
  // For state in opposite direction, control should flip sign.
  Eigen::VectorXd Xpos(6), Xneg(6);
  Xpos << 10.0, 10.0, 10.0, 0.0, 0.0, 0.0;
  Xneg << -10.0, -10.0, -10.0, 0.0, 0.0, 0.0;

  Eigen::Vector3d u_pos = solve_qp_for_state(Xpos);
  Eigen::Vector3d u_neg = solve_qp_for_state(Xneg);

  for (int i = 0; i < 3; ++i) {
    // Signs should be opposite (or both zero)
    EXPECT_LT(u_pos(i) * u_neg(i), 1e-10)
      << "u_pos(" << i << ")=" << u_pos(i)
      << " and u_neg(" << i << ")=" << u_neg(i)
      << " should have opposite signs";
  }
}

TEST(QpSolveTest, SmallStateProprtionalControl)
{
  // Near origin, control should be small (not saturated).
  Eigen::VectorXd X(6);
  X << 1e-3, 1e-3, 1e-3, 0.0, 0.0, 0.0;

  Eigen::Vector3d u = solve_qp_for_state(X);

  for (int i = 0; i < 3; ++i) {
    EXPECT_LT(std::abs(u(i)), U_MAX * 0.5)
      << "u(" << i << ") = " << u(i)
      << " should be well below saturation for small state";
  }
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
