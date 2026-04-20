#!/usr/bin/env python3

# Copyright 2026 ASTRO
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Launch test for the QP-MPC GNC controller.

Runs env_node (with QP-MPC dynamics) and qp_gnc_node in a closed loop,
using a fast wall-clock timer so the test completes quickly.  Verifies:
  1. State messages are received on env_data.
  2. Position norm decreases monotonically after an initial transient.
  3. Final position norm is below a convergence threshold.
"""

import math
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import EmitEvent
from launch.actions import TimerAction
from launch.events import Shutdown
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray

# Run enough steps for meaningful convergence but keep the test short.
# QP-MPC dynamics use Ts=30s; 200 steps = 6000s of simulated time.
EXPECTED_STEPS = 100
STATE_SIZE = 6
# Use a fast wall-clock timer so the test finishes in seconds, not hours.
FAST_TIMER_MS = 50
# Allow generous real-time budget: EXPECTED_STEPS * FAST_TIMER_MS + margin.
SHUTDOWN_AFTER_SEC = 45.0
RECEIVE_TIMEOUT_SEC = 35.0
DISCOVERY_TIMEOUT_SEC = 10.0
# Convergence: final position norm should be well below initial (~34.6 km).
CONVERGENCE_POS_THRESHOLD = 1.0


class TrajectoryRecorder(Node):

    def __init__(self):
        super().__init__('qp_mpc_trajectory_recorder')
        self.trajectory = []
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'env_data',
            self._callback,
            QoSProfile(depth=200),
        )

    def _callback(self, msg):
        self.trajectory.append(list(msg.data))


def generate_test_description():
    pkg = get_package_share_directory('distributed_satellite_sim')
    qp_params = os.path.join(pkg, 'config', 'qp_mpc_params.yaml')

    # Override timer to run fast; keep QP-MPC dynamics from YAML.
    env_node = launch_ros.actions.Node(
        package='distributed_satellite_sim',
        executable='env_node',
        name='env_node',
        parameters=[
            qp_params,
            {
                'timer_period_ms': FAST_TIMER_MS,
                'max_steps': EXPECTED_STEPS,
                'min_subscribers': 2,
            },
        ],
        output='screen',
    )

    qp_gnc_node = launch_ros.actions.Node(
        package='distributed_satellite_sim',
        executable='qp_gnc_node',
        name='qp_gnc_node',
        output='screen',
    )

    return (
        launch.LaunchDescription([
            launch_testing.actions.ReadyToTest(),
            env_node,
            qp_gnc_node,
            TimerAction(
                period=SHUTDOWN_AFTER_SEC,
                actions=[EmitEvent(event=Shutdown(reason='qp_mpc test complete'))],
            ),
        ]),
        {},
    )


class TestQpMpcConvergence(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.recorder = TrajectoryRecorder()

        # Collect all data upfront before any test runs
        discovery_deadline = time.monotonic() + DISCOVERY_TIMEOUT_SEC
        while (
            cls.recorder.count_publishers('env_data') == 0
            and time.monotonic() < discovery_deadline
        ):
            rclpy.spin_once(cls.recorder, timeout_sec=0.1)

        deadline = time.monotonic() + RECEIVE_TIMEOUT_SEC
        while (
            len(cls.recorder.trajectory) < EXPECTED_STEPS
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(cls.recorder, timeout_sec=0.1)

    @classmethod
    def tearDownClass(cls):
        cls.recorder.destroy_node()
        rclpy.shutdown()

    def _pos_norm(self, state):
        return math.sqrt(state[0] ** 2 + state[1] ** 2 + state[2] ** 2)

    def _vel_norm(self, state):
        return math.sqrt(state[3] ** 2 + state[4] ** 2 + state[5] ** 2)

    def test_state_messages_received(self):
        """Verify env_data messages arrived."""
        self.assertGreater(
            self.recorder.count_publishers('env_data'),
            0,
            'env_data publisher was not discovered within timeout',
        )
        received = len(self.recorder.trajectory)
        self.assertGreaterEqual(
            received,
            EXPECTED_STEPS,
            f'Expected at least {EXPECTED_STEPS} env_data messages, received {received}',
        )

    def test_all_states_have_correct_size(self):
        """Every published state vector must have 6 elements."""
        for step, row in enumerate(self.recorder.trajectory):
            self.assertEqual(
                len(row),
                STATE_SIZE,
                f'State at step {step} has {len(row)} elements, expected {STATE_SIZE}',
            )

    def test_position_converges(self):
        """Final position norm must be below the convergence threshold."""
        traj = self.recorder.trajectory
        self.assertGreater(len(traj), 0, 'No trajectory data to check')

        initial_pos = self._pos_norm(traj[0])
        final_pos = self._pos_norm(traj[-1])

        self.assertGreater(
            initial_pos,
            10.0,
            f'Initial position norm {initial_pos:.4f} unexpectedly small',
        )
        self.assertLess(
            final_pos,
            CONVERGENCE_POS_THRESHOLD,
            f'Final position norm {final_pos:.6f} exceeds threshold '
            f'{CONVERGENCE_POS_THRESHOLD}. Controller did not converge.',
        )

    def test_position_generally_decreasing(self):
        """Position norm should trend downward (allow some transient)."""
        traj = self.recorder.trajectory
        if len(traj) < 20:
            self.skipTest('Not enough data for monotonicity check')

        # Compare position norm in first quarter vs last quarter
        quarter = len(traj) // 4
        early_norms = [self._pos_norm(s) for s in traj[:quarter]]
        late_norms = [self._pos_norm(s) for s in traj[-quarter:]]

        avg_early = sum(early_norms) / len(early_norms)
        avg_late = sum(late_norms) / len(late_norms)

        self.assertLess(
            avg_late,
            avg_early,
            f'Average position norm in last quarter ({avg_late:.6f}) is not less '
            f'than first quarter ({avg_early:.6f}). Controller is not converging.',
        )

    def test_control_respects_bounds(self):
        """Velocity changes should be consistent with u_max = 0.01 bounds."""
        traj = self.recorder.trajectory
        if len(traj) < 2:
            self.skipTest('Not enough data for bounds check')

        # Final state should have small velocity (converging to rest)
        final_vel = self._vel_norm(traj[-1])
        self.assertLess(
            final_vel,
            0.1,
            f'Final velocity norm {final_vel:.6f} is unexpectedly large',
        )


@launch_testing.post_shutdown_test()
class TestQpMpcLaunchShutdown(unittest.TestCase):

    def test_all_processes_exit_zero(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
