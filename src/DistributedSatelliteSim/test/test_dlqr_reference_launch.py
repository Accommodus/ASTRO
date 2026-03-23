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

import csv
import os
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import EmitEvent
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray

FIXTURE_PATH = os.path.join(
    os.path.dirname(__file__),
    'data',
    'dlqr_reference_trajectory.csv',
)
EXPECTED_STEPS = 91
STATE_SIZE = 6
ABS_TOLERANCE = 1e-4
SHUTDOWN_AFTER_SEC = 30.0
RECEIVE_TIMEOUT_SEC = 25.0
DISCOVERY_TIMEOUT_SEC = 10.0


def _load_reference_trajectory():
    with open(FIXTURE_PATH, newline='', encoding='utf-8') as csv_file:
        rows = [[float(value) for value in row] for row in csv.reader(csv_file)]

    if len(rows) != EXPECTED_STEPS:
        raise AssertionError(
            f'Reference fixture must contain {EXPECTED_STEPS} rows, found {len(rows)}'
        )

    for step, row in enumerate(rows):
        if len(row) != STATE_SIZE:
            raise AssertionError(
                f'Reference fixture row {step} must contain {STATE_SIZE} values, '
                f'found {len(row)}'
            )

    return rows


class TrajectoryRecorder(Node):

    def __init__(self):
        super().__init__('dlqr_reference_trajectory_recorder')
        self.trajectory = []
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'env_data',
            self._callback,
            QoSProfile(depth=100),
        )

    def _callback(self, msg):
        self.trajectory.append(list(msg.data))


def generate_test_description():
    package_share = get_package_share_directory('distributed_satellite_sim')
    launch_path = os.path.join(package_share, 'launch', 'sim.launch.py')

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        launch_arguments={
            'max_steps': str(EXPECTED_STEPS),
            'min_subscribers': '2',
        }.items(),
    )

    return (
        launch.LaunchDescription([
            launch_testing.actions.ReadyToTest(),
            sim_launch,
            TimerAction(
                period=SHUTDOWN_AFTER_SEC,
                actions=[EmitEvent(event=Shutdown(reason='trajectory capture complete'))],
            ),
        ]),
        {},
    )


class TestDlqrReferenceTrajectory(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.recorder = TrajectoryRecorder()

    @classmethod
    def tearDownClass(cls):
        cls.recorder.destroy_node()
        rclpy.shutdown()

    def test_ros_trajectory_matches_reference(self):
        # Provenance: the fixture represents the 91-step closed-loop trajectory from the
        # original standalone DLQR pair in reference/DLQR/udp_hcw_discrete_txrx 2 1.cpp
        # and reference/DLQR/udp_roundtrip_discrete.cpp using the default initial state.
        reference_trajectory = _load_reference_trajectory()

        discovery_deadline = time.monotonic() + DISCOVERY_TIMEOUT_SEC
        while (
            self.recorder.count_publishers('env_data') == 0
            and time.monotonic() < discovery_deadline
        ):
            rclpy.spin_once(self.recorder, timeout_sec=0.1)

        self.assertGreater(
            self.recorder.count_publishers('env_data'),
            0,
            'env_data publisher was not discovered within timeout',
        )

        deadline = time.monotonic() + RECEIVE_TIMEOUT_SEC
        while (
            len(self.recorder.trajectory) < EXPECTED_STEPS
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self.recorder, timeout_sec=0.1)

        received = len(self.recorder.trajectory)
        self.assertEqual(
            received,
            EXPECTED_STEPS,
            f'Expected {EXPECTED_STEPS} env_data messages, received {received}',
        )

        for step, (actual_row, expected_row) in enumerate(
            zip(self.recorder.trajectory, reference_trajectory)
        ):
            self.assertEqual(
                len(actual_row),
                STATE_SIZE,
                f'Trajectory row {step} should contain {STATE_SIZE} state values',
            )
            for index, (actual_value, expected_value) in enumerate(
                zip(actual_row, expected_row)
            ):
                difference = abs(actual_value - expected_value)
                self.assertLessEqual(
                    difference,
                    ABS_TOLERANCE,
                    f'Trajectory mismatch at step '
                    f'{step}, state[{index}]: actual={actual_value:.15f}, '
                    f'expected={expected_value:.15f}, abs_diff={difference:.15f}, '
                    f'tolerance={ABS_TOLERANCE}. '
                    f'Tolerance accounts for text-formatted reference output and '
                    f'cross-platform floating-point serialization differences.',
                )


@launch_testing.post_shutdown_test()
class TestDlqrLaunchShutdown(unittest.TestCase):

    def test_all_processes_exit_zero(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
