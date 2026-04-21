#!/usr/bin/env python3

import math
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
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray


EXPECT_BASILISK = os.environ.get('ASTRO_EXPECT_BASILISK') == '1'

try:
    import Basilisk  # noqa: F401
except ModuleNotFoundError as error:
    BASILISK_AVAILABLE = False
    BASILISK_IMPORT_ERROR = error
else:
    BASILISK_AVAILABLE = True
    BASILISK_IMPORT_ERROR = None


if EXPECT_BASILISK and not BASILISK_AVAILABLE:
    raise AssertionError(
        'ASTRO_EXPECT_BASILISK=1 but the Basilisk runtime is not importable: '
        f'{BASILISK_IMPORT_ERROR}'
    )


EXPECTED_STEPS = 5
STATE_SIZE = 6
SHUTDOWN_AFTER_SEC = 15.0
RECEIVE_TIMEOUT_SEC = 12.0
DISCOVERY_TIMEOUT_SEC = 10.0


class BasiliskTrajectoryRecorder(Node):
    def __init__(self):
        super().__init__('basilisk_bridge_trajectory_recorder')
        self.trajectory = []
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'env_data',
            self._callback,
            QoSProfile(depth=100),
        )

    def _callback(self, msg):
        self.trajectory.append(list(msg.data))


if BASILISK_AVAILABLE:

    @pytest.mark.rostest
    def generate_test_description():
        package_share = get_package_share_directory('external_sim_bridge')
        launch_path = os.path.join(package_share, 'launch', 'bridge_sim.launch.py')

        bridge_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
            launch_arguments={
                'backend_type': 'basilisk',
                'max_steps': str(EXPECTED_STEPS),
                'min_subscribers': '2',
            }.items(),
        )

        return (
            launch.LaunchDescription([
                launch_testing.actions.ReadyToTest(),
                bridge_launch,
                TimerAction(
                    period=SHUTDOWN_AFTER_SEC,
                    actions=[EmitEvent(event=Shutdown(reason='basilisk bridge validation complete'))],
                ),
            ]),
            {},
        )


    class TestBasiliskBridgeLaunch(unittest.TestCase):
        @classmethod
        def setUpClass(cls):
            if not rclpy.ok():
                rclpy.init()
            cls.recorder = BasiliskTrajectoryRecorder()

        @classmethod
        def tearDownClass(cls):
            cls.recorder.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

        def test_basilisk_bridge_publishes_finite_state_trajectory(self):
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

            saw_motion = False
            previous_state = None
            for step, state in enumerate(self.recorder.trajectory):
                self.assertEqual(
                    len(state),
                    STATE_SIZE,
                    f'Trajectory row {step} should contain {STATE_SIZE} state values',
                )
                for index, value in enumerate(state):
                    self.assertTrue(
                        math.isfinite(float(value)),
                        f'Trajectory row {step} state[{index}] must be finite',
                    )

                if previous_state is not None and any(
                    abs(current - previous) > 1e-9
                    for current, previous in zip(state, previous_state)
                ):
                    saw_motion = True

                previous_state = state

            self.assertTrue(saw_motion, 'Basilisk launch trajectory did not change between steps')


    @launch_testing.post_shutdown_test()
    class TestBasiliskBridgeShutdown(unittest.TestCase):
        def test_all_processes_exit_zero(self, proc_info):
            launch_testing.asserts.assertExitCodes(proc_info)
