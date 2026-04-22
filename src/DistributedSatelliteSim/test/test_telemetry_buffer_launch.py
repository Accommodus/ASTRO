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

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import pytest
import rclpy
from rclpy.node import Node

from distributed_satellite_sim.srv import GetRecentActuationHistory
from distributed_satellite_sim.srv import GetRecentEnvHistory


@pytest.mark.launch_test
def generate_test_description():
    env_node = launch_ros.actions.Node(
        package='distributed_satellite_sim',
        executable='env_node',
        name='env_node',
        output='screen',
        parameters=[{
            'max_steps': 20,
            # buffer_node also subscribes to env_data, so require both
            # gnc_node and buffer_node before the startup gate opens
            'min_subscribers': 2,
        }],
    )

    gnc_node = launch_ros.actions.Node(
        package='distributed_satellite_sim',
        executable='gnc_node',
        name='gnc_node',
        output='screen',
    )

    buffer_node = launch_ros.actions.Node(
        package='distributed_satellite_sim',
        executable='telemetry_buffer_node',
        name='telemetry_buffer',
        output='screen',
        parameters=[{
            'env_buffer_capacity': 50,
            'actuation_buffer_capacity': 50,
            'log_buffer_capacity': 50,
        }],
    )

    return (
        launch.LaunchDescription([
            env_node,
            gnc_node,
            buffer_node,
            launch_testing.actions.ReadyToTest(),
        ]),
        {'env_node': env_node, 'gnc_node': gnc_node, 'buffer_node': buffer_node},
    )


class TestTelemetryBufferNodes(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_telemetry_buffer_client')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _call_service(self, client, request, timeout=5.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
        return future.result() if future.done() else None

    def test_env_history_queryable(self):
        client = self.node.create_client(
            GetRecentEnvHistory,
            '/telemetry_buffer/get_recent_env_history')
        try:
            self.assertTrue(
                client.wait_for_service(timeout_sec=10.0),
                'get_recent_env_history service not available')

            # Allow simulation steps to accumulate in the buffer
            time.sleep(3.0)

            request = GetRecentEnvHistory.Request()
            request.limit = 0
            response = self._call_service(client, request)

            self.assertIsNotNone(response, 'Service call did not complete')
            self.assertGreater(
                len(response.entries), 0,
                'Expected at least one env history entry')
            self.assertEqual(response.capacity, 50)
            self.assertEqual(response.total_buffered, len(response.entries))
            for entry in response.entries:
                self.assertEqual(len(entry.state), 6)
        finally:
            self.node.destroy_client(client)

    def test_actuation_history_queryable(self):
        client = self.node.create_client(
            GetRecentActuationHistory,
            '/telemetry_buffer/get_recent_actuation_history')
        try:
            self.assertTrue(
                client.wait_for_service(timeout_sec=10.0),
                'get_recent_actuation_history service not available')

            # Allow actuation_applied messages to accumulate in the buffer
            time.sleep(3.0)

            request = GetRecentActuationHistory.Request()
            request.limit = 0
            response = self._call_service(client, request)

            self.assertIsNotNone(response, 'Service call did not complete')
            self.assertGreater(
                len(response.entries), 0,
                'Expected at least one actuation history entry')
            self.assertEqual(response.capacity, 50)
            self.assertEqual(response.total_buffered, len(response.entries))
            for entry in response.entries:
                self.assertEqual(len(entry.actuation), 3)
        finally:
            self.node.destroy_client(client)

    def test_env_history_limit_parameter(self):
        client = self.node.create_client(
            GetRecentEnvHistory,
            '/telemetry_buffer/get_recent_env_history')
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=10.0))

            request = GetRecentEnvHistory.Request()
            request.limit = 2
            response = self._call_service(client, request)

            self.assertIsNotNone(response)
            self.assertLessEqual(len(response.entries), 2)
        finally:
            self.node.destroy_client(client)


@launch_testing.post_shutdown_test()
class TestTelemetryBufferShutdown(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)
