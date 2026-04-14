import time
import unittest
from unittest.mock import patch

import rclpy
from distributed_satellite_sim.srv import ActuationCmd
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from external_sim_bridge.bridge_node import ExternalSimBridgeNode
from external_sim_bridge.config import BridgeConfig


class ExplodingBackend:
    def initialize(self) -> None:
        return None

    def validate_control(self, control) -> None:
        return None

    def apply_control(self, control) -> None:
        return None

    def advance(self):
        raise RuntimeError('boom')


class WrongStateBackend:
    def initialize(self) -> None:
        return None

    def validate_control(self, control) -> None:
        return None

    def apply_control(self, control) -> None:
        return None

    def advance(self):
        return [1.0, 2.0]


class RejectingControlBackend:
    def initialize(self) -> None:
        return None

    def validate_control(self, control) -> None:
        raise ValueError('backend rejected control')

    def apply_control(self, control) -> None:
        return None

    def advance(self):
        return [0.0] * 6


class BridgeNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.helper_node = Node('external_sim_bridge_test_helper')
        self.executor = SingleThreadedExecutor()
        self.received_messages = []
        self.subscription = self.helper_node.create_subscription(
            Float64MultiArray,
            'env_data',
            self.received_messages.append,
            10,
        )
        self.client = self.helper_node.create_client(ActuationCmd, 'actuation_cmd')
        self.bridge = None

    def tearDown(self):
        if self.bridge is not None:
            self.executor.remove_node(self.bridge)
            self.bridge.destroy_node()
            self.bridge = None
        self.executor.remove_node(self.helper_node)
        self.executor.shutdown()
        self.helper_node.destroy_node()

    def _make_bridge(self, *, backend, config: BridgeConfig):
        with (
            patch('external_sim_bridge.bridge_node.load_bridge_config', return_value=config),
            patch('external_sim_bridge.bridge_node.create_backend', return_value=backend),
        ):
            self.bridge = ExternalSimBridgeNode()
        self.executor.add_node(self.bridge)
        self.executor.add_node(self.helper_node)

    def _spin_for(self, seconds: float):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)

    def _call_actuation(self, thrust):
        self.assertTrue(self.client.wait_for_service(timeout_sec=2.0))
        request = ActuationCmd.Request()
        request.thrust = list(thrust)
        future = self.client.call_async(request)
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.05)
        self.assertTrue(future.done(), 'actuation_cmd service call did not complete')
        return future.result()

    def test_bridge_waits_for_min_subscribers_before_publishing(self):
        self._make_bridge(
            backend=WrongStateBackend(),
            config=BridgeConfig(backend_type='fake', max_steps=91, min_subscribers=2),
        )

        self._spin_for(0.35)

        self.assertEqual(len(self.received_messages), 0)
        self.assertIsNone(self.bridge._runtime_failure)

    def test_bridge_returns_failure_when_backend_rejects_control(self):
        self._make_bridge(
            backend=RejectingControlBackend(),
            config=BridgeConfig(backend_type='fake', max_steps=91, min_subscribers=0),
        )

        response = self._call_actuation([1.0, 2.0, 3.0])

        self.assertFalse(response.success)

    def test_bridge_stops_timer_on_backend_runtime_failure(self):
        with patch('external_sim_bridge.bridge_node.rclpy.shutdown', return_value=None):
            self._make_bridge(
                backend=ExplodingBackend(),
                config=BridgeConfig(backend_type='fake', max_steps=91, min_subscribers=0),
            )

            self._spin_for(0.25)

        self.assertIsNotNone(self.bridge._runtime_failure)
        self.assertTrue(self.bridge._timer.is_canceled())

    def test_bridge_rejects_wrong_state_dimension(self):
        with patch('external_sim_bridge.bridge_node.rclpy.shutdown', return_value=None):
            self._make_bridge(
                backend=WrongStateBackend(),
                config=BridgeConfig(backend_type='fake', max_steps=91, min_subscribers=0),
            )

            self._spin_for(0.25)

        self.assertIsNotNone(self.bridge._runtime_failure)
        self.assertTrue(self.bridge._timer.is_canceled())
