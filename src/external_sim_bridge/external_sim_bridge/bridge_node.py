from __future__ import annotations

import math
from typing import Sequence

from distributed_satellite_sim.srv import ActuationCmd
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from external_sim_bridge.backends import create_backend
from external_sim_bridge.backends.base import BackendValidationError, ExternalSimBackend
from external_sim_bridge.config import BridgeConfig, load_bridge_config


class ExternalSimBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('external_sim_bridge')

        try:
            self._config = load_bridge_config(self)
            self._backend = self._create_backend(self._config)
        except Exception as error:
            self.get_logger().error(f'Bridge startup failed: {error}')
            raise

        self._latest_command = [0.0, 0.0, 0.0]
        self._step = 0
        self._sim_started = False
        self._runtime_failure: Exception | None = None

        self._state_pub = self.create_publisher(Float64MultiArray, 'env_data', 10)
        self._cmd_srv = self.create_service(
            ActuationCmd,
            'actuation_cmd',
            self._handle_actuation_command,
        )
        self._timer = self.create_timer(
            self._config.timer_period_sec,
            self._handle_timer_tick,
        )

        self.get_logger().info(
            'External simulator bridge started '
            f'(backend_type={self._config.backend_type}, max_steps={self._config.max_steps}, '
            f'min_subscribers={self._config.min_subscribers})'
        )

    def _create_backend(self, config: BridgeConfig) -> ExternalSimBackend:
        backend = create_backend(config.backend_type)
        backend.initialize()
        return backend

    def _handle_actuation_command(
        self,
        request: ActuationCmd.Request,
        response: ActuationCmd.Response,
    ) -> ActuationCmd.Response:
        try:
            control = self._validate_vector(
                values=request.thrust,
                expected_length=3,
                vector_name='control',
            )
            self._backend.validate_control(control)
        except (BackendValidationError, ValueError) as error:
            self.get_logger().error(f'Invalid actuation command: {error}')
            response.success = False
            return response

        self._latest_command = control
        response.success = True
        self.get_logger().info(
            'actuation_cmd received: u = [%.6e, %.6e, %.6e]'
            % tuple(self._latest_command)
        )
        return response

    def _handle_timer_tick(self) -> None:
        if not self._sim_started:
            if self._state_pub.get_subscription_count() < self._config.min_subscribers:
                return
            self._sim_started = True

        if self._config.max_steps > 0 and self._step >= self._config.max_steps:
            self._timer.cancel()
            self.get_logger().info(f'Simulation complete after {self._step} steps')
            return

        try:
            self._backend.apply_control(self._latest_command)
            state = self._validate_vector(
                values=self._backend.advance(),
                expected_length=6,
                vector_name='state',
            )
        except Exception as error:
            self.get_logger().error(f'Backend runtime failure on step {self._step}: {error}')
            self._timer.cancel()
            self._runtime_failure = RuntimeError(
                f'Backend runtime failure on step {self._step}: {error}'
            )
            rclpy.shutdown()
            return

        self._state_pub.publish(Float64MultiArray(data=state))
        self.get_logger().info(
            'step %d: x = [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]'
            % (self._step, *state)
        )
        self._step += 1

    def _validate_vector(
        self,
        values: Sequence[float],
        expected_length: int,
        vector_name: str,
    ) -> list[float]:
        vector = list(values)
        if len(vector) != expected_length:
            raise ValueError(
                f'{vector_name} vector must have length {expected_length}, received {len(vector)}'
            )

        validated = []
        for index, value in enumerate(vector):
            numeric_value = float(value)
            if not math.isfinite(numeric_value):
                raise ValueError(
                    f'{vector_name}[{index}] must be finite, received {numeric_value!r}'
                )
            validated.append(numeric_value)

        return validated


def spin_bridge() -> None:
    bridge = ExternalSimBridgeNode()
    runtime_failure = None
    try:
        rclpy.spin(bridge)
    finally:
        runtime_failure = bridge._runtime_failure
        bridge.destroy_node()

    if runtime_failure is not None:
        raise runtime_failure
