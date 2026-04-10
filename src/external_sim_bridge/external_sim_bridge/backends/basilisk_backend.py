from __future__ import annotations

from typing import Any, Sequence

from external_sim_bridge.backends.base import BackendValidationError


class BasiliskBackend:
    """Basilisk backend for a fixed translational validation scenario.

    Provenance assumptions:
    - designed against Basilisk's documented Python APIs for `SimulationBaseClass`,
      `spacecraft.Spacecraft`, `thrusterDynamicEffector`, `simIncludeThruster`,
      `SCStatesMsgPayload`, and `THRArrayOnTimeCmdMsgPayload`
    - the validation scenario is an internal bridge-only configuration named
      `translational_3dof_axis_thrusters`

    Scenario assumptions:
    - telemetry uses `scStateOutMsg.r_BN_N` and `scStateOutMsg.v_BN_N`
    - the exported ROS state is `[x, y, z, vx, vy, vz]` in the same inertial frame
    - the spacecraft attitude is initialized to zero and no torque is introduced, so
      body-frame thruster axes stay aligned with the inertial axes for this scenario
    - translational actuation is modeled with six center-of-mass thrusters:
      `+x`, `-x`, `+y`, `-y`, `+z`, `-z`
    - each ROS thrust component maps to the matching Basilisk axis pair using
      one-to-one axis ordering and a fixed force-to-on-time conversion over the
      bridge's 100 ms step

    Known limitation:
    - this scenario is a bridge-validation path, not an HCW-equivalent Basilisk model
    """

    _STEP_PERIOD_SEC = 0.1
    _TASK_NAME = 'basilisk_bridge_task'
    _PROCESS_NAME = 'basilisk_bridge_process'
    _SPACECRAFT_TAG = 'basilisk_bridge_spacecraft'
    _THRUSTER_TAG = 'basilisk_bridge_thrusters'
    _AXIS_THRUSTER_MAX_THRUST_N = 1.0
    _INITIAL_POSITION_M = [20.0, 20.0, 20.0]
    _INITIAL_VELOCITY_MPS = [0.00930458, -0.0467472, 0.00798343]
    _THRUSTER_LAYOUT = (
        ('+x', [1.0, 0.0, 0.0]),
        ('-x', [-1.0, 0.0, 0.0]),
        ('+y', [0.0, 1.0, 0.0]),
        ('-y', [0.0, -1.0, 0.0]),
        ('+z', [0.0, 0.0, 1.0]),
        ('-z', [0.0, 0.0, -1.0]),
    )

    def __init__(self) -> None:
        self._control = [0.0, 0.0, 0.0]
        self._current_stop_time_ns = 0
        self._step_period_ns: int | None = None
        self._messaging: Any = None
        self._sim: Any = None
        self._macros: Any = None
        self._spacecraft: Any = None
        self._thruster_effector: Any = None
        self._thruster_command_msg: Any = None

    def initialize(self) -> None:
        modules = self._import_basilisk_modules()
        self._messaging = modules['messaging']
        self._macros = modules['macros']
        self._sim = modules['SimulationBaseClass'].SimBaseClass()
        self._step_period_ns = self._macros.sec2nano(self._STEP_PERIOD_SEC)

        dynamics_process = self._sim.CreateNewProcess(self._PROCESS_NAME)
        dynamics_process.addTask(
            self._sim.CreateNewTask(self._TASK_NAME, self._step_period_ns)
        )

        self._spacecraft = self._create_spacecraft(modules['spacecraft'])
        self._thruster_effector = self._create_thrusters(
            sim_include_thruster=modules['simIncludeThruster'],
            thruster_dynamic_effector=modules['thrusterDynamicEffector'],
        )

        self._sim.AddModelToTask(self._TASK_NAME, self._thruster_effector)
        self._sim.AddModelToTask(self._TASK_NAME, self._spacecraft)

        zero_cmd = self._build_thruster_payload([0.0] * len(self._THRUSTER_LAYOUT))
        self._thruster_command_msg = self._messaging.THRArrayOnTimeCmdMsg().write(zero_cmd)
        self._thruster_effector.cmdsInMsg.subscribeTo(self._thruster_command_msg)

        self._sim.InitializeSimulation()

    def validate_control(self, control: Sequence[float]) -> None:
        if len(control) != 3:
            raise BackendValidationError(
                f'Basilisk backend expected a 3-element control vector, received {len(control)}'
            )

        for axis_name, value in zip(('x', 'y', 'z'), control):
            if abs(float(value)) > self._AXIS_THRUSTER_MAX_THRUST_N:
                raise BackendValidationError(
                    f'Basilisk {axis_name}-axis command {value} exceeds the configured '
                    f'per-axis thrust limit of {self._AXIS_THRUSTER_MAX_THRUST_N} N'
                )

    def apply_control(self, control: Sequence[float]) -> None:
        self.validate_control(control)
        self._control = [float(value) for value in control]
        self._thruster_command_msg.write(self._build_thruster_payload(self._control_to_on_times()))

    def advance(self) -> list[float]:
        if self._step_period_ns is None:
            raise RuntimeError('Basilisk backend was not initialized before advance()')

        self._current_stop_time_ns += self._step_period_ns
        self._sim.ConfigureStopTime(self._current_stop_time_ns)
        self._sim.ExecuteSimulation()

        state = self._spacecraft.scStateOutMsg.read()
        return [
            float(state.r_BN_N[0]),
            float(state.r_BN_N[1]),
            float(state.r_BN_N[2]),
            float(state.v_BN_N[0]),
            float(state.v_BN_N[1]),
            float(state.v_BN_N[2]),
        ]

    def _import_basilisk_modules(self) -> dict[str, Any]:
        try:
            from Basilisk.architecture import messaging
            from Basilisk.simulation import spacecraft, thrusterDynamicEffector
            from Basilisk.utilities import SimulationBaseClass, macros, simIncludeThruster
        except ModuleNotFoundError as error:
            raise ImportError(
                'Basilisk backend selected, but the Basilisk Python package is not installed '
                'or is missing required modules'
            ) from error

        return {
            'SimulationBaseClass': SimulationBaseClass,
            'macros': macros,
            'messaging': messaging,
            'simIncludeThruster': simIncludeThruster,
            'spacecraft': spacecraft,
            'thrusterDynamicEffector': thrusterDynamicEffector,
        }

    def _create_spacecraft(self, spacecraft_module: Any) -> Any:
        spacecraft_object = spacecraft_module.Spacecraft()
        spacecraft_object.ModelTag = self._SPACECRAFT_TAG
        spacecraft_object.hub.mHub = 1.0
        spacecraft_object.hub.r_BcB_B = [0.0, 0.0, 0.0]
        spacecraft_object.hub.IHubPntBc_B = [
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
        spacecraft_object.hub.r_CN_NInit = self._INITIAL_POSITION_M
        spacecraft_object.hub.v_CN_NInit = self._INITIAL_VELOCITY_MPS
        spacecraft_object.hub.sigma_BNInit = [0.0, 0.0, 0.0]
        spacecraft_object.hub.omega_BN_BInit = [0.0, 0.0, 0.0]
        return spacecraft_object

    def _create_thrusters(
        self,
        sim_include_thruster: Any,
        thruster_dynamic_effector: Any,
    ) -> Any:
        thruster_effector = thruster_dynamic_effector.ThrusterDynamicEffector()
        thruster_factory = sim_include_thruster.thrusterFactory()

        for label, axis in self._THRUSTER_LAYOUT:
            thruster_factory.create(
                'Blank_Thruster',
                [0.0, 0.0, 0.0],
                axis,
                label=label,
                MaxThrust=float(self._AXIS_THRUSTER_MAX_THRUST_N),
                useMinPulseTime=False,
            )

        thruster_factory.addToSpacecraft(
            self._THRUSTER_TAG,
            thruster_effector,
            self._spacecraft,
        )
        return thruster_effector

    def _control_to_on_times(self) -> list[float]:
        x_force, y_force, z_force = self._control
        scale = self._STEP_PERIOD_SEC / self._AXIS_THRUSTER_MAX_THRUST_N
        return [
            max(x_force, 0.0) * scale,
            max(-x_force, 0.0) * scale,
            max(y_force, 0.0) * scale,
            max(-y_force, 0.0) * scale,
            max(z_force, 0.0) * scale,
            max(-z_force, 0.0) * scale,
        ]

    def _build_thruster_payload(self, on_times: Sequence[float]) -> Any:
        payload = self._messaging.THRArrayOnTimeCmdMsgPayload()
        for index, on_time in enumerate(on_times):
            payload.OnTimeRequest[index] = float(on_time)
        return payload
