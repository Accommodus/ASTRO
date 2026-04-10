# External Simulator Bridge Design Note

## Purpose

This note defines the v1 contract for ASTRO's external simulator bridge under GH issue `#24`.
The goal is to validate the manuscript branch's external deployment pattern against Basilisk
without changing ASTRO's current ROS-facing control boundary.

For v1, the bridge replaces `env_node` in the closed loop while leaving
`gnc_node` unchanged.

## Current `main` Baseline

The implementation baseline is the current `main` branch package at
`src/DistributedSatelliteSim`.

The behavior to preserve is defined primarily by:

- `src/DistributedSatelliteSim/include/distributed_satellite_sim/env_node.hpp`
- `src/DistributedSatelliteSim/include/distributed_satellite_sim/gnc_node.hpp`
- `src/DistributedSatelliteSim/launch/sim.launch.py`
- `src/DistributedSatelliteSim/test/test_dlqr_reference_launch.py`

Key baseline facts:

- `EnvNode` advances on a `100 ms` wall timer.
- `EnvNode` publishes `env_data` from `timer_callback()`.
- `EnvNode` stores the latest thrust command in `u_now_` when `actuation_cmd` is called.
- `EnvNode` does not use service-call-driven lock-step stepping.
- `GncNode` computes control from `env_data` and sends thrust using `async_send_request()`.
- `EnvNode` defaults `max_steps` to `91` and `min_subscribers` to `0`.
- `sim.launch.py` currently overrides `min_subscribers` to `1`.
- `test_dlqr_reference_launch.py` overrides `min_subscribers` to `2` so the recorder and
  `gnc_node` are both present before the run starts.

## V1 ROS-Facing Contract

The bridge must preserve the existing ROS contract for the current DLQR path.

### Telemetry boundary

- Topic name: `env_data`
- Message type: `std_msgs/msg/Float64MultiArray`
- Required ordering: `[x, y, z, vx, vy, vz]`
- Required length: exactly `6`

### Control boundary

- Service name: `actuation_cmd`
- Service type: `distributed_satellite_sim/srv/ActuationCmd`
- Request shape: `float64[3] thrust`
- Response shape: `bool success`

### Timing and lifecycle contract

- The bridge runs a `100 ms` wall timer, matching `EnvNode`.
- The bridge uses latest-command-wins semantics, matching `EnvNode`.
- The service callback stores the newest `thrust[3]` for use on the next timer tick.
- The timer tick advances the backend and publishes the next `env_data` sample.
- The bridge respects `min_subscribers` before the first advance or publish.
- The bridge defaults `max_steps` to `91` and `min_subscribers` to `0` to mirror
  `EnvNode` exactly. Launch files may override these defaults for test or operator needs.

## Role Split

The bridge package should be split into simulator-agnostic bridge logic and simulator-specific
backend drivers.

### Simulator-agnostic bridge logic

The generic bridge node owns:

- ROS publisher creation for `env_data`
- ROS service creation for `actuation_cmd`
- parameter parsing
- timer scheduling
- startup gating through `min_subscribers`
- step counting and `max_steps` handling
- dimension validation for published state and received control vectors
- error handling and ROS logging

This logic should not depend on Basilisk-specific types, paths, coordinate conventions,
or scenario setup.

### Simulator-specific backend drivers

A backend driver owns:

- simulator import and initialization
- simulator-native state reads
- translation from simulator-native telemetry into ASTRO's six-state ROS vector
- translation from ASTRO's `thrust[3]` command into simulator-native actuation inputs
- simulator-native timing or stepping calls
- simulator-specific runtime requirements and provenance

To avoid overloading terms from the manuscript diagrams, the whole ROS-facing component is the
"bridge" or "adapter." Basilisk-specific code should be called a "backend" or
"backend driver."

## Python and C++ Interop

The bridge is intended to be implemented in Python using `rclpy`, while `gnc_node` remains in
C++ using `rclcpp`.

This is an intentional design choice:

- Basilisk integration is expected to be easier from Python.
- ROS 2 topic and service interoperability is language-agnostic at the DDS boundary.
- `gnc_node` can remain unchanged and continue using the existing `ActuationCmd` service.

The `external_sim_bridge` package should declare a direct dependency on
`distributed_satellite_sim` so it can import and serve `ActuationCmd`.

## Deterministic Regression Backend

Before Basilisk is introduced, the bridge should ship with a deterministic linear-dynamics
backend that mirrors the current `EnvNode` implementation exactly.

That backend should use:

- the same initial state
- the same `Ad` matrix
- the same `Bd` matrix
- the same `100 ms` timer cadence at the bridge-node layer
- the same latest-command-wins update semantics

This backend is not a toy stub. It is the bridge-core regression target that allows:

- closed-loop testing with the existing `gnc_node`
- comparison against the existing DLQR reference fixture in
  `src/DistributedSatelliteSim/test/data/dlqr_reference_trajectory.csv`
- isolation of bridge-architecture bugs before simulator-specific Basilisk work begins

## Failure Handling

Failure behavior should distinguish between service-path validation problems and runtime
backend failures.

### Service-path validation

If the bridge receives malformed control input or the backend rejects an actuation command
during service-path validation:

- the bridge should log a clear error
- the bridge should return `success = false`

### Timer-path runtime failure

If the backend throws or fails while advancing on the timer tick:

- the bridge should log the failure clearly
- the failure should be treated as a backend/runtime error, not as a service failure
- the timer should stop so the node fails in a defined and visible way

## Manuscript Alignment

Issue `#24` validates the manuscript branch's external deployment pattern, but v1 does not
implement the richer ROS-side decomposition shown in the manuscript's external architecture
figure.

For v1:

- the bridge validates the external deployment mode
- the ROS-facing contract stays the same as the current `env_node` to `gnc_node` boundary
- the existing monolithic `gnc_node` remains intact

This keeps `#24` separate from issue `#6`, which is the broader controller and interface
generalization track.

## Basilisk Paper Comparison

The current Basilisk backend matches the manuscript and Basilisk/ROS bridge literature at the
high-level adapter boundary only:

- Basilisk remains behind a ROS adapter instead of leaking simulator-native messages into the ROS
  graph
- `gnc_node` remains unchanged and still consumes `env_data` and calls `actuation_cmd`
- simulator-specific setup, message translation, and timing live inside the Basilisk backend

For v1, the implementation intentionally does not reproduce a richer paper-style decomposition of
navigation, guidance, and control modules on the ROS side. The bridge validates the external
deployment pattern first while preserving ASTRO's current ROS contract.

## Correction to Presentation Wording

Some presentation text describes `gnc_node` as if it "serves" `actuation_cmd`.
That is not how the code behaves on `main`.

The correct client/server roles are:

- `gnc_node` is the `actuation_cmd` client
- `env_node`, and later the bridge, is the `actuation_cmd` server

## Explicit Non-Goals for `#24`

The following are out of scope for v1 bridge validation:

- changing `env_data` or `ActuationCmd` schemas
- introducing a generalized telemetry message format
- implementing the manuscript's full Sensor/Nav/Guidance/Control decomposition
- universal controller selection or interface redesign under issue `#6`
- circular-buffer or operator tooling work under issue `#25`
- showcase or presentation packaging work under issue `#26`

If Basilisk reveals that the current ROS boundary is too narrow, the bridge work should record
that limitation and open follow-on interface issues rather than widening issue `#24`.

## Basilisk Assumptions Template

The Basilisk backend implementation should record the following in code comments and user-facing
documentation:

- Basilisk version or source provenance
- scenario name and entrypoint used for validation
- exact telemetry fields used to build `[x, y, z, vx, vy, vz]`
- frame and axis conventions assumed during mapping
- timing model used between bridge timer ticks and Basilisk stepping
- actuation interface used for translational thrust
- additional runtime dependencies or environment setup requirements
- known gaps, approximations, or validation blockers

## Current Basilisk Backend Provenance

The current `basilisk` backend is implemented against Basilisk's published Python API
documentation, because Basilisk is not installed in the active development environment used for
this slice.

Documentation used while implementing the backend:

- `SimulationBaseClass` API from Basilisk `2.4.0`
- `thrusterDynamicEffector` API from Basilisk `2.4.0`
- `simIncludeThruster` Python helper from Basilisk `2.4.0`
- `SCStatesMsgPayload` message fields from Basilisk `2.4.0`
- `THRArrayOnTimeCmdMsgPayload` message fields from Basilisk `2.4.0`
- `spacecraft` module API from Basilisk `2.10.0`

This is sufficient to implement the backend boundary and fail-fast import behavior, but it is not
yet a substitute for validating against a single installed Basilisk release in CI or in the
development container.

## Current Basilisk Scenario Assumptions

The current backend uses an internal validation scenario named
`translational_3dof_axis_thrusters`.

Current scenario assumptions are:

- initial translational state matches ASTRO's current `EnvNode` default state
- telemetry is read from `scStateOutMsg.r_BN_N` and `scStateOutMsg.v_BN_N`
- the exported ROS vector remains `[x, y, z, vx, vy, vz]`
- the spacecraft attitude is initialized to zero and angular dynamics are initialized to zero
- translational control is represented by six center-of-mass thrusters aligned to `+x`, `-x`,
  `+y`, `-y`, `+z`, and `-z`
- ROS `thrust[3]` is split into positive and negative axis pairs using one-to-one axis ordering
- each axis command is converted into Basilisk thruster on-time requests over the bridge's
  `100 ms` timer period

## Current Runtime Requirements and Blockers

The current backend requires:

- the Basilisk Python package to be installed and importable as `Basilisk`
- the ROS 2 workspace to be built with both `distributed_satellite_sim` and
  `external_sim_bridge`
- launch via `external_sim_bridge/launch/bridge_sim.launch.py` or direct `ros2 run`

Known current blockers and gaps:

- the active workspace used for implementation does not have Basilisk installed
- the backend therefore has startup validation only, not a live end-to-end Basilisk smoke test
- the exact Basilisk release to support still needs to be pinned and tested as a single runtime
  source of truth
