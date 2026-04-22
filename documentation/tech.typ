#set table(stroke: 0.5pt)

= Technical Details

Unless noted otherwise, paths are relative to the ASTRO repository root on the `main` branch. Project reports, presentations, and Typst manuscripts may live on a separate branch (for example `no-merge/manuscripts`) alongside the software; the implementation described here tracks `main`.

== Codebase overview

ASTRO is developed in a single public GitHub repository, #link("https://github.com/Accommodus/ASTRO")[Accommodus/ASTRO], with default branch `main`. Work is coordinated through Git branches, pull requests, and GitHub Issues.

The repository is a ROS 2 workspace. `src/DistributedSatelliteSim/` (`distributed_satellite_sim`, C++) provides `env_node`, DLQR `gnc_node`, and QP-MPC `qp_gnc_node`, plus launches and tests. `src/external_sim_bridge/` (`external_sim_bridge`, Python) supplies an optional `bridge_node` that uses the same `env_data` topic and `actuation_cmd` service with pluggable backends (`fake`, `basilisk`). Standalone reference code for ports and validation lives under `reference/`. Multi-machine demos use Docker Compose under `demo/` (build context is the repository root). Images and entrypoints live under `.docker/`; CI under `.github/workflows/`; devcontainers under `.devcontainer/`; planning notes in `docs/` (including `next-phase-plan.md`). A long-lived branch `no-merge/manuscripts` holds presentations and related material separately from `main`.

== Key dependencies and libraries

The C++ package uses `ament_cmake`, `rclcpp`, `std_msgs`, and `rosidl` generators and runtime for `ActuationCmd.srv`. Eigen3 supports linear algebra; QuadProg++ is vendored for QP-MPC. Tests use `ament_cmake_gtest`, `launch_testing`, `launch_ros`, `rclpy`, and related ament tooling.

The Python bridge package depends on `rclpy`, `std_msgs`, `numpy`, optional Basilisk (`bsk==2.10.0`), `pytest`, and `setuptools`.

Building from source assumes ROS 2 Kilted, `colcon`, `rosdep`, Eigen3, and a C++17 compiler. For distributed deployments, the published runtime uses Eclipse Zenoh (`rmw_zenoh_cpp`) so peers can discover each other across hosts without relying solely on multicast DDS across routed networks; peers are configured via environment variables and the stacks described in `demo/README.md`.

== Deployment instructions

=== Local development

From the repository root, after sourcing ROS 2, run `rosdep update` and `rosdep install --from-paths src --ignore-src -y`, then `colcon build --packages-select distributed_satellite_sim external_sim_bridge` (or a subset), then `source install/setup.bash`. Typical launches include `ros2 launch distributed_satellite_sim sim.launch.py` (DLQR), `ros2 launch distributed_satellite_sim qp_mpc_launch.py` (QP-MPC), and `ros2 launch external_sim_bridge bridge_sim.launch.py` with `backend_type:=fake` or `backend_type:=basilisk`. Launch arguments such as `max_steps` and `min_subscribers` control run length and startup synchronization. Devcontainers under `.devcontainer/` automate `rosdep` on first open where configured.

=== Containers

Published images include the base dev environment and the simulation deployment image below. The deployment container uses `ROLE=ENV` or `ROLE=GNC`. Common variables include `ROS_DOMAIN_ID`, `MAX_STEPS`, `MIN_SUBSCRIBERS`, and `SIM_RATE_MS`. For Tailscale-based demos, `TS_AUTHKEY` and Tailscale hostname variables apply; for LAN setups, `LAN_PEER_HOST` or explicit `ROS_DISCOVERY_PEER` matter. See `demo/README.md` for full semantics. Images can be built from `.docker/distributed_satellite_sim.Dockerfile` instead of pulling.

#table(
  columns: 2,
  [*Image*], [*Description*],
  [`ghcr.io/accommodus/astro:latest`],
  [Base devcontainer — ROS 2 Kilted and dev tooling (`linux/amd64` and `linux/arm64`).],

  [`ghcr.io/accommodus/astro/distributed-satellite-sim:latest`],
  [Deployment image — built workspace with Zenoh RMW; override via `SIM_IMAGE` if needed.],
)

CI rebuilds these when the corresponding Dockerfiles or package paths change.

=== Two-machine demo

Run Compose from `demo/` so paths resolve. Use `compose.tailscale.yaml` with `TS_AUTHKEY` and profiles `env` / `gnc` on separate hosts, or `compose.local.yaml` on a shared LAN with host networking and `LAN_PEER_HOST` (or `ROS_DISCOVERY_PEER`). Zenoh carries discovery and traffic per `demo/README.md` (including port 7447 on LAN setups). If `/dev/net/tun` is unavailable on the host, `TS_USERSPACE=true` may be required for Tailscale.

=== Tests

Run `colcon test --packages-select distributed_satellite_sim external_sim_bridge` and `colcon test-result --verbose`.

== Database schema and data migration details

ASTRO does not use a relational database for runtime operation. Simulation state lives in process memory and is exchanged through ROS 2 messages and services. Persistent artifacts are files: for example, `src/DistributedSatelliteSim/test/data/dlqr_reference_trajectory.csv` (91 steps, six HCW state components) supports DLQR regression testing against standalone reference output within tolerance `1e-4`. Behavior is also configured through YAML under `src/DistributedSatelliteSim/config/` — notably `dlqr_params.yaml` and `qp_mpc_params.yaml` for timer periods, step limits, docking checks, and dynamics parameters. Changes are made by editing those files or passing launch arguments (e.g. `max_steps:=200`); there are no database migrations or ETL pipelines.

== Software architecture

At a high level, when a simulator cannot run as a native ROS 2 node, a bridge maps external simulator I/O to the same ROS interfaces; `external_sim_bridge` implements that for selected backends. When everything runs as ROS 2 nodes, they share one graph — the primary mode on `main`.

The minimal contract is topic `env_data` (`std_msgs/Float64MultiArray`) for the six-element state each step, and service `actuation_cmd` for a three-axis thrust command. That lets either the C++ environment or the Python bridge stand in as the “environment” side.

#table(
  columns: 3,
  [*Node*], [*Executable*], [*Role*],
  [`EnvNode`], [`env_node`], [HCW dynamics; publishes `env_data`, serves `actuation_cmd`.],
  [`GncNode`], [`gnc_node`], [DLQR: $u = -K x$; calls `actuation_cmd`.],
  [`QpGncNode`],
  [`qp_gnc_node`],
  [QP-MPC (15-step horizon); QuadProg++ with matrices under `include/distributed_satellite_sim/matrices/`.],
)

`EnvNode` steps the discrete-time model, can wait for `min_subscribers`, and may stop on docking tolerances when enabled. `QpGncNode` solves a receding-horizon QP each cycle and applies the first control chunk.

`bridge_node` mirrors `env_node`'s interfaces. The `fake` backend uses NumPy linear HCW dynamics for tests without Basilisk; `basilisk` wraps Basilisk `bsk==2.10.0` for higher-fidelity physics. Backends implement the protocol in `external_sim_bridge/backends/`.

=== Operating modes and deployment

#table(
  columns: 2,
  [*Mode*], [*Description*],
  [Internal],
  [
    `env_node` publishes state on `env_data`; `gnc_node` or `qp_gnc_node` subscribes and sends thrust through `actuation_cmd` back to the environment.
  ],
  [Bridge],
  [
    `bridge_node` plays the environment role (same topic and service contract) while delegating physics to a backend; GNC nodes require no changes.
  ],
  [Cross-host],
  [
    Across machines, `rmw_zenoh_cpp` and the `demo/` Compose stacks (`compose.tailscale.yaml`, `compose.local.yaml`) configure peers for stable cross-host operation.
  ],
)

=== CI/CD workflows

Workflows under `.github/workflows/` build and publish images and validate the bridge package (`build-base-image.yml`, `build-distributed-satellite-sim-image.yml`, `validate-external-sim-bridge.yml`, `track-issues.yml`); see the repository for triggers.

== Current limitations and tracked follow-on work

On `main`, dynamics and scenarios can remain partially hardcoded compared to the long-term vision in `docs/next-phase-plan.md`, and richer operator tooling is still largely future work. The GitHub issue tracker continues to capture enhancements (for example telemetry and logging). Strengthening package-level CI so every pull request runs the same checks as local development remains an important follow-on.
