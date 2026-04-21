= Technical Details

== Codebase Overview

=== Repository

ASTRO is available at #link("https://github.com/Accommodus/ASTRO")[github.com/Accommodus/ASTRO] as a standard ROS 2 workspace. The top-level layout of it is

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt,
  [*Path*], [*Purpose*],
  [`src/`], [ROS 2 packages (primary source code)],
  [`.devcontainer/`], [OS-specific devcontainer configs and base Dockerfile],
  [`.docker/`], [Deployment image Dockerfile and entrypoint script],
  [`.github/`], [GitHub Actions workflows and issue-branch config],
  [`demo/`], [Docker Compose for distributed two-machine deployment],
  [`docs/`], [Living roadmap (`next-phase-plan.md`)],
  [`reference/`], [Standalone C++ DLQR and QP-MPC reference executables],
)

Two ROS2 packages live within `src/`:

- `src/DistributedSatelliteSim/` - C++ package (`distributed_satellite_sim`)
- `src/external_sim_bridge/` - Python package (`external_sim_bridge`)

=== Version Control

The repository uses Git with GitHub as the remote. The default branch is `main`. A notable branch is `no-merge/manuscripts` as this is where all the presentations live and further details about the weekly updates of the project can be found there.

== Key Dependencies and Libraries

=== ROS 2 Platform

All packages target *ROS 2 Kilted* (`ros:kilted`). The build system is `colcon` with `ament_cmake` (C++) and `ament_python` (Python).

=== C++ Package — `distributed_satellite_sim`

#table(
  columns: (auto, auto, 1fr),
  stroke: 0.5pt,
  [*Dependency*], [*Source*], [*Purpose*],
  [`rclcpp`], [ROS 2], [C++ ROS client library],
  [`std_msgs`], [ROS 2], [Standard message types (`Float64MultiArray`)],
  [`rosidl_default_generators`], [ROS 2], [Custom service (`ActuationCmd`) code generation],
  [`Eigen3`], [System], [Linear algebra — state vectors and gain matrix multiplication],
  [QuadProg++], [Vendored (`quadprogpp/`)], [Quadratic programming solver for QP-MPC],
  [`ament_cmake_gtest`], [ROS 2 (test)], [GTest integration via ament],
  [`launch_testing_ament_cmake`], [ROS 2 (test)], [Launch-file regression tests],
)

=== Python Package — `external_sim_bridge`

#table(
  columns: (auto, auto, 1fr),
  stroke: 0.5pt,
  [*Dependency*], [*Version*], [*Purpose*],
  [`rclpy`], [ROS 2], [Python ROS client library],
  [`std_msgs`], [ROS 2], [Shared message types with the C++ package],
  [`numpy`], [System (`python3-numpy`)], [Linear dynamics in `FakeBackend`],
  [`Basilisk`], [`bsk==2.10.0`], [High-fidelity physics engine (optional; `BasiliskBackend` only)],
  [`pytest`], [System], [Unit and integration test runner],
  [`setuptools`], [System], [Python package installation],
)

== Deployment Instructions

=== Prerequisites

- Docker with Buildx (for multi-arch builds)
- A Tailscale account and auth key (`TS_AUTHKEY`) for distributed deployment
- `ros:kilted` base image (pulled automatically)

=== Container Images

Two images are published to the GitHub Container Registry:

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt,
  [*Image*], [*Description*],
  [`ghcr.io/accommodus/astro:latest`], [Base devcontainer — ROS 2 Kilted + dev tools],
  [`ghcr.io/accommodus/astro/distributed-satellite-sim:latest`], [Deployment image — built workspace + Zenoh RMW],
)

Both images support `linux/amd64` and `linux/arm64`. CI rebuilds the base image on changes to `.devcontainer/base.Dockerfile` and the deployment image on changes to `src/DistributedSatelliteSim/` or `.docker/`.

=== Local Development (devcontainer)

Open the repository in VS Code and reopen in the devcontainer for your platform (Linux, Windows, or macOS). The `postCreateCommand` automatically runs:

```bash
rosdep update && rosdep install --from-paths src --ignore-src -y
```

Build and run the simulation locally:

```bash
source /opt/ros/kilted/setup.bash
colcon build --packages-select distributed_satellite_sim external_sim_bridge
source install/setup.bash

# DLQR simulation
ros2 launch distributed_satellite_sim sim.launch.py

# QP-MPC simulation
ros2 launch distributed_satellite_sim qp_mpc_launch.py

# Bridge simulation (fake backend, no Basilisk required)
ros2 launch external_sim_bridge bridge_sim.launch.py backend_type:=fake

# Bridge simulation (Basilisk backend)
ros2 launch external_sim_bridge bridge_sim.launch.py backend_type:=basilisk
```

=== Environment Variables

#table(
  columns: (auto, auto, 1fr),
  stroke: 0.5pt,
  [*Variable*], [*Default*], [*Description*],
  [`ROLE`], [_(required)_], [`ENV` starts the environment node; `GNC` starts the GNC node],
  [`ROS_DOMAIN_ID`], [`42`], [ROS 2 domain isolation],
  [`ROS_AUTOMATIC_DISCOVERY_RANGE`], [`SUBNET`], [`LOCALHOST` in devcontainer; `SUBNET` in compose],
  [`ROS_DISCOVERY_PEER`], [_(unset)_], [Comma-separated unicast peer IPs/hostnames],
  [`ROS_AUTO_TAILSCALE_PEER`], [`1`], [Auto-configure Zenoh unicast peer from Tailscale hostnames],
  [`TAILSCALE_SIM_NAME_ENV`], [`astro-sim-env`], [Tailscale hostname of the ENV machine],
  [`TAILSCALE_SIM_NAME_GNC`], [`astro-sim-gnc`], [Tailscale hostname of the GNC machine],
  [`TS_AUTHKEY`], [_(required for demo)_], [Tailscale authentication key — never commit],
  [`MAX_STEPS`], [`91`], [Simulation step count passed to `env_node`],
  [`MIN_SUBSCRIBERS`], [`1`], [Startup subscriber barrier for `env_node`],
  [`SIM_RATE_MS`], [`100`], [Simulation timer period in milliseconds],
  [`ASTRO_EXPECT_BASILISK`], [_(unset)_], [Set to `1` to hard-fail CI if Basilisk is missing],
)

=== Distributed Deployment (Two-Machine Demo)

`demo/compose.yaml` defines a Tailscale sidecar pattern that allows `env_node` and `gnc_node` to run on separate physical machines. ROS 2 communication is tunneled over Tailscale using `rmw_zenoh_cpp` (TCP unicast, port 7447).

*Machine 1 — ENV role:*
```bash
export TS_AUTHKEY='tskey-auth-...'
docker compose -f demo/compose.yaml --profile env up
```

*Machine 2 — GNC role:*
```bash
export TS_AUTHKEY='tskey-auth-...'
docker compose -f demo/compose.yaml --profile gnc up
```

Each profile starts a Tailscale sidecar alongside the satellite container, which shares the sidecar's network namespace via `network_mode: service:tailscale-*`. The entrypoint auto-detects peer hostnames via Tailscale DNS when `ROS_AUTO_TAILSCALE_PEER=1`. If the host does not expose `/dev/net/tun`, set `TS_USERSPACE=true` to use Tailscale's userspace networking mode.

=== Running Tests

```bash
colcon test --packages-select distributed_satellite_sim
colcon test --packages-select external_sim_bridge
colcon test-result --verbose
```

---

== Data and Configuration

ASTRO does not use a relational database. Persistent data consists of two artifacts:

=== Reference Trajectory Fixture

`src/DistributedSatelliteSim/test/data/dlqr_reference_trajectory.csv`

A 91-row, 6-column CSV file containing the ground-truth DLQR state trajectory used for regression testing. Each row is one simulation step; columns are the six HCW state variables `[x, y, z, vx, vy, vz]`. This file was generated from the standalone C++ reference executable in `reference/` and is committed to the repository. The launch regression test (`test_dlqr_reference_launch.py`) compares live simulation output against this fixture within a tolerance of `1e-4`.

=== ROS 2 Parameter Configuration Files

Runtime behavior is configured via YAML parameter files loaded at launch time:

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt,
  [*File*], [*Contents*],
  [`config/dlqr_params.yaml`],
  [`timer_period_ms: 100`, `max_steps: 91`, `enable_docking_check: false`, initial state `X0`],

  [`config/qp_mpc_params.yaml`],
  [`timer_period_ms: 30000` (30 s/step), `max_steps: 1000`, `enable_docking_check: true`, explicit `Ad` (6×6) and `Bd` (6×3) matrices for HCW at $T_s = 30$ s, $R_e = 6371$ km, $R_o = 650$ km],
)

Parameter changes are made by editing these YAML files or by passing ROS 2 launch arguments at startup (e.g., `ros2 launch ... max_steps:=200`). There are no migration steps.

---

== Software Architecture

=== Overview

ASTRO implements a closed-loop spacecraft relative-motion control simulation. The two ROS 2 packages communicate over one topic and one service:

- *Topic* `env_data` (`std_msgs/Float64MultiArray`) — the environment node publishes the current 6-element state vector `[x, y, z, vx, vy, vz]` at each simulation step.
- *Service* `actuation_cmd` (`distributed_satellite_sim/srv/ActuationCmd`) — the GNC node sends a 3-element thrust command; the environment node applies it and returns a success flag.

This interface is intentionally minimal so that either side can be swapped out independently.

=== Package: `distributed_satellite_sim` (C++)

#table(
  columns: (auto, auto, 1fr),
  stroke: 0.5pt,
  [*Node*], [*Executable*], [*Role*],
  [`EnvNode`],
  [`env_node`],
  [Simulates HCW spacecraft dynamics; owns the `env_data` publisher and `actuation_cmd` service server],

  [`GncNode`], [`gnc_node`], [DLQR controller; computes $bold(u) = -K bold(x)$ and calls `actuation_cmd`],
  [`QpGncNode`],
  [`qp_gnc_node`],
  [QP-MPC controller (horizon = 15); solves a quadratic program each step using QuadProg++],
)

`EnvNode` maintains simulation state and advances it on each timer tick by applying $bold(x)_(k+1) = A_d bold(x)_k + B_d bold(u)_k$, then publishing the new state. It blocks startup until `min_subscribers` GNC nodes have subscribed to `env_data`, preventing a race condition at launch. When `enable_docking_check` is true, the simulation halts once $||bold(r)|| < epsilon_"pos"$ and $||bold(v)|| < epsilon_"vel"$.

`GncNode` (DLQR) uses a pre-computed $3 times 6$ gain matrix $K$ derived from the discrete-time LQR solution for the HCW equations. It fires on each `env_data` message and issues a thrust command synchronously.

`QpGncNode` (QP-MPC) formulates a receding-horizon QP over 15 steps. The QP matrices ($H$, $Q$, $A_"ineq"$, $b_"ineq"$) are pre-generated and compiled in as C++ headers under `include/distributed_satellite_sim/matrices/`. At each step the node solves the 45-variable QP via QuadProg++ and applies the first 3 control inputs.

=== Package: `external_sim_bridge` (Python)

Provides a drop-in replacement for `env_node` that delegates physics to a pluggable external simulator backend. The node exposes the same `env_data` topic and `actuation_cmd` service, so the GNC node requires no modification.

The backend is selected by the `backend_type` ROS parameter:

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt,
  [*Backend*], [*Description*],
  [`fake`],
  [Pure-numpy linear HCW dynamics; produces output bit-identical to `EnvNode`. Used for CI regression and bridge-core testing without Basilisk.],

  [`basilisk`],
  [Wraps the Basilisk (`bsk==2.10.0`) physics engine. Models a 1 kg spacecraft with six center-of-mass thrusters (±x, ±y, ±z, 1 N max each). Maps the 3-element thrust command to Basilisk on-time fractions over a 100 ms step, then reads position and velocity from `scStateOutMsg`.],
)

The backend interface is defined as a Python `Protocol` (`backends/base.py`) with four methods: `initialize`, `validate_control`, `apply_control`, and `advance`. New simulator backends require only implementing this protocol.

=== Closed-Loop Data Flow

```
┌──────────────────────────────────────────────────────────────┐
│  Mode A — Internal simulation (distributed_satellite_sim)    │
│                                                              │
│  env_node ──[env_data]──► gnc_node / qp_gnc_node            │
│      ▲                           │                           │
│      └─────[actuation_cmd]───────┘                           │
└──────────────────────────────────────────────────────────────┘

┌──────────────────────────────────────────────────────────────┐
│  Mode B — External simulator bridge (external_sim_bridge)    │
│                                                              │
│  bridge_node ──[env_data]──► gnc_node                        │
│      ▲    ▲                        │                         │
│      │    └─────[actuation_cmd]────┘                         │
│      │                                                       │
│  FakeBackend | BasiliskBackend                               │
└──────────────────────────────────────────────────────────────┘
```

In distributed deployment, the two sides of either mode run in separate containers on separate machines. ROS 2 discovery is handled by `rmw_zenoh_cpp` over a Tailscale VPN (TCP unicast), replacing the default multicast DDS with a topology that works across routed networks.

=== CI/CD Pipeline

#table(
  columns: (auto, 1fr),
  stroke: 0.5pt,
  [*Workflow*], [*Trigger and Action*],
  [`build-base-image.yml`],
  [Push to `main` touching `.devcontainer/base.Dockerfile` → build and push `ghcr.io/accommodus/astro:latest` (amd64 + arm64)],

  [`build-distributed-satellite-sim-image.yml`],
  [Push to `main` touching `.docker/` or `src/DistributedSatelliteSim/` → build and push the deployment image (amd64 + arm64)],

  [`validate-external-sim-bridge.yml`],
  [Push or PR touching either `src/` package → build both packages with colcon, install `bsk==2.10.0`, run `external_sim_bridge` tests],

  [`track-issues.yml`],
  [Issue opened / PR opened or closed → auto-create issue branch, open draft PR, auto-close and delete branch on merge],
)
