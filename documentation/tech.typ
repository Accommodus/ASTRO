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

All packages target *ROS 2 Kilted* (`ros:kilted`). The build system is `colcon` with `ament_cmake` (C++) and `ament_python` (Python).

The C++ package (`distributed_satellite_sim`) depends on `rclcpp` and the standard ROS 2 message types, `Eigen3` for linear algebra, and a vendored copy of the QuadProg++ solver. The Python package (`external_sim_bridge`) depends on `rclpy`, `numpy`, and optionally Basilisk (`bsk==2.10.0`) for high-fidelity physics simulation. See @appendix-dependencies for the full dependency manifest.

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

Four launch configurations are provided: `sim.launch.py` (DLQR), `qp_mpc_launch.py` (QP-MPC), and `bridge_sim.launch.py` with either `backend_type:=fake` or `backend_type:=basilisk`. See @appendix-launch-commands for the full build and launch commands.

=== Environment Variables

Two variables are required at runtime: `ROLE` (either `ENV` to start the environment node or `GNC` to start the GNC node) and `TS_AUTHKEY` (Tailscale auth key, required for distributed deployment — never commit this value). See @appendix-env-vars for the full environment variable reference.

=== Distributed Deployment (Two-Machine Demo)

`demo/compose.yaml` defines a Tailscale sidecar pattern allowing `env_node` and `gnc_node` to run on separate physical machines. Each machine runs its role profile with a Tailscale auth key; ROS 2 communication is tunneled over Tailscale via `rmw_zenoh_cpp` on TCP port 7447. See @appendix-distributed-deployment for the step-by-step deployment procedure.

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

Runtime behavior is configured via YAML parameter files loaded at launch time. `config/dlqr_params.yaml` sets the timer period, step count, docking check flag, and initial state. `config/qp_mpc_params.yaml` additionally embeds the discrete-time `Ad` and `Bd` system matrices for the HCW equations at the chosen orbital parameters. Parameter changes are made by editing these files or passing ROS 2 launch arguments (e.g., `ros2 launch ... max_steps:=200`). There are no migration steps.

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

Four GitHub Actions workflows handle CI/CD: two build and push the devcontainer and deployment images on changes to their respective Dockerfiles, one validates both ROS packages by building with colcon and running the test suite on every push or PR touching `src/`, and one automates issue and PR branch lifecycle management. See @appendix-cicd for the full workflow table.
