= Appendix

== Dependency Manifest <appendix-dependencies>

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

== Build and Launch Commands <appendix-launch-commands>

Build both packages and source the workspace:

#raw(
  block: true,
  lang: "bash",
  "source /opt/ros/kilted/setup.bash\ncolcon build --packages-select distributed_satellite_sim external_sim_bridge\nsource install/setup.bash\n",
)

Run any of the four launch configurations:

#raw(
  block: true,
  lang: "bash",
  "ros2 launch distributed_satellite_sim sim.launch.py\nros2 launch distributed_satellite_sim qp_mpc_launch.py\nros2 launch external_sim_bridge bridge_sim.launch.py backend_type:=fake\nros2 launch external_sim_bridge bridge_sim.launch.py backend_type:=basilisk\n",
)

== Environment Variable Reference <appendix-env-vars>

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

== Distributed Deployment Procedure <appendix-distributed-deployment>

Both machines must have Docker installed. Run commands from `demo/` so Compose build context resolves to the repository root. For Tailscale, each host needs a valid `TS_AUTHKEY`. If the host does not expose `/dev/net/tun`, set `TS_USERSPACE=true` for Tailscale's userspace networking mode.

*Tailscale — machine 1 (ENV):*

#raw(
  block: true,
  lang: "bash",
  "export TS_AUTHKEY='tskey-auth-...'\ndocker compose -f compose.tailscale.yaml --profile env up\n",
)

*Tailscale — machine 2 (GNC):*

#raw(
  block: true,
  lang: "bash",
  "export TS_AUTHKEY='tskey-auth-...'\ndocker compose -f compose.tailscale.yaml --profile gnc up\n",
)

*Local LAN (no Tailscale):* use `compose.local.yaml` with `LAN_PEER_HOST` set to the peer's address and the same `env` / `gnc` profiles; see `demo/README.md`.

Each Tailscale profile starts a sidecar alongside the satellite container, sharing the sidecar's network namespace via `network_mode: service:tailscale-*`. The entrypoint auto-detects peer hostnames via Tailscale DNS when `ROS_AUTO_TAILSCALE_PEER=1`.

== CI/CD Workflow Table <appendix-cicd>

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
