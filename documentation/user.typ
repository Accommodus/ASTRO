= User Documentation

This section describes how operators run the two-machine distributed satellite simulation using the Docker Compose stacks in the ASTRO repository, and how developers extend the ROS2 workspace. Unless noted otherwise, paths are relative to the repository root on the `main` branch, with your shell current working directory at that root. The authoritative reference for compose variables and commands is #link("https://github.com/Accommodus/ASTRO/blob/main/demo/README.md")[demo/README.md].

== Running the distributed demo

The demo ships two equivalent compositions: `demo/compose.tailscale.yaml` for hosts joined to a Tailscale tailnet, and `demo/compose.local.yaml` for machines on the same LAN using host networking and Zenoh discovery over TCP (port 7447). In both cases you run exactly one stack per physical machine and choose a single Compose profile so the container receives the correct `ROLE`: `env` runs `env_node` (typically a desktop or workstation), and `gnc` runs `gnc_node` (typically a Jetson). Discovery between hosts does not rely on plain DDS multicast across the WAN or Docker Desktop NAT; the image entrypoint configures Eclipse Zenoh (`rmw_zenoh_cpp`) when a peer address or hostname is available via `ROS_DISCOVERY_PEER` or the Tailscale-oriented auto-peer names described in the demo README.

Always run Compose from `demo/` so the compose file's build `context: ../` resolves to the repository root, or pass an explicit project directory as documented in `demo/README.md`. Create a file named `.env` in `demo/` (it is gitignored) for `COMPOSE_PROFILES` (`env` or `gnc`), optional `COMPOSE_FILE` (`compose.local.yaml` or `compose.tailscale.yaml`), and any secrets or peer addresses. Treat `TS_AUTHKEY` like a password and restrict file permissions on Linux (for example `chmod 600 .env`).

=== Tailscale (`compose.tailscale.yaml`)

Use this stack when the ENV and GNC machines are not on the same LAN or when you prefer Tailscale. Each host runs a Tailscale sidecar; the simulation container shares that sidecar's network (`network_mode: service:tailscale-*`). Obtain a reusable or ephemeral Tailscale auth key from the admin console (Settings → Keys), use the same tailnet on both hosts, and keep `ROS_DOMAIN_ID` identical on both sides (the compose default is `42`). On the ENV machine run `docker compose -f compose.tailscale.yaml --profile env up` with `TS_AUTHKEY` set (via export or `.env`); on the GNC machine run the same file with `--profile gnc` and the same key. You may set `COMPOSE_FILE=compose.tailscale.yaml` and `COMPOSE_PROFILES` in `.env` so a plain `docker compose up` from `demo/` suffices. Optional hostnames default to `astro-sim-env` and `astro-sim-gnc`; adjust `TS_HOSTNAME_ENV`, `TS_HOSTNAME_GNC`, or the `TAILSCALE_SIM_NAME_*` aliases consistently on both sides. If MagicDNS names fail inside the container, set `ROS_DISCOVERY_PEER` to the other host's tailnet IP or hostname. Kernel `/dev/net/tun` is required when `TS_USERSPACE` is false; if the bind fails in your Docker setup, follow the userspace-mode notes at the bottom of `compose.tailscale.yaml`.

=== Local LAN with Zenoh (`compose.local.yaml`)

Use this stack on a shared lab or office network without Tailscale. Both services use `network_mode: host` so Zenoh can use real LAN addresses and port 7447, which is especially important on Jetson where the default Docker bridge often misbehaves. Set `LAN_PEER_HOST` to the *other* host's IP or resolvable hostname: on the ENV machine point it at the Jetson, and on the GNC machine point it at the desktop. Run `docker compose -f compose.local.yaml --profile env up` on one machine and `--profile gnc` on the other, or mirror the same choices in `.env` with `COMPOSE_FILE=compose.local.yaml`. If you prefer explicit peers, set `ROS_DISCOVERY_PEER` (comma-separated); when set, it overrides `LAN_PEER_HOST`. On Docker Desktop for Windows or macOS, enable host or LAN access for Linux containers so host networking behaves as expected; if the first connection stalls, ping the Jetson once from the desktop to warm ARP, as noted in the demo README. If the desktop cannot use host mode, use a small compose override with a bridge network, publish `7447:7447`, and set `ROS_DISCOVERY_PEER` to this machine's LAN address as seen from the Jetson.

=== Shared runtime options

Both compose files accept the same simulation-related environment variables where applicable, including `MAX_STEPS`, `MIN_SUBSCRIBERS`, `SIM_RATE_MS`, and `SIM_IMAGE` (default `ghcr.io/accommodus/astro/distributed-satellite-sim:latest`). To build the image from the repository instead of pulling, the compose definitions include a `build` section that uses `.docker/distributed_satellite_sim.Dockerfile` with build context set to the repository root (the parent of `demo/`).

== Developer guide: adding packages and wiring interfaces

New ROS2 packages should live under `src/`, declare dependencies in `package.xml`, and list build and test targets in `CMakeLists.txt` following the existing `distributed_satellite_sim` package. From the repository root, install system dependencies with `rosdep install --from-paths src --ignore-src -y`, then build with `colcon build --packages-select <your_package>` and source `install/setup.bash` before running or testing.

Align new nodes with the established graph boundaries: the baseline publishes simulated state on `env_data` and exchanges actuator commands through the `actuation_cmd` service type defined in `distributed_satellite_sim`. Reuse those interfaces when extending the digital twin so multiple lab projects can interoperate without ad hoc sockets. Expose tunable behavior through ROS2 parameters and YAML where possible rather than hardcoding dynamics, gains, or timing in node sources, so launch files and tests can reproduce scenarios. When adding launch files, mirror the patterns in `sim.launch.py` (explicit arguments such as `max_steps` and `min_subscribers`, clear defaults) and extend the package test suite with `colcon test` so regressions remain detectable in CI and on developer machines.

For distributed or containerized runs, keep `ROS_DOMAIN_ID` and discovery-related variables consistent with the demo compositions documented above; new packages that introduce additional topics or services should document their names and QoS expectations beside the existing `env_data` / `actuation_cmd` contract.

== Training materials and screenshots

Operator training should combine this chapter with a short live demo: start `env_node` and `gnc_node` locally, show `ros2 topic echo /env_data`, and walk through one distributed run using `demo/README.md`. Screenshots of RViz, Foxglove, or terminal sessions are not embedded in this PDF build; add figures to a slide deck or print appendix if your course requires visual evidence. Video of the final presentation is linked from the title page.

== Frequently asked questions

*Why Zenoh instead of default DDS across two laptops?* Multicast discovery is unreliable across NAT, Docker Desktop, and many lab networks; the deployment image configures `rmw_zenoh_cpp` with explicit peers so ENV and GNC containers find each other.

*Which compose file should I use?* Use `compose.tailscale.yaml` when hosts are on different networks or you want Tailscale; use `compose.local.yaml` on one LAN with `LAN_PEER_HOST` pointing at the peer.

*Where do I change simulation length?* Use launch arguments (e.g. `max_steps`) or YAML under `src/DistributedSatelliteSim/config/` as described in the Technical Details and appendices.
