# Distributed satellite sim — Docker Compose

This directory has two Compose stacks for the same two-node simulation (**ENV** and **GNC**). On each physical machine you run **one** stack and choose **one** profile so the container gets the right `ROLE`.

| Role | Typical machine | ROS node |
|------|-----------------|----------|
| **ENV** | Desktop / workstation | `env_node` |
| **GNC** | Jetson (NJON) | `gnc_node` |

Discovery between hosts does not rely on plain DDS multicast across the WAN or Docker Desktop NAT; the image entrypoint configures **Eclipse Zenoh** (`rmw_zenoh_cpp`) when a peer address or hostname is available (`ROS_DISCOVERY_PEER` or the Tailscale auto-peer names).

Run commands from this directory (`b/demo/`) so build `context: ../` resolves correctly.

---

## Dotenv configuration

This section describes the **`.env`** file (dotenv): a plain text file named exactly `.env` in the project directory.
Docker Compose reads it from the **project directory**. With the commands in this README, that directory should be **`b/demo/`** (the folder that contains the compose files). Variables in `.env` are used for **Compose interpolation** (`${VAR}` in the YAML) and are also passed into containers when listed under `environment:`.

`b/.gitignore` ignores `**/.env`, so each developer or machine keeps its own file and it is not committed.

**Choosing ENV vs GNC (both stacks)**  
Use Compose’s built-in variable **`COMPOSE_PROFILES`**: set it to exactly **`env`** or **`gnc`**. That is the same mechanism for **`compose.tailscale.yaml`** and **`compose.local.yaml`**—no separate “role” variable exists in the YAML; the profile name selects which service runs and sets the container’s `ROLE`.

**Optional: `COMPOSE_FILE`**  
If you put **`COMPOSE_FILE=compose.local.yaml`** or **`COMPOSE_FILE=compose.tailscale.yaml`** in `.env`, you can run plain **`docker compose up`** from `b/demo/` without passing **`-f`** every time. Use only one stack’s file in `COMPOSE_FILE` at a time (or override with `-f` on the command line when switching).

**Example — local LAN on the desktop (ENV)**

```dotenv
COMPOSE_FILE=compose.local.yaml
COMPOSE_PROFILES=env
LAN_PEER_HOST=192.168.1.80
```

**Example — local LAN on the Jetson (GNC)**

```dotenv
COMPOSE_FILE=compose.local.yaml
COMPOSE_PROFILES=gnc
LAN_PEER_HOST=192.168.1.40
```

**Example — Tailscale on the ENV machine**

```dotenv
COMPOSE_FILE=compose.tailscale.yaml
COMPOSE_PROFILES=env
TS_AUTHKEY=tskey-auth-...
```

Use **`COMPOSE_PROFILES=gnc`** and the same **`TS_AUTHKEY`** on the GNC host. Treat **`TS_AUTHKEY`** like a password (`chmod 600 .env` on Linux).

**Running from another directory**  
If you start Compose from elsewhere, set the project directory explicitly so `.env` and paths resolve correctly, for example:

```bash
docker compose --project-directory /path/to/b/demo -f /path/to/b/demo/compose.local.yaml up
```

Place **`.env`** in **`b/demo/`** when using **`--project-directory`** that way.

---

## Tailscale — `compose.tailscale.yaml`

Use this when the two machines are not on the same LAN (or you prefer Tailscale). Each host runs a **Tailscale sidecar**; the sim container shares that container’s network (`network_mode: service:tailscale-*`).

**Prerequisites**

- A Tailscale auth key: admin console → **Settings** → **Keys** → create a reusable or ephemeral key.
- Same **Tailscale tailnet** and same **`ROS_DOMAIN_ID`** on both hosts (default `42`).

**On the ENV machine**

```bash
export TS_AUTHKEY='tskey-auth-...'
docker compose -f compose.tailscale.yaml --profile env up
```

**On the GNC machine**

```bash
export TS_AUTHKEY='tskey-auth-...'
docker compose -f compose.tailscale.yaml --profile gnc up
```

To avoid **`export`** and **`--profile`**, put **`TS_AUTHKEY`**, **`COMPOSE_PROFILES`**, and optionally **`COMPOSE_FILE=compose.tailscale.yaml`** in **`b/demo/.env`** (see [Dotenv configuration](#dotenv-configuration)) and run **`docker compose up`** from **`b/demo/`**.

**Optional**

- Rename MagicDNS hostnames consistently with **`TS_HOSTNAME_ENV`** / **`TS_HOSTNAME_GNC`** (or **`TAILSCALE_SIM_NAME_ENV`** / **`TAILSCALE_SIM_NAME_GNC`**); defaults are `astro-sim-env` and `astro-sim-gnc`.
- If names fail in-container, set **`ROS_DISCOVERY_PEER`** to the other host’s tailnet IP (e.g. `100.x.y.z`) or hostname.
- **`ROS_AUTO_TAILSCALE_PEER=0`** turns off automatic peer selection (multicast-only; same host).

Kernel **`/dev/net/tun`** is required for **`TS_USERSPACE=false`** (default). If that bind fails, see the comments at the bottom of `compose.tailscale.yaml` for userspace mode.

---

## Local LAN — `compose.local.yaml`

Use this on the **same network** (e.g. lab Wi‑Fi) **without** Tailscale. Both services use **`network_mode: host`** so Zenoh can use real LAN addresses and port **7447** (important on Jetson, where the default Docker bridge often misbehaves).

**Prerequisites**

- Same **`ROS_DOMAIN_ID`** on both machines (default `42`).
- Each side must know how to reach the **other** host on the LAN.

**On the ENV machine** (set peer to the **Jetson** address)

```bash
export LAN_PEER_HOST=192.168.x.y   # GNC / Jetson IP or resolvable hostname
docker compose -f compose.local.yaml --profile env up
```

**On the GNC machine** (set peer to the **desktop** address)

```bash
export LAN_PEER_HOST=192.168.a.b   # ENV / desktop IP or hostname
docker compose -f compose.local.yaml --profile gnc up
```

For **`LAN_PEER_HOST`**, **`COMPOSE_PROFILES`**, and optional **`COMPOSE_FILE=compose.local.yaml`** in **`b/demo/.env`**, see [Dotenv configuration](#dotenv-configuration).

**Optional**

- Set **`ROS_DISCOVERY_PEER`** yourself (comma-separated); if set, it overrides **`LAN_PEER_HOST`**.

**Docker Desktop (Windows / macOS)**  
Enable host / LAN access for Linux containers so **`network_mode: host`** behaves as expected. If the first connection stalls, **ping the Jetson once from the desktop** so ARP/route is warm (known Docker Desktop quirk). If you cannot use host mode on the desktop, use a small compose override: bridge network, publish **7447:7447**, and set **`ROS_DISCOVERY_PEER`** to this machine’s **LAN IP as seen from the Jetson**.

---

## Shared knobs

Both stacks accept the same simulation-related environment variables where applicable (see the compose files), for example **`MAX_STEPS`**, **`MIN_SUBSCRIBERS`**, **`SIM_RATE_MS`**, and **`SIM_IMAGE`** (override the default `ghcr.io/accommodus/astro/distributed-satellite-sim:latest`).

To build the image from the repo instead of pulling, the compose files include a **`build`** section pointing at `b/` with `.docker/distributed_satellite_sim.Dockerfile`.
