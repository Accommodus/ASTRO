#!/usr/bin/env bash
# Avoid `set -u` because ROS setup scripts may reference env vars (e.g., COLCON_TRACE)
# without defining defaults, which would make the container fail during startup.
set -eo pipefail

if [[ -z "${ROLE:-}" ]]; then
  echo "Error: ROLE env var must be set to 'ENV' or 'GNC' (got empty)." >&2
  exit 2
fi

# Tailscale (and many VPNs) do not forward DDS multicast, so default DDS discovery does not
# reach the other host. With ROS_AUTO_TAILSCALE_PEER left at default, we point Cyclone at the
# sibling's TS_HOSTNAME (MagicDNS) via TAILSCALE_SIM_NAME_ENV / TAILSCALE_SIM_NAME_GNC.
# Override with ROS_DISCOVERY_PEER (comma-separated IPs or names), or set ROS_AUTO_TAILSCALE_PEER=0
# to use normal multicast discovery (same machine only).
_auto_peer="${ROS_AUTO_TAILSCALE_PEER:-1}"
if [[ -z "${ROS_DISCOVERY_PEER:-}" && "${_auto_peer}" != "0" && "${_auto_peer}" != "false" && "${_auto_peer}" != "no" ]]; then
  case "${ROLE}" in
    ENV)
      ROS_DISCOVERY_PEER="${TAILSCALE_SIM_NAME_GNC:-astro-sim-gnc}"
      ;;
    GNC)
      ROS_DISCOVERY_PEER="${TAILSCALE_SIM_NAME_ENV:-astro-sim-env}"
      ;;
  esac
fi

if [[ -n "${ROS_DISCOVERY_PEER:-}" ]]; then
  for _wait in $(seq 1 90); do
    [[ -d /sys/class/net/tailscale0 ]] && break
    sleep 1
  done
  if [[ ! -d /sys/class/net/tailscale0 ]]; then
    echo "Error: ROS_DISCOVERY_PEER is set but network interface tailscale0 is missing after wait." >&2
    exit 2
  fi

  peers_block=""
  IFS=',' read -ra _peer_addrs <<< "${ROS_DISCOVERY_PEER}"
  for _addr in "${_peer_addrs[@]}"; do
    _addr="${_addr#"${_addr%%[![:space:]]*}"}"
    _addr="${_addr%"${_addr##*[![:space:]]}"}"
    [[ -z "${_addr}" ]] && continue
    if [[ ! "${_addr}" =~ ^[0-9A-Za-z._:\\-]+$ ]]; then
      echo "Error: invalid ROS_DISCOVERY_PEER component '${_addr}' (use IPs, hostnames, or IPv6)." >&2
      exit 2
    fi
    peers_block+="        <Peer address=\"${_addr}\"/>\n"
  done
  if [[ -z "${peers_block}" ]]; then
    echo "Error: ROS_DISCOVERY_PEER is set but empty after parsing." >&2
    exit 2
  fi

  _cyclone_xml="/tmp/cyclonedds_tailscale.xml"
  printf '%s\n' \
    '<?xml version="1.0" encoding="UTF-8" ?>' \
    '<CycloneDDS>' \
    '  <Domain>' \
    '    <General>' \
    '      <Interfaces>' \
    '        <NetworkInterface name="tailscale0" priority="default" multicast="false"/>' \
    '      </Interfaces>' \
    '      <AllowMulticast>false</AllowMulticast>' \
    '    </General>' \
    '    <Discovery>' \
    '      <Peers>' \
    "$(printf '%b' "${peers_block}")" \
    '      </Peers>' \
    '    </Discovery>' \
    '  </Domain>' \
    '</CycloneDDS>' > "${_cyclone_xml}"

  export CYCLONEDDS_URI="file://${_cyclone_xml}"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  unset ROS_AUTOMATIC_DISCOVERY_RANGE
fi

source /astro_ws/install/setup.bash

case "${ROLE}" in
  ENV)
    # EnvNode parameters (declared in env_node.hpp):
    # - max_steps (default 91 in sim.launch.py)
    # - min_subscribers (default 1 in sim.launch.py)
    MAX_STEPS="${MAX_STEPS:-91}"
    MIN_SUBSCRIBERS="${MIN_SUBSCRIBERS:-1}"

    exec ros2 run distributed_satellite_sim env_node \
      --ros-args \
      -p "max_steps:=${MAX_STEPS}" \
      -p "min_subscribers:=${MIN_SUBSCRIBERS}"
    ;;
  GNC)
    exec ros2 run distributed_satellite_sim gnc_node
    ;;
  *)
    echo "Error: ROLE env var must be exactly 'ENV' or 'GNC' (got '${ROLE}')." >&2
    exit 2
    ;;
esac

