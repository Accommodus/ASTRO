#!/usr/bin/env bash
# Avoid `set -u` because ROS setup scripts may reference env vars (e.g., COLCON_TRACE)
# without defining defaults, which would make the container fail during startup.
set -eo pipefail

if [[ -z "${ROLE:-}" ]]; then
  echo "Error: ROLE env var must be set to 'ENV' or 'GNC' (got empty)." >&2
  exit 2
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

