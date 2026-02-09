Convert QP_MPC to ROS Package
===

1. Layout ROS structure:
    - Current Hardware Mapping:
        - Desktop: Handles external environment and simulation.
        - NJON (Edge): Executes the MCP algorithm.
    - ROS 2 Component Mapping:
        - Env (**Node**): Executes simulation software.
        - GNC (**Node**): Ported from the NJON part of `QP_MPC`. (Consider splitting into specific **Navigation**, **Guidance**, and **Control** nodes if complexity grows).
        - Communication:
            - `env_data` (**Topic**): High-frequency telemetry/state info from env to GNC.
            - `actuation_cmd` (**Service**): Control signals sent from GNC to the environment.
        - *Note: More ROS2 components may need to be added*
2. Implementation & Node Capture
    - Node Migration: Wrap existing `QP_MCP` functionality into ROS 2 nodes.
    - Distributed Deployment: Test on separate devices (Desktop and NJON).
    - Verification: Test and compare performance/outputs against the original standalone executable.
        - Automated Testing: Add integration tests using `launch_testing`.
            - Reference the [ROS 2 Testing Documentation](https://docs.ros.org/en/kilted/Tutorials/Intermediate/Testing/Testing-Main.html) for `ament_cmake_gtest` setup.

## Dylan:

- Make ROS2 Component Skeleton
- Populate Env Node with code in `reference/udp_hcw_discrete_txrx 2 1.cpp`
    - Remove networking
    - Replace logging with ROS2 logging

## Cannon:

- Extract final set of C++ header files

## Caleb:

- Populate Control Node with code in `reference/quadprogMPC_roundtrip.cpp`
    - Remove networking
    - Replace logging with ROS2 logging