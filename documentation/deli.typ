= Deliverables

== Completed Deliverables
The ASTRO project delivered a working ROS2-based simulation and control baseline in the `distributed_satellite_sim` package. The team completed the core ROS2 architecture needed to run a closed-loop satellite simulation workflow and validated it through automated testing and repository workflows.

This includes a functional environment node (`env_node`) and guidance, navigation, and control node (`gnc_node`) connected through ROS2 communication interfaces, along with a defined ROS2 service contract (`ActuationCmd.srv`) and a launch entrypoint (`sim.launch.py`) to run the closed-loop scenario. The team also delivered unit tests for both environment and GNC nodes, plus launch-based regression coverage for the DLQR reference trajectory. Supporting infrastructure, including devcontainer setup and deployment image workflows, was completed for reproducible execution across systems, and the QP_MPC controller integration milestone has been marked complete in the repository tracker.

== Implemented Functional Specifications
The current implementation satisfies the primary functional requirements for a modular digital-twin baseline.
The system now provides closed-loop simulation and control operation where environment state is published, control is computed, and actuation commands are returned through ROS2. It uses standardized ROS2 topic/service communication to replace ad-hoc coupling with clearer integration boundaries, and supports launch-driven execution for repeatable startup and test automation. The package layout is modular enough to support some of our recent additions such as alternate controllers, bridge adapters, and telemetry tooling.

== Current Project Status
The project is currently in a stable, delivered baseline state on `main` for the ROS2 DLQR and MPC workflows. The previously planned testing and integration milestones have been completed and merged, and the repository now contains validated node behavior, regression checks, and deployment-oriented infrastructure.

In practical terms, ASTRO has moved from architecture and initial porting into an operational baseline that can be demonstrated, tested, and incrementally expanded.

== Remaining Work, Backlog, and Risks
The remaining backlog is now focused on expansion and long-term robustness rather than baseline feasibility. With the conclusion of the Issues `#24` and `#25` Basilisk has been verified and circular telemetry has been added for tooling and debugging support. At the point in time we have no open issues, but are open to further expansion as it becomes necessary. There is also a maintainability risk as the project grows beyond DLQR-centric assumptions, which reinforces the need for strict interface contracts and clean configuration boundaries.
