#import "style.typ": presentation, text-diagram
#import "diagrams.typ": external-arch, internal-arch
#import "@preview/fletcher:0.5.8": *
#import "@preview/touying:0.6.3": speaker-note

#show: presentation

= Introduction

== What is ASTRO?

*Autonomous Satellite Test & Robotics Operations* — a ROS 2 framework that bridges STAR Lab satellite simulation software with guidance, navigation, and control (GNC) software in a modular, reproducible way.

- Replaces ad hoc UDP wiring with a standard ROS 2 graph
- Runs locally or across two machines (desktop + Jetson)
- Supports both native ROS 2 nodes and an external simulator bridge

#speaker-note[Keep this under 30 seconds. Frame the problem, then say "the rest of the presentation is mostly a live walkthrough."]

== Today's Agenda

+ Project walkthrough (live demo)
+ Individual contributions
+ Wrap-up and next steps

#speaker-note[Remind viewers the walkthrough is live software, not screenshots.]

= Project Walkthrough

== Live Demo Plan

The bulk of this video is the live demo below. Segments:

- Repository tour — `src/DistributedSatelliteSim/`, `src/external_sim_bridge/`, `demo/`, `.docker/`
- Local run — `ros2 launch distributed_satellite_sim sim.launch.py` (DLQR) and `qp_mpc_launch.py` (QP-MPC)
- Bridge run — `ros2 launch external_sim_bridge bridge_sim.launch.py backend_type:=fake` / `backend_type:=basilisk`
- Two-machine demo — `demo/compose.tailscale.yaml` and `demo/compose.local.yaml`
- Test suite — `colcon test --packages-select distributed_satellite_sim external_sim_bridge`

#speaker-note[Cut to screen share here. Keep each segment tight; don't narrate anything you are not actively doing on screen.]

== Architecture

#text-diagram(
  [Internal ROS 2 mode: `env_node` publishes `env_data`; `gnc_node` or `qp_gnc_node` returns thrust via `actuation_cmd`. The bridge mode swaps in `bridge_node` from `external_sim_bridge` with a `fake` or `basilisk` backend — GNC stays the same.],
  internal-arch,
)

#speaker-note[One breath of context before returning to the live walkthrough.]

= Individual Responsibilities

== Cannon Whitney — Manager

- Coordinated milestones, advisor communication, and validation scheduling
- Drove integration decisions for the internal vs bridge paths
- Contributed to documentation, demo readiness, and report deliverables

#speaker-note[Cannon introduces himself and briefly states what he personally shipped this semester.]

== Dylan Long — SCRUM Master

- Ran sprint cadence, task board, and reviews on the GitHub repository
- Maintained devcontainers, deployment image, and repo automation
- Authored and refined user-facing documentation and demo guides

#speaker-note[Dylan speaks to process and deployment pieces he owned.]

== Caleb Jackson — Developer

- Built the ROS 2 package layout and node implementations (`env_node`, `gnc_node`, `qp_gnc_node`)
- Ported DLQR and integrated the QP-MPC controller path with QuadProg++
- Implemented the `external_sim_bridge` and its `fake` / `basilisk` backends

#speaker-note[Caleb highlights the core ROS 2 implementation work and the bridge.]

= Wrap-up

== Where We Land

- Closed-loop ROS 2 simulation on `main`, with DLQR and QP-MPC controllers
- External simulator bridge validated against a pluggable backend
- Two-machine demo reproducible via Tailscale or local LAN
- Remaining enhancement: telemetry / log buffer (Issue #25)

== Thank You

Questions and full report: see the documentation PDF and the ASTRO repository at *github.com/Accommodus/ASTRO*.

#speaker-note[End the recording cleanly; keep the final beat short.]
