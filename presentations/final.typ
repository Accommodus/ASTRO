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

- Local run — `ros2 launch distributed_satellite_sim sim.launch.py` (DLQR) and `qp_mpc_launch.py` (QP-MPC)
- Two-machine demo — `demo/compose.tailscale.yaml` and `demo/compose.local.yaml`
- Test suite — `colcon test --packages-select distributed_satellite_sim external_sim_bridge`

#speaker-note[Cannon cuts to screen share immediately after this slide and runs the live demo. Keep each segment tight; don't narrate anything you are not actively doing on screen.]

== Architecture

#text-diagram(
  [Internal ROS 2 mode: `env_node` publishes `env_data`; `gnc_node` or `qp_gnc_node` returns thrust via `actuation_cmd`. The bridge mode swaps in `bridge_node` from `external_sim_bridge` with a `fake` or `basilisk` backend — GNC stays the same.],
  internal-arch,
)

#speaker-note[Use this slide immediately after the live demo as a short recap of what the audience just saw.]

= Individual Responsibilities

== Cannon Whitney — Manager

- Coordinated milestones, advisor communication, and overall integration planning
- Implemented the original `gnc_node` / DLQR control path and helped steer the internal-vs-bridge architecture
- Owned deployment/runtime integration, including the deployment image and the Tailscale / local-LAN demo setups

#speaker-note[Cannon introduces himself and briefly states the management and integration work he personally shipped this semester.]

== Dylan Long — SCRUM Master

- Ran sprint cadence, GitHub issue flow, and review coordination
- Built the initial ROS 2 package skeleton and `env_node`, then expanded testing and validation coverage
- Designed and implemented `external_sim_bridge`, including the `fake` / `basilisk` backends and their validation tests

#speaker-note[Dylan speaks to the process, validation, and bridge work he owned.]

== Caleb Jackson — Developer

- Integrated the QP-MPC controller path with QuadProg++
- Added `qp_gnc_node`, its launch/config support, and controller test coverage
- Contributed technical documentation and follow-on telemetry / logging work

#speaker-note[Caleb highlights the advanced-controller work and related technical contributions he owned.]

= Wrap-up

== Where We Land

- Closed-loop ROS 2 simulation on `main`, with DLQR and QP-MPC controllers
- External simulator bridge validated against a pluggable backend
- Two-machine demo reproducible via Tailscale or local LAN
- Remaining enhancement: telemetry / log buffer (Issue #25)

== Thank You

Questions and full report: see the documentation PDF and the ASTRO repository at *github.com/Accommodus/ASTRO*.

#speaker-note[End the recording cleanly; keep the final beat short.]
