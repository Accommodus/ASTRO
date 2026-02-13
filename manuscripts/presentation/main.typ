#import "@preview/touying:0.6.1": *
#import themes.university: *
#import "@preview/numbly:0.1.0": numbly
#import "@preview/fletcher:0.5.8" as fletcher: diagram, edge, node
#import "diagrams.typ": external-arch, internal-arch, ui-arch

#set table(stroke: 0.5pt, inset: 0.6em)

#let fletcher-diagram = touying-reducer.with(
  reduce: fletcher.diagram,
  cover: fletcher.hide,
)

#show: university-theme.with(
  aspect-ratio: "16-9",
  config-info(
    title: [ASTRO],
    subtitle: [Autonomous Satellite Test & Robotics Operations],
    author: [Cannon Whitney, Dylan Long, Caleb Jackson],
    date: datetime.today(),
    logo: none,
  ),
  config-colors(
    primary: rgb("#04364A"),
    secondary: rgb("#176B87"),
    tertiary: rgb("#448C95"),
  ),
)

#set heading(numbering: numbly("{1}.", default: "1.1"))

#title-slide(authors: ([Cannon Whitney], [Dylan Long], [Caleb Jackson]))

= Project Overview

== Motivation

The STAR laboratory at UF relies on ad hoc UDP client-server architectures to connect simulations with flight software. This limits:

- *Interoperability* — Projects cannot easily work together
- *Modularity* — Logging, calculations, and networking are bundled into monolithic executables
- *Scalability* — Simulations cannot serve multiple systems simultaneously

== Goals

+ *Interoperate* — Enable seamless communication between simulation desktops and Jetson edge hardware through ROS 2 nodes

+ *Modularize* — Package existing lab software (orbital mechanics, GNC, logging) into discrete ROS 2 components

+ *Extend* — Build a framework that connects to _any_ simulation, not just Basilisk, expanding the lab's testing and operational capabilities

= Project Design

== Software Architecture — External Sim

When the simulation environment cannot be wrapped as a ROS 2 node, a bridge adapter translates between raw simulation data and ROS messages.

#align(center, scale(50%, external-arch))

== Software Architecture — Internal ROS 2

When the simulation _can_ be wrapped in ROS 2, all components live inside the ROS 2 graph as native nodes.

#align(center, scale(60%, internal-arch))

== ROS 2 Component Mapping

#table(
  columns: (auto, 1fr),
  align: (left, left),
  table.header([*Component*], [*Role*]),
  [Env Node], [Executes simulation software (orbital mechanics, actuator models)],
  [GNC Node], [Navigation, Guidance, and Control — ported from DLQR on NJON],
  [`env_data` Topic], [High-frequency telemetry / state from Env to GNC],
  [`actuation_cmd` Service], [Control signals sent from GNC to the environment],
)

== User Interface Design

ROS 2 exposes system data through topics, services, and logging — accessible via CLI, dashboards, and plotting tools.

#align(center, scale(65%, ui-arch))

= Project Progress

== Milestones

#table(
  columns: (auto, 1fr, auto),
  align: (center, left, center),
  table.header([*\#*], [*Milestone*], [*Status*]),
  [1], [Complete integration with DLQR], [In Progress],
  [2], [Complete integration with QP\_MPC], [Planned],
  [3], [Add tasking layer], [Planned],
  [4], [Integrate UI], [Planned],
)

== Current Sprint

- Laying out the ROS 2 package structure
- Mapping existing hardware topology (Desktop + NJON) to ROS 2 nodes
- Wrapping DLQR functionality into Env and GNC nodes
- Removing ad hoc networking; replacing with ROS 2 topics and services

= Individual Responsibilities

== Cannon — Project Manager

- Extract final set of C++ header files for shared types
- Coordinate architecture decisions and team schedule
- Oversee integration testing between nodes

== Dylan — SCRUM Master

- Build the ROS 2 component skeleton (package layout, launch files)
- Populate Env Node from `udp_hcw_discrete_txrx` reference code
  - Remove networking layer
  - Replace logging with ROS 2 logging

== Caleb — Backend Developer

- Populate Control Node from `udp_roundtrip_discrete` reference code
  - Remove networking layer
  - Replace logging with ROS 2 logging
- Verify outputs against the original standalone executable
