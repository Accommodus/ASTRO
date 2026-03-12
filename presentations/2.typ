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

= Progress Update

== Requirements, Backlog, and Project Plan

From our proposal and backlog, we focused this phase on finalizing core implementation and starting validation.

#table(
  columns: 3,
  table.header([*Backlog / Milestone Item*], [*Status*], [*Evidence / Next Step*]),
  [Modular simulation + flight software engines], [In Progress], [ROS 2 Env + GNC node path implemented and integrated],
  [Bridge across machines], [In Progress], [Current ROS 2 graph operational; CI/CD deployment image issue remains open (#18)],
  [Software validation], [In Progress], [Env tests (#9) and GNC tests (#8) closed; comparison + full suite still open (#10, #11)],
  [UI and operator tooling], [Planned], [CLI/logging path active; dashboard-level UI remains a next milestone],
  [Hardware-in-the-loop expansion], [Planned], [Deferred until validation closes and baseline behavior is stable],
)

== GitHub Progress Since Presentation 1

#table(
  columns: 3,
  table.header([*ID*], [*Change*], [*State*]),
  [PR #16], [GNC node implementation from DLQR reference], [Merged],
  [PR #17], [Env node unit tests added and integrated], [Merged],
  [PR #14], [Devcontainer rosdep and OS-specific setup fixes], [Merged],
  [PR #15], [Issue/branch tracking GitHub workflows], [Merged],
  [PR #19], [Project README documentation], [Merged],
  [PR #20], [GNC node unit tests], [Open],
  [Issues #8, #9], [Unit test tracks for GNC and Env], [Closed],
  [Issues #10, #11, #18], [Comparison suite, DLQR test completion, CI/CD image], [Open],
)

= Project Walk-Through

== External Sim

When the simulation environment cannot be wrapped as a ROS 2 node, a bridge adapter translates simulation data and ROS messages.

#align(center, scale(50%, external-arch))

== Internal ROS 2

When the simulation _can_ be wrapped in ROS 2, all components live inside the ROS 2 graph as native nodes.

#align(center, scale(60%, internal-arch))

== ROS 2 Component Mapping

#table(
  columns: 2,
  table.header([*Component*], [*Role*]),
  [Env Node], [Executes simulation software (orbital mechanics, actuator models)],
  [GNC Node], [Navigation, Guidance, and Control — ported from DLQR on NJON],
  [`env_data` Topic], [High-frequency telemetry / state from Env to GNC],
  [`actuation_cmd` Service], [Control signals sent from GNC to the environment],
)

== User Interface Design

ROS 2 exposes system data through topics, services, and logging — accessible via CLI, dashboards, and plotting tools.

#align(center, scale(65%, ui-arch))

== Functional Data and Control Path

Current implemented control loop:

- Env node computes and publishes high-rate state on `env_data`
- GNC node subscribes to `env_data` and computes DLQR command using `u = -K*x`
- GNC node serves actuation output through `actuation_cmd`
- This replaces prior ad hoc UDP coupling with ROS 2 interfaces

#align(center, scale(78%, diagram(
  node((0, 0), [Env Node]),
  node((2, 0), [GNC Node]),
  node((4, 0), [Actuation Output]),
  edge((0, 0), (2, 0), "->", label: [env_data]),
  edge((2, 0), (4, 0), "->", label: [actuation_cmd]),
)))

== Implementation Status

#table(
  columns: 2,
  table.header([*Implemented*], [*Planned / Not Yet Programmed*]),
  [
    - Env and GNC ROS 2 nodes
    - Core DLQR control path
    - Unit-test infrastructure in ROS 2 package
    - CI issue/branch automation workflow
  ],
  [
    - QP\_MPC controller integration (#6)
    - Full ROS2 vs reference output closure (#10)
    - Meta DLQR test suite closure (#11)
    - CI/CD deployment image for distributed sim (#18)
  ],
)

= Test Plan

== Completed Testing and Validation Work

Completed this cycle:

- Env node unit tests merged: `ZeroThrust`, `NonZeroThrust`, `DefaultInitialState`, `ServiceReturnsSuccess`, `TopicPublishesCorrectSize`
- GNC node implementation merged and unit-test track closed
- Build/dev environment stabilized across Linux/macOS/Windows devcontainer workflows

== Validation In Progress and Exit Criteria

Current test plan for software validation:

- *Output equivalence* (#10): ROS 2 package trajectory and control output must match reference implementation within defined tolerance
- *DLQR suite completion* (#11): close remaining test cases and regression checks in a single suite
- *Integration checks* (#5): launch-level tests to verify multi-node behavior in real startup flow
- *Definition of done for this phase*: issues #10 and #11 closed with reproducible results in CI

= Issues and Strategies

/* problem with this table
#table(
  columns: 3,
  table.header([*Issue*], [*Strategy*], [*Status*]),
  [CMake merge conflict while parallel feature work], [Merge `main` into feature branch early; preserve both env and gnc targets during conflict resolution], [Mitigated],
  [Hard to locate complete legacy C++ file set], [Cross-check NJON + Desktop sources and coordinate with STAR C&C team], [Ongoing],
  [Cross-platform devcontainer instability], [Adopt OS-specific/runtime-safe rosdep flow and update workflows], [Mitigated],
  [Lab hardware Docker/network constraints], [Collect diagnostics first, avoid disruptive host changes until coordinated], [Ongoing],
)
*/

= Individual Responsibilities

== Dylan — SCRUM Master

- Completed: Env node and testing support, workflow/process automation, repository operations
- Next: close open GNC test PR and support comparison-test closure (#10)

== Caleb — Backend Developer

- Completed: GNC node implementation merged from DLQR reference path
- Next: support QP\_MPC integration track and verify control behavior against baseline output

== Cannon — Project Manager

- Completed: infrastructure support, CI/devcontainer reliability, validation coordination with STAR lab
- Next: drive closure of #10 and #11 and prepare deployment-image path (#18)

= Near-Term Goals (Next 1-2 Weeks)

- Close PR #20 and finalize remaining validation issues (#10, #11)
- Produce a repeatable demo runbook for project functionality demonstration
- Begin implementation planning for QP\_MPC mode and deployment-image workflow
- Keep scope focused on validated, demo-ready behavior before adding UI/HIL expansions
