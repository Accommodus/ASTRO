#import "style.typ": presentation, text-diagram
#import "diagrams.typ": external-arch, internal-arch
#import "@preview/fletcher:0.5.8": *

#show: presentation

= Progress Update

== Requirements, Backlog, and Project Plan

From our proposal and backlog, we focused this phase on finalizing core implementation and starting validation.

#table(
  columns: 3,
  table.header([*Backlog / Milestone Item*], [*Status*], [*Evidence / Next Step*]),
  [Modular simulation + flight software engines], [In Progress], [ROS 2 Env + GNC node path implemented and integrated],
  [Bridge across machines],
  [In Progress],
  [Graph operational; CI/CD deployment image issue remains open (#18)],

  [Software validation],
  [In Progress],
  [Env tests merged; PR #20 and PR #21 implemented and awaiting review; final validation closure remains under #11],

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
  [PR #20], [GNC node unit tests implemented], [Open for Review],
  [PR #21], [ROS 2 vs reference trajectory launch test implemented], [Open for Review],
  [Issues #8, #9], [Unit test tracks for GNC and Env], [Closed / Implemented],
  [Issues #10, #11], [Reference comparison and DLQR validation closure], [In Review / Open],
)

= Project Walk-Through

== External Sim

#text-diagram(
  [When the simulation environment cannot be wrapped as a ROS 2 node, a bridge adapter translates simulation data and ROS messages.],
  external-arch,
)

== Internal ROS 2

#text-diagram(
  [When the simulation _can_ be wrapped in ROS 2, all components live inside the ROS 2 graph as native nodes.],
  internal-arch,
)

== ROS 2 Component Mapping

#table(
  columns: 2,
  table.header([*Component*], [*Role*]),
  [Env Node], [Executes simulation software (orbital mechanics, actuator models)],
  [GNC Node], [Navigation, Guidance, and Control — ported from DLQR on NJON],
  [`env_data` Topic], [High-frequency telemetry / state from Env to GNC],
  [`actuation_cmd` Service], [Control signals sent from GNC to the environment],
)

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

== Validation and End-to-End Demo

Current closed-loop validation path now exercises the real launch flow:

- Our demo is the ROS 2 closed-loop system running end to end: launching Env and GNC together, producing the control trajectory, and showing that it matches the expected reference behavior
- `sim.launch.py` starts the Env and GNC nodes together
- PR #20 adds the GNC unit-test layer and exposes the control node for test coverage
- PR #21 records the ROS 2 closed-loop trajectory and compares it to a committed reference fixture
- Together, these changes turn the demo from architecture-only into an executable validation path pending final teammate review

#align(center, scale(75%, diagram(
  node((0, 0), [Reference Trajectory CSV]),
  node((2, 0), [launch_testing Recorder]),
  node((2, 1.5), [sim.launch.py]),
  node((4, 1.5), [Env + GNC Nodes]),
  edge((2, 1.5), (4, 1.5), "->", label: [launch]),
  edge((4, 1.5), (2, 0), "->", label: [env_data trace]),
  edge((0, 0), (2, 0), "->", label: [expected trajectory]),
)))

== Implementation Status

#table(
  columns: 2,
  table.header([*Implemented*], [*Planned / Not Yet Programmed*]),
  [
    - Env and GNC ROS 2 nodes
    - Core DLQR control path
    - Env unit tests merged into main
    - GNC unit tests + trajectory comparison implemented in open review
    - CI issue/branch automation workflow
  ],
  [
    - Final review and merge of PR #20 and PR #21
    - Validation milestone closure under issue #11
    - Universal GNC interface / selectable controller path (#6)
    - Future deployment image for distributed sim (#18)
  ],
)

= Test Plan

== Completed Testing and Validation Work

Completed this cycle:

- Env node unit tests merged, covering initialization, thrust behavior, service behavior, and topic publication
- GNC unit-test work implemented and pending final review
- ROS 2 vs reference trajectory comparison implemented and pending final review
- Build/dev environment stabilized across Linux/macOS/Windows devcontainer workflows

== Validation In Progress and Exit Criteria

Current test plan for software validation:

- *Merged test layer*: Env node unit tests are already merged into `main`
- *Implemented / open-review test layer*: GNC unit tests and launch-based trajectory comparison are both implemented and awaiting review
- *Validation umbrella*: closes once that test work is reviewed, merged, and accepted as the reproducible DLQR baseline
- *Integration checks*: launch-level and eventual multi-machine verification remain the broader next validation layer

== Issues and Strategies

#table(
  columns: 3,
  table.header([*Issue*], [*Strategy*], [*Status*]),
  [CMake merge conflict while parallel feature work],
  [Merge `main` into feature branch early; preserve both env and gnc targets during conflict resolution],
  [Mitigated],

  [Reference code spread across multiple legacy sources],
  [Use the latest tracked source set and wrap it in ROS 2 first; refine against newer upstream revisions later],
  [Managed],

  [Cross-platform devcontainer instability],
  [Adopt OS-specific/runtime-safe rosdep flow and update workflows],
  [Mitigated],

  [Open validation PRs still awaiting final teammate review],
  [Present them as implemented validation work now and close review/merge promptly after team signoff],
  [In Progress],
)

== Possible Next Steps After Validation

- *Validation + universal GNC interface*: finish DLQR validation and define a clean selectable controller interface
- *Higher-fidelity rosified models*: port more of the existing system into ROS 2 with richer state inputs and dynamics
- *Operator tooling + telemetry/FDC*: add monitoring, fault response logic, and queryable circular-buffer logging

= Individual Responsibilities

== Dylan — SCRUM Master

- Completed: Env node and testing support, workflow/process automation, repository operations
- Next: shepherd PR #21 review/merge and help close the remaining DLQR validation milestone (#11)

== Caleb — Backend Developer

- Completed: GNC node implementation merged from DLQR reference path; GNC unit tests implemented in PR #20
- Next: finalize review of PR #20 and verify controller behavior against the accepted baseline output

== Cannon — Project Manager

- Completed: infrastructure support, CI/devcontainer reliability, validation coordination with STAR lab
- Next: drive closure of #11 and help the team choose the highest-impact post-validation direction
