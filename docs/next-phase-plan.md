# ASTRO Next-Phase Planning

## Purpose

This is a living planning note for the ASTRO repository.

Use it to:

- understand what is currently implemented on `main`
- separate current-phase closeout work from true next-phase work
- map the broader manuscript/proposal goals onto the live GitHub tracker
- give new contributors enough context to pick up a task without reconstructing the whole project history

This document should be treated as a roadmap snapshot, not a historical status report.

## Current Snapshot

### Intended project scope

From the proposal and manuscript branch, ASTRO is meant to provide:

- a modular ROS 2 simulation and flight-software framework
- support for both fully internal ROS 2 execution and external simulator bridging
- a path toward multiple controllers, operator tooling, and eventual hardware-in-the-loop workflows

### What is implemented on `main`

`main` now contains a validated DLQR-oriented ROS 2 baseline for `distributed_satellite_sim`:

- `env_node` publishes the simulated six-state system trajectory
- `gnc_node` computes DLQR control and sends thrust back through `actuation_cmd`
- `sim.launch.py` launches the closed-loop pair together
- environment-node unit tests are merged
- GNC-node unit tests are merged
- a launch-based reference regression test is merged
- a deployment image exists for running one node role per machine with `ROLE=ENV` or `ROLE=GNC`

### Important current limitations

The implementation is still intentionally narrow:

- the environment dynamics, timing, and initial state are hardcoded
- the controller path is hardcoded to the DLQR scenario
- there is no generic external simulator bridge implementation yet
- there is no operator-facing dashboard or telemetry tooling yet
- there is no package CI workflow that enforces build/test validation in GitHub Actions

### Important repo drift

Some repo docs still lag behind the code:

- `README.md` still says there is no committed end-to-end regression test, but `origin/main` now includes one
- `README.md` also still mentions placeholder license metadata, but `package.xml` on `main` is now `Apache-2.0`

This file is intended to be the more current planning source until those docs are refreshed.

## Issue Status

### Closed and landed

#### Issue #3: `feat: implement ROS2 GNC Node from DLQR reference`

Status: complete and merged.

What it delivered:

- the ROS 2 GNC node for the DLQR path
- the current `env_data` -> `gnc_node` -> `actuation_cmd` control loop

#### Issue #8: `test: GNC Node unit tests`

Status: complete and merged.

What it delivered:

- isolated GNC-node test coverage for the DLQR control law

#### Issue #9: `test: Env Node unit tests`

Status: complete and merged.

What it delivered:

- environment-node tests for propagation, service behavior, and message publication

#### Issue #13: `infra: Reset devcontainer setup back to OS-specific version`

Status: complete and merged.

What it delivered:

- the current OS-specific devcontainer layout

#### Issue #18: `feat: CI/CD deployment image for distributed_satellite_sim`

Status: complete and merged.

What it delivered:

- a standalone deployment image for `distributed_satellite_sim`
- a role-aware entrypoint that can start `env_node` or `gnc_node`
- a GitHub Actions workflow that publishes the image

### Current-phase closeout work

These issues are the most important remaining work if the team wants to finish the current DLQR phase cleanly.

#### Issue #10: `test: ROS 2 package output vs original reference comparison`

Status: implemented on `main`, but still open in the tracker.

What already exists:

- a committed 91-step reference trajectory fixture
- a merged `launch_testing` regression that exercises `sim.launch.py`
- a documented numeric tolerance in the test itself
- a `min_subscribers` startup barrier to avoid subscriber discovery races in the regression path

What still needs to happen:

- re-run and record the validation evidence in the accepted environment
- update stale repo docs that still claim this regression test does not exist
- close the issue with explicit evidence

#### Issue #11: `test: complete DLQR testing suite`

Status: partially blocked by closeout wording rather than missing code.

What already exists:

- Issue `#8` merged
- Issue `#9` merged
- Issue `#10` appears implemented on `main`

What still needs to happen:

- finish the closeout of Issue `#10`
- decide whether the current acceptance language should require real package CI
- either add package CI or relax/rewrite the "devcontainer CI environment" wording

#### Issue #5: `test: add integration tests with launch_testing`

Status: partly satisfied by merged work, but still open because its scope is broader than Issue `#10`.

What already exists:

- the merged launch-based regression path from Issue `#10`

What still needs to happen:

- decide whether the merged regression test fully satisfies the automation part of the issue
- decide whether the remaining value in this issue is now the manual/distributed verification on separate devices
- if so, document that explicitly instead of leaving the issue half-overlapping with `#10`

### True next-phase work already tracked

#### Issue #4: `chore: extract final set of C++ header files`

Status: open and still a prerequisite for the QP_MPC path.

Current value:

- confirms and relocates the final QP_MPC matrix/header assets

#### Issue #6: `feat: integrate QP_MPC controller as alternative GNC mode`

Status: open and still large.

Current value:

- milestone-2 controller expansion after the DLQR baseline is fully closed out

Important dependency:

- this still depends on the QP_MPC reference assets and likely on environment parameterization

#### Issue #24: `feat: validate generic external simulator bridge against Basilisk`

Status: open and well-framed.

Current value:

- defines the generic external adapter contract
- uses Basilisk as the first validation target, not as the architecture itself

#### Issue #25: `feat: add circular telemetry/log buffer node`

Status: open and well-framed.

Current value:

- first concrete step toward operator tooling, telemetry/FDC support, and queryable recent history

#### Issue #26: `prep: senior showcase in-person demo readiness`

Status: open and near-term.

Current value:

- prepares the two-machine showcase demo
- includes runtime choice, networking, fallback planning, and team member roles

## Immediate Priorities

If the team wants the cleanest forward path, the current order of work should be:

1. finish and close Issue `#10`
2. resolve the acceptance mismatch in Issue `#11`
3. clarify the remaining scope of Issue `#5`
4. prepare the Senior Showcase demo in Issue `#26`
5. begin either Issue `#24` or Issue `#25` as the first true next-phase engineering task

## Next-Phase Themes

The manuscript branch still points to three strong directions after DLQR closeout:

1. controller extensibility
2. higher-fidelity ROS-native modeling
3. operator tooling / telemetry / fault handling

Given the current repo state, the most coherent order is:

1. stabilize and close the validated DLQR baseline
2. prepare the distributed showcase demo
3. define the generic external simulator adapter contract
4. build the first operator-tooling primitives
5. generalize the environment/controller architecture for alternate backends
6. expand into QP_MPC and later higher-fidelity/HIL work

## Already Tracked For Next Phase

The following next-phase areas already exist in the tracker and should not be re-opened as duplicate issues:

- generic external simulator bridge validation: Issue `#24`
- circular telemetry/log buffer node: Issue `#25`
- senior showcase demo readiness: Issue `#26`
- QP_MPC controller path: Issue `#6`
- QP_MPC reference/header extraction: Issue `#4`

## Still Worth Opening Later

The following items are still useful and are not yet captured as separate issues in a clear way.

### 1. `docs: write architecture and onboarding baseline for ASTRO`

Why:

- new contributors still have to synthesize the code, README, manuscript branch, and issue tracker themselves

Definition of done:

- one durable architecture note for the current ASTRO baseline
- a small glossary for `env_data`, `actuation_cmd`, DLQR, QP_MPC, external bridge, and internal ROS path
- a contributor-oriented map of current-phase closeout work versus next-phase work

### 2. `infra: add package CI workflow for build, lint, and tests`

Why:

- Issue `#11` already assumes CI-backed validation, but the repo does not currently enforce that

Definition of done:

- pull requests run package build/test checks
- the workflow reports unit-test and launch-regression results
- Issue `#11` can point to a real CI artifact instead of implied validation

### 3. `feat: parameterize EnvNode dynamics, timing, and initial conditions`

Why:

- this is the main enabling step for alternate controllers and richer scenarios

Definition of done:

- default DLQR behavior remains reproducible
- alternate configs can be loaded without code changes
- parameter-loading behavior is tested

### 4. `feat: define a controller interface for selectable GNC backends`

Why:

- the manuscripts repeatedly call for a universal/selectable controller path

Definition of done:

- a documented ROS-side controller contract exists
- the DLQR implementation fits that contract
- future controllers can be swapped without rewriting the environment wiring

### 5. `test: add regression support for multiple configurable scenarios`

Why:

- once parameters and multiple controller backends exist, the current single DLQR fixture/test will not be enough

Definition of done:

- the current DLQR regression still passes
- a second scenario can be added without rewriting the harness
- fixture provenance and tolerance policy are documented per scenario

### 6. `feat: define hardware-in-the-loop integration plan`

Why:

- HIL is still a stated project goal, but it is not yet broken down into executable work

Definition of done:

- one planning document with topology, dependencies, risks, and prerequisites
- follow-on implementation issues can be created from it

## Recommended Sequence

To keep the project coherent, use this order:

1. close out the DLQR validation milestone
2. prepare and rehearse the distributed showcase demo
3. add package CI so validation is enforceable
4. define the generic external simulator adapter contract
5. add the first telemetry/operator-tooling primitive
6. parameterize the environment and define a controller abstraction
7. move into QP_MPC and later higher-fidelity / HIL work

This ordering keeps the team from stacking new architecture and UI work on top of a baseline that is still only partially closed out.
