#import "project_status_report_template.typ": *

#team-report(
  date: datetime(day: 10, month: 4, year: 2026),
  completed: [
    - Refresh next-phase project roadmap documentation (PR #27)
    - Align README with merged DLQR regression-test coverage and close issue #10 (PR #30)
    - Finalize reference-file import / header extraction path for issue #4 (PRs #28 and #29)
    - Improve Docker image build workflows for cross-compilation (PR #33 / issue #31)
    - Add Tailscale-based networking support for multi-machine ROS connectivity, including Docker Compose services and ROS discovery peer configuration (PR #34 / issue #32)
    - Complete senior showcase demo implementation and planning for a two-computer distributed demonstration; only the final presentation script remains (issue #26)
  ],
  in-progress-list: (
    in-progress(
      due: datetime(month: 4, day: 17, year: 2026),
      notes: [Dylan defined the generic bridge contract, scaffolded the package, and began a Basilisk backend on feat/24-validate-external-bridge.],
      percent: [45%],
      person: [Dylan],
      task: [Validate generic external simulator bridge against Basilisk (issue #24)],
    ),
    in-progress(
      due: datetime(month: 4, day: 17, year: 2026),
      notes: [Caleb reported major implementation progress: roughly 75% on env-node updates, added QP GNC node work, and added debug functionality; not yet fully merged.],
      percent: [75%],
      person: [Caleb],
      task: [Integrate QP_MPC controller path and env/debug updates (issue #6)],
    ),
    in-progress(
      due: datetime(month: 4, day: 17, year: 2026),
      notes: [Planning and scope definition are underway; intended to support logging / telemetry capture for demo and simulator integration work.],
      percent: [15%],
      person: [Cannon, Dylan, Caleb],
      task: [Add circular telemetry/log buffer node (issue #25)],
    ),
    in-progress(
      due: datetime(month: 4, day: 17, year: 2026),
      notes: [Distributed demo path, machine roles, networking approach, and runtime setup are effectively complete; remaining work is the short spoken demo script and rehearsal polish.],
      percent: [90%],
      person: [Cannon, Dylan, Caleb],
      task: [Senior showcase in-person demo readiness (issue #26)],
    ),
  ),
  next-week-list: (
    next-week(
      task: [Continue Basilisk bridge validation and test the external_sim_bridge package],
      person: [Dylan],
      notes: [Advance issue #24 from design/scaffolding toward runnable validation],
    ),
    next-week(
      task: [Continue QP GNC / env-node integration and merge-ready cleanup],
      person: [Caleb],
      notes: [Carry the env-node, debug, and QP controller work from branch progress into reviewable form],
    ),
    next-week(
      task: [Define circular buffer scope and connect it to demo / telemetry needs],
      person: [Cannon, Dylan, Caleb],
      notes: [Clarify architecture so issue #25 supports both showcase logging and longer-term integration work],
    ),
    next-week(
      task: [Finish the senior showcase demo script and rehearse the presentation flow],
      person: [Cannon, Dylan, Caleb],
      notes: [Use the completed two-machine setup and finalize speaker handoff, explanation, and fallback wording],
    ),
  ),
  issue-history-list: (
    issue-history(
      issue: [Reference source organization was unclear and the initial submodule approach was not ideal for the repo],
      plan: [Merged PRs #28 and #29 to bring the needed reference files directly into the repository and close issue #4 cleanly],
    ),
    issue-history(
      issue: [External simulator support and multi-machine demonstration require both software interfaces and reliable networking],
      plan: [Addressed the networking side with PRs #33 and #34 and completed the practical demo setup/planning; remaining work is bridge validation (#24), circular logging support (#25), and final demo scripting],
    ),
  ),
  next-meeting: datetime(day: 15, month: 4, year: 2026),
)

#image("Commits_13.png")