#import "project_status_report_template.typ": *

#team-report(
  date: datetime(day: 13, month: 3, year: 2026),
  completed: [
    - Merge test environment with unit tests (PR #17)
    - Added README Document & Merged with Main (PR #19)
    - GNC Node Unit tests updated (PR #20)
    - ROS2 Reference Trajector Test for DLQR scenario with launch testing (PR #21)
  ],
  in-progress-list: (
    in-progress(
      due: datetime(month: 3, day: 14, year: 2026),
      notes: [Blocked on #8 and #9; both now merged. Prep or implementation.],
      percent: [60%],
      person: [Cannon, Dylan, Caleb],
      task: [ROS2 vs reference comparison (issue #10)],
    ),
    in-progress(
      due: datetime(month: 3, day: 14, year: 2026),
      notes: [Closes when #8, #9, #10 are done],
      percent: [90%],
      person: [Cannon, Dylan, Caleb],
      task: [Complete DLQR testing suite (issue #11)],
    ),
  ),
  next-week-list: (
    next-week(
      task: [Implement/finalise ROS2 vs reference comparison (#10)],
      person: [Cannon, Dylan, Caleb],
      notes: [launch_testing; compare trajectory to reference output],
    ),
    next-week(
      task: [Close DLQR suite meta-issue (#11)],
      person: [Cannon, Dylan, Caleb],
      notes: [After #10 is merged],
    ),
  ),
  issue-history-list: (
    issue-history(
      issue: [Merge conflict on feat/env-tests with main (CMakeLists.txt)],
      plan: [Merged main into feat/env-tests; resolved conflict to keep env_node_lib and gnc_node targets],
    ),
    issue-history(
      issue: [Difficult to find all C++ files for MPC-Sim],
      plan: [Inspect NJON + Desktop; Talk to C&C team at STAR lab],
    ),
  ),
  next-meeting: datetime(day: 25, month: 3, year: 2026),
)

#image("Commits_9.png")
