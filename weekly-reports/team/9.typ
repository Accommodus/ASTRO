#import "project_status_report_template.typ": *

#team-report(
  date: datetime(day: 13, month: 3, year: 2026),
  completed: [
    - Env Node unit tests (issue #9, PR #17): extracted EnvNode to header, added ament_cmake_gtest and five test cases (ZeroThrust, NonZeroThrust, DefaultInitialState, ServiceReturnsSuccess, TopicPublishesCorrectSize).
    - GNC Node implementation merged to main (issue #3, PR #16): DLQR control node with u = -K*x, actuation_cmd service, env_data subscription.
    - GNC Node unit tests (issue #8): completed on branch 8-test-gnc-node-unit-tests and merged.
    - Devcontainer fix (PR #14): rosdep update without sudo, sudo -E for rosdep install, restore build/install in .gitignore; OS-specific devcontainer configs (ROS2-linux, ROS2-macos, ROS2-Windows).
    - GitHub Actions (PR #15): issue-branch workflow and track-issues workflow.
    - Merge of main into feat/env-tests: resolved CMakeLists.txt conflict (env_node_lib + gnc_node both present).
  ],
  in-progress-list: (
    in-progress(
      due: datetime(month: 3, day: 14, year: 2026),
      notes: [Blocked on #8 and #9; both now merged. Prep or implementation.],
      percent: [10%],
      person: [Cannon, Dylan, Caleb],
      task: [ROS2 vs reference comparison (issue #10)],
    ),
    in-progress(
      due: datetime(month: 3, day: 14, year: 2026),
      notes: [Closes when #8, #9, #10 are done],
      percent: [75%],
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
    )
  ),
  next-meeting: datetime(day: 25, month: 3, year: 2026),
)

#image("Commits_9.png")
