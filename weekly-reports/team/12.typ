#import "project_status_report_template.typ": *

#team-report(
  date: datetime(day: 3, month: 4, year: 2026),
  completed: [
    - Begin work on Basilisk compatibility (Issue #24)
    - Complete connectivity testing between machines
    - Begin Circular Memory Buffer (Issue #24)
  ],
  in-progress-list: (
    in-progress(
      due: datetime(month: 4, day: 10, year: 2026),
      notes: [See issues #26 and #25 for further info],
      percent: [10%],
      person: [Cannon, Dylan, Caleb],
      task: [Implement Circular Memory Buffer & Basilisk Compatibility],
    ),
    in-progress(
      due: datetime(month: 4, day: 10, year: 2026),
      notes: [Closes when #10 is done],
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
    next-week(
      task: [Basilisk functionality and circular buffer, Issues #25 and #26],
      person: [Cannon, Dylan, Caleb],
      notes: [See issues #25 and #26 for info],
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
  next-meeting: datetime(day: 8, month: 4, year: 2026),
)

#image("Commits_12.png")
