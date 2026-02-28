#import "project_status_report_template.typ": *

#team-report(
  completed: [
  ],
  in-progress-list: (
    in-progress(
      due: datetime(month: 2, day: 28, year: 2026),
      notes: [Based on files in reference],
      percent: [90%],
      person: [Dylan, Cannon, Caleb],
      task: [Convert first two C++ algs. to ROS2 package],
    ),
    in-progress(
      due: datetime(month: 3, day: 10, year: 2026),
      notes: [Based on files in reference],
      percent: [90%],
      person: [Dylan, Cannon, Caleb],
      task: [Convert MCP C++ to ROS2 package],
    ),
  ),
  next-week-list: (
    next-week(
      task: [Populate Control Node],
      person: [Caleb],
      notes: [Remove networking & replace logging],
    ),
    next-week(
      task: [Add further ROS2 Components],
      person: [Cannon, Dylan, Caleb],
      notes: [See TODO for specifics],
    ),
  ),
  issue-history-list: (
    issue-history(
      issue: [Difficult to find all C++ files for MPC-Sim],
      plan: [Inspect NJON + Desktop; Talk to C&C team at STAR lab],
    )
  ),
  next-meeting: datetime(day: 24, month: 2, year: 2026),
)

#image("Commits_7.png")