#import "project_status_report_template.typ": *

#team-report(
  completed: [
    - presentation
    - architecture for first package
  ],
  in-progress-list: (
    in-progress(
      due: datetime(month: 3, day: 7, year: 2026),
      notes: [See Diagram and Issues],
      percent: [20%],
      person: [Caleb, Cannon],
      task: [Set-up ENV node],
    ),
    in-progress(
      due: datetime(month: 3, day: 7, year: 2026),
      notes: [Based on files in reference],
      percent: [90%],
      person: [Dylan, Cannon, Caleb],
      task: [Convert C++ FSW algs. to ROS2 package],
    ),
  ),
  next-week-list: (
    next-week(
      task: [Finalise Control Node],
      person: [Caleb],
      notes: [Remove networking & replace logging],
    ),
    next-week(
      task: [Add further ROS2 Components],
      person: [Cannon, Dylan, Caleb],
      notes: [See issues for specifics],
    ),
  ),
  issue-history-list: (
    issue-history(
      issue: [Difficult to find all C++ files for MPC-Sim],
      plan: [Inspect NJON + Desktop; Talk to C&C team at STAR lab],
    )
  ),
  next-meeting: datetime(day: 4, month: 3, year: 2026),
)

#image("Commits_7.png")
