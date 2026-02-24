#import "project_status_report_template.typ": *

#team-report(
  completed: [
    - presentation
    - architecture for first package
  ],
  in-progress-list: (
    in-progress(
      due: datetime(month: 2, day: 23, year: 2026),
      notes: [Make branches and auto github insights],
      percent: [70%],
      person: [Dylan],
      task: [Set-up SCRUM / Agile],
    ),
    in-progress(
      due: datetime(month: 2, day: 28, year: 2026),
      notes: [Based on files in reference],
      percent: [40%],
      person: [Dylan, Cannon, Caleb],
      task: [Convert C++ FSW algs. to ROS2 package],
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

#image("Commits_6.png")