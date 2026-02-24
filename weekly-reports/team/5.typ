#import "project_status_report_template.typ": *

#team-report(
  completed: [
    - presentation
    - architecture for first package
  ],
  in-progress-list: (
    in-progress(
      due: datetime(month: 2, day: 12, year: 2026),
      notes: [Make branches and auto github insights],
      percent: [40%],
      person: [Dylan],
      task: [Set-up SCRUM / Agile],
    ),
    in-progress(
      due: datetime(month: 2, day: 17, year: 2026),
      notes: [Based on files in reference],
      percent: [23%],
      person: [Dylan, Cannon, Caleb],
      task: [Convert C++ FSW algs. to ROS2 package],
    ),
  ),
  next-week-list: (
    next-week(
      task: [Write README],
      person: [Caleb],
      notes: [Document basic project goals and structure],
    ),
    next-week(
      task: [Create ICD],
      person: [Cannon, Dylan, Caleb],
      notes: [Read through STAR Lab repo],
    ),
  ),
  issue-history-list: (
    issue-history(
      issue: [Difficult to find all C++ files for MPC-Sim],
      plan: [Inspect NJON + Desktop; Talk to C&C team at STAR lab],
    )
  ),
  next-meeting: datetime(day: 18, month: 2, year: 2026),
)

#image("Commits_4.png")
