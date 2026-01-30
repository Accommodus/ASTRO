#import "project_status_report_template.typ": *

#team-report(
  completed: [
    - basic dev env
    - initial project plan
  ],
  in-progress-list: (
    in-progress(
      due: datetime(month: 2, day: 3, year: 2026),
      notes: [We have already found a few],
      percent: [30%],
      person: [Caleb, Dylan],
      task: [Research similar projects],
    ),
    in-progress(
      due: datetime(month: 2, day: 6, year: 2026),
      notes: [Already have the structure],
      percent: [20%],
      person: [Cannon],
      task: [Set-up env],
    ),
    in-progress(
      due: datetime(month: 2, day: 7, year: 2026),
      notes: [Make branches and auto github insights],
      percent: [5%],
      person: [Dylan],
      task: [Set-up SCRUM / Agile],
    ),
  ),
  next-week-list: (
    next-week(
      task: [Break down tasks],
      person: [Cannon],
      notes: [],
    ),
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
  issue-history-list: (),
)
