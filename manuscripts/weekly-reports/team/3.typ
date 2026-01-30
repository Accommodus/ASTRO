#import "project_status_report_template.typ": *

#team-report(
  completed: [
    - dev env
    - basic setup
  ],
  in-progress-list: (
    in-progress(
      notes: [],
      percent: [],
      person: [],
      task: [Research similar projects],
    )
  ),
  next-week-list: (
    next-week(
      task: [],
      person: [],
      notes: [],
    )
  ),
  issue-history-list: (
    issue-history(
      issue: [],
      plan: [],
    )
  ),
)
