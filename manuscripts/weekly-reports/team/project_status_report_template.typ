#let date-format = "[Month repr:long] [day padding:none], [year]"

#let in-progress(
  task: [],
  percent: [],
  due: datetime.today() + duration(days: 7),
  person: [],
  notes: []
) = (
  task,
  percent,
  due.display(date-format),
  person,
  notes
)

#let next-week(
  task: [],
  person: [],
  notes: []
) = (
  task,
  person,
  notes
)

#let issue-history(
  issue: [],
  plan: []
) = (
  issue,
  plan
)

#let team-report(
  date: datetime.today(),
  team-name: [ROS2],
  prepared-for: [Dr. Chrispy],
  completed: [],
  in-progress-list: (),
  next-week-list: (),
  issue-history-list: (),
  advisor-meeting: [Yes],
  next-meeting: datetime.today() + duration(days: 7)
) = {
  let colors = (
    accent: rgb("1E2651"),
    highlight: rgb("2E69FF"),
    lowlight: rgb("595959"),
    header: white,
    text: black
  )

  let header-font = "Calibri"
  let input-font = "Cambria"

  let inset-amount = 0.65em

  let process-args(func: auto, data) = {
    if func == auto {return data}

    let data = if type(data) != array {(data,)} else {data}
    let data = data.map(it => {
      if type(it) == dictionary {return func(..it)} else {return it}
    })

    return data.flatten()
  }

  let summary_table(
    lines: true,
    func: auto,
    columns,
    headers,
    data
  ) = table(
    inset: inset-amount,
    stroke: if lines {(bottom:0.5pt + colors.accent, rest: none)} else {none},
    columns: columns,
    table.header(
      ..headers.map(it => {
        set text(font: header-font, fill: colors.accent, size: 10pt, weight: "light")
        upper(it)
      })
    ),
    ..process-args(func: func, data)
  )

  set text(font: input-font, fill: colors.text, size: 10pt, weight: "light")

  show title: it => {
    set text(font: header-font, fill: colors.accent, size: 24pt, weight: "light")
    upper(it)
  }

  show heading: it => {
    set text(font: header-font, fill: colors.header, size: 11pt, weight: "light")
    block(width: 100%, inset: inset-amount, fill: colors.accent, upper(it))
  }

  [
    #title[Project Status Report]

    = Project Summary
    #summary_table(
      lines: false,
      (1fr, 1fr, 1fr),
      ([Report Date], [Team Name], [Prepared For]),
      (date.display(date-format), team-name, prepared-for)
    )

    = Status Summary/Completed Activities
    #block(inset: inset-amount, completed)

    = Activities in Progress
    #summary_table(
      func: in-progress,
      (1fr, 0.5fr, 1fr, 1.25fr, 1fr),
      ([Task], [% Done], [Due Date], [Person Responsible], [Notes]),
      in-progress-list
    )

    = Activities Planned for Next Week
    #summary_table(
      func: next-week,
      (1fr, 1fr, 2fr),
      ([Tasks], [Person Responsible], [Notes]),
      next-week-list
    )

    = Risk and issue history
    #summary_table(
      func: issue-history,
      (1fr, 1fr),
      ([Issue], [Mitigation Plan]),
      issue-history-list
    )

    #v(1em)
    #block(
      width: 100%,
      stroke: 0.5pt + colors.accent,
      inset: inset-amount,
      {
        set text(font: header-font, fill: colors.text, size: 11pt, weight: "light")
        show text: upper

        highlight()[
          DID YOUR TEAM MEET WITH YOUR FACULTY ADVISOR THIS WEEK? #advisor-meeting \
          Next Scheduled Meeting with your advisor: #next-meeting.display(date-format)
        ]
      }
    )
  ]
}
