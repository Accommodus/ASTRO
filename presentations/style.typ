#let text-diagram(text, diagram) = {
  let scale-to-container(body) = layout(ly => {
    let body-size = measure(body)

    let base = (
      x: ly.width / body-size.width * 100%,
      y: ly.height / body-size.height * 100%,
    )

    if base.y > base.x {base.y = auto} else {base.x = auto}

    scale(
      x: base.x,
      y: base.y,
      reflow: true,
      body,
    )
  })

  grid(
    columns: 1fr,
    rows: (auto, 1fr),
    row-gutter: 1em,
    text,
    align(center + horizon, scale-to-container(diagram))
  )
}

#let presentation(body) = {
  import "@preview/numbly:0.1.0": numbly
  import "@preview/fletcher:0.5.8": *
  import "@preview/touying:0.6.3": *
  import themes.university: *

  show: university-theme.with(
    aspect-ratio: "16-9",
    config-info(
      title: [ASTRO],
      subtitle: [Autonomous Satellite Test & Robotics Operations],
      authors: ([Cannon Whitney], [Dylan Long], [Caleb Jackson]),
      date: datetime.today(),
      logo: none,
    ),
    config-colors(
      primary: rgb("#04364A"),
      secondary: rgb("#176B87"),
      tertiary: rgb("#448C95"),
    ),
  )

  set heading(numbering: numbly("{1}.", default: "1.1"))
  set table(stroke: 0.5pt, inset: 0.6em)

  title-slide()
  body
}
