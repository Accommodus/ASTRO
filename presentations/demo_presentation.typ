#let astro-blue = rgb("#04364A")
#let astro-teal = rgb("#176B87")
#let astro-light = rgb("#EAF3F5")

#set page(
  width: 13.333in,
  height: 7.5in,
  margin: (x: 0.7in, y: 0.55in),
)

#set text(font: "Times New Roman", size: 21pt, fill: astro-blue)
#set par(justify: false, leading: 0.7em)

#let slide(title, body) = {
  rect(
    width: 100%,
    height: 100%,
    inset: 0pt,
    fill: white,
    [
      #align(left + top, text(size: 28pt, weight: "bold", fill: astro-blue)[#title])
      #v(0.16in)
      #line(length: 100%, stroke: 1.6pt + astro-teal)
      #v(0.28in)
      #body
    ],
  )
}

#let title-slide() = {
  align(center + horizon, [
    #v(1.2in)
    #text(size: 34pt, weight: "bold", fill: astro-blue)[ASTRO]
    #v(0.25in)
    #text(size: 22pt, fill: astro-teal)[Senior Showcase Demo]
    #v(0.5in)
    #text(size: 18pt)[Autonomous Satellite Test & Robotics Operations]
    #v(0.7in)
    #grid(
      columns: 3,
      column-gutter: 1.5in,
      [Cannon Whitney],
      [Dylan Long],
      [Caleb Jackson],
    )
  ])
}

#let callout(title, text-body) = rect(
  radius: 8pt,
  inset: 14pt,
  stroke: 1.2pt + astro-teal,
  fill: astro-light,
  [
    #text(weight: "bold", size: 18pt, fill: astro-blue)[#title]
    #v(0.08in)
    #text(size: 16pt, fill: astro-blue)[#text-body]
  ],
)

#title-slide()

#pagebreak()

#slide(
  [Why This Matters in the STAR Lab],
  [
    Satellite simulation and flight software in the STAR Lab often live on different machines. Today, those systems are frequently connected through ad hoc UDP links between a desktop simulator and Jetson-based flight hardware. That works for one narrow setup, but it is difficult to reuse, difficult to test as a common baseline, and difficult to extend across multiple lab projects.

    #v(0.28in)

    #grid(
      columns: 2,
      column-gutter: 0.35in,
      row-gutter: 0.24in,
      callout(
        [Current Problem],
        [Networking, control, and logging are bundled together inside monolithic executables.]
      ),
      callout(
        [ASTRO Improvement],
        [ROS 2 separates those responsibilities into reusable components with standard interfaces.]
      ),
      callout(
        [Current Problem],
        [Each simulator-to-controller connection becomes a custom integration job.]
      ),
      callout(
        [ASTRO Improvement],
        [A modular interface makes systems easier to swap, validate, and combine across the lab.]
      ),
    )
  ],
)

#pagebreak()

#slide(
  [Two-Laptop Demo Setup],
  [
    #grid(
      columns: (1fr, auto, 1fr),
      column-gutter: 0.35in,
      align: horizon,
      rect(
        radius: 8pt,
        inset: 18pt,
        stroke: 1.2pt + astro-teal,
        [
          #text(weight: "bold", size: 19pt)[Laptop 1]
          #v(0.08in)
          #text(size: 17pt)[Environment Node]
          #v(0.08in)
          #text(size: 15pt)[Publishes satellite state]
        ],
      ),
      align(center + horizon, [
        #text(size: 20pt, weight: "bold")[`env_data`]
        #v(0.12in)
        #text(size: 24pt)[→]
        #v(0.18in)
        #text(size: 20pt, weight: "bold")[`actuation_cmd`]
        #v(0.12in)
        #text(size: 24pt)[←]
      ]),
      rect(
        radius: 8pt,
        inset: 18pt,
        stroke: 1.2pt + astro-teal,
        [
          #text(weight: "bold", size: 19pt)[Laptop 2]
          #v(0.08in)
          #text(size: 17pt)[GNC Node]
          #v(0.08in)
          #text(size: 15pt)[Computes DLQR command]
        ],
      ),
    )

    #v(0.42in)

    #text(size: 18pt)[In this demo, the environment and control roles are running on separate computers. The environment node publishes the six-state telemetry, and the GNC node receives that state, computes the control action, and sends the actuation command back.]
  ],
)

#pagebreak()

#slide(
  [Why This Demo Matters],
  [
    #text(size: 20pt)[This is not just a mockup of two terminals talking. It shows the project in a near-final form: a distributed ROS 2 control loop, split across two machines, using the same modular interfaces the lab can build on later.]

    #v(0.34in)

    #rect(
      radius: 8pt,
      inset: 18pt,
      stroke: 1.2pt + astro-teal,
      fill: astro-light,
      [
        #text(size: 20pt, weight: "bold")[Validated Baseline]
        #v(0.12in)
        #text(size: 18pt)[The demonstrated system already includes environment-node tests, GNC-node tests, and a regression check against the original reference trajectory.]
      ],
    )
  ],
)

#pagebreak()

#slide(
  [Current Status],
  [
    #text(size: 20pt)[ASTRO already demonstrates the core idea: a reusable ROS 2 path for satellite simulation and control, running in a distributed setup instead of a one-off direct coupling.]

    #v(0.3in)

    #grid(
      columns: 2,
      column-gutter: 0.35in,
      callout(
        [Working Today],
        [Distributed Env and GNC demo, ROS 2 interfaces, and a validated DLQR baseline.]
      ),
      callout(
        [Ongoing Work],
        [Validation of the broader external simulator bridge path, telemetry improvements, and additional controller support such as QP_MPC.]
      ),
    )
  ],
)
