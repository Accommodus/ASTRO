#import "@preview/fletcher:0.5.8" as fletcher: diagram, edge, node

// Color Palette
#let palette = (
  sim: (
    border: luma(39%),
    bg: luma(96%),
  ),
  ros2: (
    border: blue.darken(20%),
    bg: blue.lighten(95%),
    accent: blue.darken(30%),
  ),
  gnc: (
    border: teal.darken(20%),
    bg: teal.lighten(95%),
    accent: teal.darken(30%),
  ),
  ui: (
    border: purple.darken(20%),
    bg: purple.lighten(95%),
    accent: purple.darken(30%),
  ),
)

// Style Attribute Sets
#let diagram-attrs = (
  spacing: (3em, 1.5em),
  node-stroke: 0.06em,
  node-corner-radius: 0.25em,
  edge-stroke: 0.05em,
  mark-scale: 70%,
)

#let label-size = 1em
#let parallel-shift = 0.2em
#let group-inset = 1em

// Node Generators

/// Standard component node — white-filled box inheriting diagram stroke.
#let component(pos, body, name: none, ..rest) = {
  node(pos, body, name: name, fill: white, ..rest)
}

/// Group that creates member nodes at positions relative to `root`.
/// `members`: array of (relative-pos, body, name) arrays.
/// Title is placed in its own node at the top center of the group.
/// Pass `title-name` to give the title node a label for use in outer enclosures.
#let group(root, title, members, color: palette.sim, inset: group-inset, title-name: none, ..rest) = {
  let accent = color.at("accent", default: black)
  let rx = root.at(0)
  let ry = root.at(1)

  let names = ()
  let xs = ()
  let ys = ()

  for m in members {
    let rel = m.at(0)
    let ax = rx + rel.at(0)
    let ay = ry + rel.at(1)
    xs.push(ax)
    ys.push(ay)
    names.push(m.at(2))
    component((ax, ay), m.at(1), name: m.at(2))
  }

  let min-y = calc.min(..ys)
  let center-x = (calc.min(..xs) + calc.max(..xs)) / 2
  let title-pos = (center-x, min-y - 1)

  // Title node (unstyled, just text)
  node(title-pos, text(fill: accent, weight: "bold", title),
    name: title-name, stroke: none, fill: none)

  // Enclosure around members + title
  node(
    enclose: (..names, title-pos),
    stroke: 0.09em + color.border,
    fill: color.bg,
    inset: inset,
    snap: -1,
    ..rest,
  )
}

/// Enclosure-only wrapper for nesting around already-created nodes.
#let wrap(title, members, color: palette.ros2, inset: group-inset, ..rest) = {
  let accent = color.at("accent", default: black)
  node(
    align(top + left, text(fill: accent, weight: "bold", title)),
    enclose: members,
    stroke: 0.09em + color.border,
    fill: color.bg,
    inset: inset,
    snap: -1,
    ..rest,
  )
}

/// Edge with a scaled-down label.
#let labeled-edge(from, to, marks, body, ..rest) = {
  edge(from, to, marks, label: text(size: label-size, body), ..rest)
}

// External Architecture (Sim ↔ Bridge ↔ ROS2)

#let external-arch = diagram(
  ..diagram-attrs,
  {
    // Simulation Environment
    group((0, 0), [Simulation Environment], (
      ((0, 0), align(center)[Orbital Mechanics / \
        Attitude Dynamics], <orbmech>),
      ((0, 3), align(center)[Actuator Model \
        (thruster / torque)], <actuator>),
    ))
    edge(<actuator>, <orbmech>, "-|>")

    // Sim Bridge / Adapter
    component((1.5, 1.5), [Sim Bridge / Adapter], name: <adapter>)

    // ROS2
    group((6, 0), [ROS2], (
      ((0, 1.5), [ROS2 Interface], <bus>),
      ((1.5, 0), [Sensor Simulator], <sens>),
      ((1.5, 1), [Navigation Module], <nav>),
      ((1.5, 2), [Guidance Module], <guid>),
      ((1.5, 3), [Control Module], <ctrl>),
    ), color: palette.ros2)

    // Bus ↔ Modules
    edge(<bus>, <sens>, "<->")
    edge(<bus>, <nav>, "<->")
    edge(<bus>, <guid>, "<->")
    edge(<bus>, <ctrl>, "<->")

    // Sim ↔ Adapter
    labeled-edge(<orbmech>, <adapter>, "-|>", [Ground Truth / Sim State])
    labeled-edge(<adapter>, <actuator>, "-|>", [Actuator Cmds])

    // Adapter ↔ Bus (parallel)
    labeled-edge(<adapter>, <bus>, "-|>",
      [Ground truth (ROS msg)],
      shift: parallel-shift, label-side: right)
    labeled-edge(<bus>, <adapter>, "-|>",
      [Control Cmds (ROS msg)],
      shift: parallel-shift, label-side: right)
  },
)

// Internal Architecture (All-in-ROS2)

#let internal-arch = diagram(
  ..diagram-attrs,
  {
    // Env Node
    group((0, 0), [Env Node], (
      ((0, 0), align(center)[Orbital Mechanics / \
        Attitude Dynamics], <env-dyn>),
      ((0, 2), align(center)[Actuator Model \
        (thruster / torque)], <env-act>),
    ), title-name: <env-title>)
    edge(<env-act>, <env-dyn>, "-|>")

    // GNC Node
    group((4, 0), [GNC Node], (
      ((0, 0), [Navigation], <gnc-nav>),
      ((0, 1), [Guidance], <gnc-guid>),
      ((0, 2), [Control], <gnc-ctrl>),
    ), color: palette.gnc, title-name: <gnc-title>)
    edge(<gnc-nav>, <gnc-guid>, "-|>")
    edge(<gnc-guid>, <gnc-ctrl>, "-|>")

    // Inter-node communication
    labeled-edge(<env-dyn>, <gnc-nav>, "-|>",
      [`env_data` Topic])
    labeled-edge(<gnc-ctrl>, <env-act>, "-|>",
      [`actuation_cmd` Service])
  },
)

// User Interface Design

#let ui-arch = diagram(
  ..diagram-attrs,
  spacing: (2.5em, 1.8em),
  {
    // Core ROS2 System
    group((1, 0), [ROS2 Backend], (
      ((0, 0), [Topics & Services], <topics>),
      ((0, 1), [ROS2 Core], <ros2-core>),
      ((0, 2), [Logging System], <logging>),
    ), color: palette.ros2)
    edge(<ros2-core>, <topics>, "<->")
    edge(<ros2-core>, <logging>, "<->")

    // User Interface Layer
    group((6, 0), [User Interfaces], (
      ((0, 0), [CLI Tools], <cli>),
      ((0, 1), [Plotting / Viz], <viz>),
      ((0, 2), [Web Dashboard], <web>),
      ((0, 3), [Log Viewer], <logview>),
    ), color: palette.ui)

    // Connections
    labeled-edge(<topics>, <cli>, "->", [`ros2 topic`])
    labeled-edge(<ros2-core>, <web>, "->", [WebSocket / REST])
    labeled-edge(<logging>, <logview>, "->", [Log Stream])
    labeled-edge(<topics>, <viz>, "->", [Real-time Data])
  },
)

