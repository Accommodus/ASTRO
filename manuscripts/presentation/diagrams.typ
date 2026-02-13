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

/// Group enclosure around named nodes.
/// `color` should be a palette entry with `border`, `bg`, and optional `accent`.
#let group(body, members, color: palette.sim, inset: group-inset, ..rest) = {
  let accent = color.at("accent", default: black)
  node(
    text(fill: accent, weight: "bold", body),
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

#let external-arch = diagram(
  ..diagram-attrs,
  {
    // Simulation Environment
    component((0, 0),
      align(center)[Orbital Mechanics / \
        Attitude Dynamics],
      name: <orbmech>)
    component((0, 3),
      align(center)[Actuator Model \
        (thruster / torque)],
      name: <actuator>)
    edge(<actuator>, <orbmech>, "-|>")

    group([Simulation Environment], (<orbmech>, <actuator>))

    // Sim Bridge / Adapter
    component((1.5, 1.5), [Sim Bridge / Adapter], name: <adapter>)

    // ROS2
    component((6, 1.5), [ROS2 Interface], name: <bus>)
    component((7.5, 0), [Sensor Simulator], name: <sens>)
    component((7.5, 1), [Navigation Module], name: <nav>)
    component((7.5, 2), [Guidance Module], name: <guid>)
    component((7.5, 3), [Control Module], name: <ctrl>)

    group([ROS2], (<bus>, <sens>, <nav>, <guid>, <ctrl>), color: palette.ros2)

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
    component((0, 0),
      align(center)[Orbital Mechanics / \
        Attitude Dynamics],
      name: <env-dyn>)
    component((0, 2),
      align(center)[Actuator Model \
        (thruster / torque)],
      name: <env-act>)
    // Physics loop: Actuator feeds state back into dynamics
    edge(<env-act>, <env-dyn>, "-|>")

    // GNC Node
    component((3, 0), [Navigation], name: <gnc-nav>)
    component((3, 1), [Guidance], name: <gnc-guid>)
    component((3, 2), [Control], name: <gnc-ctrl>)

    // GNC pipeline
    edge(<gnc-nav>, <gnc-guid>, "-|>")
    edge(<gnc-guid>, <gnc-ctrl>, "-|>")

    // Groups (outer drawn behind via layer)
    group([ROS2],
      (<env-dyn>, <env-act>, <gnc-nav>, <gnc-guid>, <gnc-ctrl>),
      color: palette.ros2, inset: 2em, layer: -1)
    group([Env Node], (<env-dyn>, <env-act>))
    group([GNC Node], (<gnc-nav>, <gnc-guid>, <gnc-ctrl>),
      color: palette.gnc)

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
    component((1, 1), [ROS2 Core], name: <ros2-core>)
    component((1, 0), [Topics & Services], name: <topics>)
    component((1, 2), [Logging System], name: <logging>)
    
    edge(<ros2-core>, <topics>, "<->")
    edge(<ros2-core>, <logging>, "<->")
    
    group([ROS2 Backend], (<ros2-core>, <topics>, <logging>), color: palette.ros2)

    // User Interface Layer
    component((5, 0), [CLI Tools], name: <cli>)
    component((5, 1), [Web Dashboard], name: <web>)
    component((5, 2), [Log Viewer], name: <logview>)
    component((5, 3), [Plotting / Viz], name: <viz>)
    
    group([User Interfaces], (<cli>, <web>, <logview>, <viz>), color: palette.ui)
    
    // Connections
    labeled-edge(<topics>, <cli>, "->", [`ros2 topic`])
    labeled-edge(<ros2-core>, <web>, "->", [WebSocket / REST])
    labeled-edge(<logging>, <logview>, "->", [Log Stream])
    labeled-edge(<topics>, <viz>, "->", [Real-time Data])
  },
)

#set page(width: auto, height: auto)
#ui-arch

#internal-arch

#external-arch