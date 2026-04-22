#import "../style.typ": report

#show: report

#title[ASTRO Final Presentation Script]

= Presenter Notes
- Target length: about 6 to 8 minutes total, with most time spent in the live walkthrough.
- Person 1 = Cannon
- Person 2 = Dylan
- Person 3 = Caleb

= Cannon (Person 1)

== Introduction - What is ASTRO?

Good [morning/afternoon]. We are presenting ASTRO, which stands for Autonomous Satellite Test and Robotics Operations. ASTRO is a ROS 2 framework for connecting STAR Lab satellite simulation software with guidance, navigation, and control software in a modular and reproducible way.

The core problem we are addressing is that the lab's simulation and control workflows have historically depended on ad hoc UDP links and one-off integrations. Those connections can work for a single setup, but they are harder to test, harder to reuse, and harder to extend across multiple lab projects. Our goal was to replace that with a standard ROS 2 interface that supports both a native ROS 2 path and a bridge path for external simulators.

== Introduction - Today's Agenda

Today's presentation is mostly a live walkthrough of the working software. We will briefly frame the project, then walk through the repository, show the local controller runs, show the external simulator bridge, show the two-machine deployment setup, and end with contributions and wrap-up.

= Dylan (Person 2)

== Project Walkthrough - Live Demo Plan

The biggest change since the earlier demo is that we are no longer showing only the original DLQR baseline. We now have a broader final project story to demonstrate.

First, we will show the repository layout, including the internal ROS 2 package, the external simulator bridge package, the deployment files under `demo/`, and the Docker support under `.docker/`.

Second, we will show the internal ROS 2 path running locally in two modes: the original DLQR launch and the newer QP-MPC launch.

Third, we will show the bridge path, where `external_sim_bridge` can stand in for the environment side using either a deterministic fake backend or the Basilisk backend.

Fourth, we will show the two-machine deployment configuration, including both Tailscale and local LAN Compose setups.

Finally, we will show the test suite that validates both the internal package and the bridge package.

== Project Walkthrough - Architecture

At a high level, ASTRO now supports two operating modes.

In the internal ROS 2 mode, `env_node` publishes state on `env_data`, and either `gnc_node` for DLQR or `qp_gnc_node` for QP-MPC computes thrust and sends it back through `actuation_cmd`.

In the bridge mode, `bridge_node` from `external_sim_bridge` takes over the environment role but keeps the same ROS contract. That means the control side does not have to change when we swap between a native environment and an external simulator backend.

That is important because it shows the project grew beyond a single controller demo. Since the residential showcase, we completed the QP-MPC controller integration, validated the external simulator bridge against both fake and Basilisk backends, and added cleaner deployment support for both local LAN and Tailscale-based two-host demos.

= Caleb (Person 3)

== Project Walkthrough - Live Software Segments

As we move through the live walkthrough, the key result to watch for is that the interfaces stay consistent even as the implementation underneath changes.

In the internal path, we can launch the original DLQR control loop or the newer QP-MPC control loop.

In the bridge path, we can replace the native environment with `external_sim_bridge`, first with the fake backend for deterministic validation and then with the Basilisk backend for higher-fidelity external simulation.

On the deployment side, we now support both a Tailscale-based cross-network setup and a local LAN Compose setup. That means the project is not just theoretically distributed; the deployment path is now documented and reproducible.

On the testing side, the project now includes unit, integration, and launch-based validation for both the internal ROS 2 package and the bridge package.

= Cannon (Person 1)

== Individual Responsibilities - Cannon Whitney

My role on the project was manager and integration coordinator. I handled milestone planning, advisor communication, and decisions around how the internal ROS 2 path and the external bridge path fit together. I also contributed to documentation, demo readiness, and the overall final deliverables. A large part of my work was making sure the project stayed coherent as it expanded from the original DLQR baseline into the final multi-path system we are showing today.

= Dylan (Person 2)

== Individual Responsibilities - Dylan Long

My role was SCRUM master and repository and deployment support. I maintained the sprint workflow, task tracking, and review flow in GitHub. I also worked on devcontainers, deployment-image support, documentation, and demo guidance. That became especially important as the project moved beyond a single local run and into a setup that needed to support bridge validation and reproducible multi-machine demos.

= Caleb (Person 3)

== Individual Responsibilities - Caleb Jackson

My role was developer on the core implementation side. I worked on the ROS 2 node path, including the internal environment and controller implementations, the QP-MPC controller integration, and the external simulator bridge with fake and Basilisk backends. The main technical through-line in my work was keeping the control boundary stable while broadening the set of modes ASTRO can support.

= Cannon (Person 1)

== Wrap-up - Where We Land

To conclude, ASTRO now lands in a much stronger place than it was in at the time of the earlier demo. We have a closed-loop ROS 2 simulation workflow with both DLQR and QP-MPC controllers, an external simulator bridge validated through pluggable backends, and a reproducible two-machine deployment path using either Tailscale or a local LAN.

In other words, the project no longer demonstrates only that a ROS 2 baseline is possible. It now demonstrates a more complete and extensible architecture for the lab's simulation and control workflows.

== Wrap-up - Remaining Work

The main remaining enhancement is telemetry and log buffering under Issue `#25`. That is now a follow-on improvement rather than a blocker for the core project outcome. The core internal path, controller alternatives, bridge path, and distributed deployment story are all in place.

== Thank You

Thank you for watching. We are happy to answer questions, and the full documentation and repository are available in the ASTRO project materials.
