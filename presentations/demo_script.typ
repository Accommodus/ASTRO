#import "../style.typ": report

#show: report

#title[ASTRO Senior Showcase Demo Script]

= Presenter Notes
- One presenter can run this entire demo.
- Aim for about 90 seconds for passersby and about 2 minutes for interested visitors.
- Keep one terminal visible for the environment side and one for the GNC side.

= Full Booth Script
Hello, we are ASTRO, which stands for Autonomous Satellite Test & Robotics Operations. Our project is about making satellite simulation and control software communicate through a modular ROS 2 architecture instead of a one-off custom link.

In the STAR Lab, this is a real problem because simulation and flight software often live on different machines. Right now, that connection is often handled through ad hoc UDP links between a powerful desktop running a simulation and Jetson-based hardware representing the onboard computer. That works for one specific setup, but it is hard to reuse, hard to test, and hard to integrate with other lab projects because networking, logging, and control behavior get bundled together inside monolithic executables. Our goal is to replace that with a standard ROS 2 interface so different simulations, controllers, and future hardware modules can interoperate more cleanly.

Prior work in this space, including simulator-specific bridge approaches and earlier custom lab integrations, shows that distributed simulation and control can be connected successfully. The gap is that those solutions are often tightly tied to one simulator or one custom workflow. ASTRO addresses that gap by moving the connection into a modular ROS 2 architecture that preserves the working control path while making the interface cleaner to test, reuse, and extend.

For this demo, this laptop is running the environment side of the simulation, and the other laptop is running the guidance, navigation, and control node. The environment node publishes the satellite state on `env_data`. The GNC node receives that state, computes a DLQR control command, and sends that command back through `actuation_cmd`.

What makes this important is that the two sides are running on separate computers, so this is a distributed demo, not just two local processes on one machine. That is the main idea of ASTRO: separate the simulation and control roles into clean ROS 2 components that can communicate across systems.

This is also a validated baseline, not just a visual demo. Our current results are a working two-laptop distributed ROS 2 control loop, a clean separation between simulation and control through reusable interfaces, and a tested DLQR baseline. This closed-loop ROS 2 path has unit tests and a regression test that compares the trajectory it produces against the original reference implementation. So what we are showing live is the same baseline we can also verify in software.

Beyond this demo, the main remaining work is less about inventing a new architecture and more about validating the broader adapter path and continuing to improve telemetry and operator tooling. The generic bridge direction is already defined and under active validation, and future controller extensions like QP_MPC build on that same modular structure.

= Short Version
ASTRO is a ROS 2-based framework for satellite simulation and control. In the STAR Lab, simulation and flight software often run on different machines and are connected with ad hoc UDP code that is hard to reuse or extend. In this demo, one laptop runs the environment node, the other runs the GNC node, and they exchange state and control data live across a distributed ROS 2 setup. This same baseline is also validated against the original reference trajectory.

= Optional Q&A Lines
- *Why two laptops?* Because the lab problem is inherently distributed: simulation and flight-software components often live on different machines.
- *What is implemented today?* A working ROS 2 DLQR path with an environment node, a GNC node, a topic for state, and a service for actuation commands.
- *What gap does this fill?* It moves the lab away from simulator-specific or one-off networking solutions toward a reusable ROS 2 interface.
- *What comes next?* Mainly validation of the broader external-simulator bridge path, plus continued telemetry tooling and controller expansion.
