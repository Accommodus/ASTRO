= Project Overview

== Project Name and Description
ASTRO: Autonomous Satellite Test & Robotics Operations

The project aimed to take an existing software system from the STAR Lab and fully integrate it into the ROS2 framework to create a "plug and play" digital twin environment. This environment was designed to be highly modular and easily extendable, allowing developers to easily add new ROS2 packages and set them up to connect seamlessly to the digital twin environment.

== Project Goals and Objectives
Our team's primary objective was to encapsulate the current infrastructure to function within ROS2 nodes. This modernization effort aimed to enable interoperability between different STAR laboratory projects without considerable rework. This would also replace ad-hoc structures with the standardized ROS2 communication modules. This effectively bridged simulations running on powerful desktops with flight software on hardware like the Nvidia Jetson. Additionally, the project provided a clear separation of concerns by modularizing monolithic executables -- which previously handled multiple tasks, such as logging, calculations, and networking -- into manageable ROS2 packages. Ultimately, these improvements expanded lab testing capabilities by allowing the integration of additional modules, such as robotic arms, into the broader simulation framework.

== Project Scope
The scope of the project included setting up the new ROS2 architecture, defining the nodes and topics for system communication, and wrapping pre-existing simulators to function correctly within the ROS ecosystem. The final product was a standardized simulation framework that could be adopted by various lab software projects revolving around satellite operations.

== Team Members and Roles
- Cannon Whitney (Project Manager): Responsible for keeping the team on schedule, handling communication with the advisor, and making final decisions on system architecture and project integration.
- Dylan Long (SCRUM Master): Organizes weekly meetings, maintains the task board, and manages the GitHub repository (including code reviews and pull requests).
- Caleb Jackson (Backend Developer): Focuses on setting up the ROS2 architecture, defining nodes and topics, and wrapping pre-existing systems to work within the ROS2 environment.
