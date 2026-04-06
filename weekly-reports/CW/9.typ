#import "../../style.typ": *
#show: report

/*
The purpose of an individual report for a computer science senior design course is to assess and communicate each team member's personal contributions, understanding, and reflections on the project. It allows you to demonstrate your specific skills, responsibilities, and learning experiences, while also providing insight into your problem-solving approach and engagement with the project. Overall, it promotes individual accountability and fosters a deeper understanding of each member's role in achieving the team's goals.

It supports Unit Objective #4, "Establish clear expectations, roles, responsibilities, and communication guidelines among team members to ensure effective collaboration and the successful completion of the project."


Time Dedicated to the Project

"Time Dedicated to the Project" refers to the amount of effort and hours you personally spent working on the project. It demonstrates your commitment and the extent of your involvement over the course of the project timeline. This can include:

    The total hours or days you allocated for different phases (planning, research, implementation, testing)
    How you managed your schedule to ensure progress
    The consistency and commitment shown in working on the project

Expressing this helps to show the level of dedication and significant effort you invested in your work.

Contribution to the Project Development Process

"Contribution to the Project Development Process" refers to the specific roles, actions, or efforts you made that helped progress, improve, or complete the project. It highlights how you actively participated in the different stages of developing the project, such as planning, designing, executing, testing, or refining. This could include tasks like:

    Coming up with a project idea or planning the approach
    Designing or building components
    Solving problems or overcoming challenges
    Collaborating with team members
    Conducting research or gathering data
    Implementing solutions or techniques
    Improving processes or workflows

Essentially, it showcases your active involvement and the impact you had on moving the project forward.


Demonstration of Applied Knowledge

"Demonstration of Applied Knowledge" refers to showing how you have practically used the knowledge, skills, or concepts you have learned in your project. It involves providing evidence or examples that illustrate your ability to apply theoretical understanding to real-world situations or tasks. This could include activities such as solving specific problems, implementing techniques, analyzing data, or creating solutions that reflect your understanding in action. Essentially, it's about proving that you can translate knowledge into practice.

    Problem-Solving: Describing how you identified a specific challenge and used your skills or concepts learned to develop a solution
    Practical Implementation: Showing how you applied a particular methodology, technique, or tool to complete a task or craft a product
    Data Analysis: Presenting how you collected, processed, and interpreted data to draw meaningful conclusions relevant to your project
    Design and Development: Illustrating how you designed a prototype, model, or system based on theoretical principles
    Case Study or Scenario: Explaining how you used your knowledge to analyze a real-world case or scenario related to your field
    Testing and Evaluation: Detailing how you tested a hypothesis or model and used results to refine or improve your approach
    Use of Theoretical Concepts: Showing how specific theories or concepts informed your decision-making process throughout the project


Reflection and Lessons Learned

"Reflection and Lessons Learned" refers to your thoughts on the overall experience of your project, including what went well, what challenges you faced, and what you learned from the process. It involves analyzing your personal growth, insights gained, and how the experience has influenced your skills or understanding. Key components include:

    Reflecting on successes and areas for improvement
    Identifying specific lessons or insights gained during the project
    Considering how these lessons will influence your future work or projects
    Discussing any unexpected outcomes or surprises

This section demonstrates your ability to critically assess your work and grow from the experience.

*/

#title[Cannon Whitney's Report]

= Time Dedicated to the Project
- 3.5 hours: Resolving Docker networking on the NJON
- 2 hours: Debugging NTP/chrony/timedatectl sync with Channing
- 2 hours: Meetings with ASTRO and STAR lab members
- 2 hours: Documenting findings

In total I spent about 9.5 hours on the project this week.

= Contribution to the Project Development Process
This week I focused on two areas: getting Docker containers to communicate reliably across the NJON and a Windows development machine, and debugging a broken NTP time-synchronization service on the NJON alongside Channing.

For Docker networking, I discovered that the default Docker bridge network fails to initialize on the Jetson due to kernel-level compatibility issues, so all containers on the NJON must now be run with the `--network host` flag. I verified basic TCP connectivity using netcat across Docker containers on both devices with host networking, noting that the Linux side must act as the server and that Docker Desktop on Windows requires an initial ping to cache the Jetson's MAC address before ARP-dependent ephemeral containers can connect. I then moved beyond raw TCP to a working ROS 2 setup: because standard Fast DDS (UDP multicast) fails across the Windows WSL2 NAT, I implemented a workaround using Eclipse Zenoh. The NJON runs a Zenoh listener on the host network acting as a TCP server, and the Windows machine runs a Zenoh client that reaches out over a custom Docker network, packing the ROS 2 UDP traffic into a reliable TCP stream.

For NTP, I paired with Channing to diagnose why `timedatectl` showed NTP as inactive on the NJON. We traced the issue to prior system commands (`sudo update-alternatives --set iptables /usr/sbin/iptables-legacy`, firewall rules via `sudo ufw allow 11811/udp`, and repeated Docker daemon restarts) that may have affected the chrony/NTP stack. Debugging is ongoing.

= Demonstration of Applied Knowledge
I applied container networking concepts at multiple layers: diagnosing why the Jetson kernel cannot support the default Docker bridge, understanding ARP resolution behavior in Docker Desktop on Windows, and selecting Eclipse Zenoh as a DDS-bridge transport to tunnel ROS 2 discovery and data over TCP when UDP multicast is unavailable. For the NTP debugging session, I used `systemctl`, `timedatectl`, and chrony tooling to inspect service states and correlated the timeline of prior administrative commands (iptables backend switch, UFW rule additions, Docker restarts) with the onset of the synchronization failure. The coordination work required reasoning about shared hardware resources across concurrent lab projects and proposing a device-allocation strategy (JON vs. JEN) to prevent destructive interference.

= Reflection and Lessons Learned
The main takeaway this week is that working on shared embedded hardware with multiple concurrent projects requires explicit device allocation and communication protocols. The NTP breakage likely resulted from accumulated system-level changes made across different debugging sessions without a clear change log, which made root-cause analysis harder. Going forward, I plan to keep a running log of every administrative command issued on the NJONs so that side effects are traceable. On the networking side, the Zenoh bridge is a pragmatic solution but adds a dependency; documenting the exact setup and providing a reproducible test script will be important so other team members can verify the communication path without deep knowledge of the DDS layer.
