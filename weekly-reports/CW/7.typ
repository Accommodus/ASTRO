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
- 2 hours: Researching Docker daemon failure on NJON and NVIDIA/Jetson forum solutions
- 4 hours: Resolving Docker on NJON (uninstall, reinstall 28.0.1, iptables-legacy configuration and verification)
- 2 hours: Preparing and organizing Presentation 2 (script, slide order, and speaker segments)
- 1 hour: Coordination with team on validation milestones and open PRs (#20, #21, #11)

In total I spent about 9 hours on the project this week.

= Contribution to the Project Development Process
This week I contributed by resolving the Docker daemon initialization failure on the NJON and by leading my portion of the team's second progress presentation. On the NJON, the daemon had been failing to start due to a networking configuration error involving iptables. I followed up on the diagnostic work from the previous week by researching the issue and found a relevant NVIDIA Developer forum discussion indicating that recent Docker versions have strict kernel module dependencies that conflict with the default Jetson Linux kernel; the forum consensus was that installing Docker version 28.0.1 resolves the compatibility issue. I completely removed the existing Docker Engine and purged residual configuration files per the official Docker uninstall documentation, reinstalled Docker via apt targeting version 28.0.1 as recommended, and forced the system to use the legacy iptables backend with `sudo update-alternatives --set iptables /usr/sbin/iptables-legacy` so the network bridge could initialize without a full Jetson kernel recompile. After applying the update and iptables change, the Docker daemon started successfully. For the presentation, I worked with the team to structure the deck and script so that each person speaks a contiguous block and Individual Responsibilities appear at the end of each speaker's section. I prepared and delivered the opening (title and Progress Update: requirements, backlog, GitHub progress, and possible next steps) and my Individual Responsibility slide.

= Demonstration of Applied Knowledge
I applied documentation-driven troubleshooting and community-sourced solutions to fix the NJON Docker failure. I used the official Docker documentation for uninstalling the Docker Engine on Ubuntu and for installing from the repository, and I applied the NVIDIA forum finding that Docker 28.0.1 addresses Jetson kernel compatibility. I used the system alternatives mechanism to switch the iptables backend to the legacy implementation so that Docker's bridge driver could register correctly without modifying or recompiling the kernel. Verifying with the hello-world image confirmed that the daemon, network bridge, and image pull/run path were all working. For the presentation, I used the Typst formatting engine to create the slide deck.

= Reflection and Lessons Learned
The main lesson this week was that persistent infrastructure issues on constrained platforms like the Jetson often have known, version-specific fixes in community forums. Once the NVIDIA discussion pointed to Docker 28.0.1 and the iptables backend, following the documented uninstall and reinstall steps and then applying the alternatives switch produced a clean, reproducible fix. For the presentation, aligning the script and slides so that each person's section was self-contained and ended with their Individual Responsibility made it easier to rehearse and hand off between speakers, and kept the narrative consistent with the assignment requirements.
