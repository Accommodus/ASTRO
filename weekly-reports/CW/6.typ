#import "../../style.typ": *
#show: report

/*
The purpose of an individual report for a computer science senior design course is to assess and communicate each team member's personal contributions, understanding, and reflections on the project. It allows you to demonstrate your specific skills, responsibilities, and learning experiences, while also providing insight into your problem-solving approach and engagement with the project. Overall, it promotes individual accountability and fosters a deeper understanding of each member's role in achieving the team's goals.

It supports Unit Objective #4, "Establish clear expectations, roles, responsibilities, and communication guidelines among team members to ensure effective collaboration and the successful completion of the project.”


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
- 2 hours: Meeting with team and other STAR lab members
- 3 hours: Working on improvements to the containerized dev environment
- 1.5 hour: Improving CI/CD pipelines
- 2 hours: Installing Docker on STAR Lab devices
- 2 hours: Working on tests for GNC node

In total I spent about 10.5 hours on the project this week.

= Contribution to the Project Development Process
This week I contributed in both infrastructure and validation work. On the STAR Lab side, I helped get Docker running on the lab desktop, worked to cache login credentials for the desktop over a wired connection, and gathered detailed diagnostic information from the NJON when Docker failed to start. Rather than making potentially disruptive changes to the network bridge or system time, I documented the failure state and held off on further modifications until we could coordinate with the relevant lab members. I continued improving the containerized development environment and CI/CD workflow, and I spent time on GNC node tests. Together, this moved the project forward by making our tooling more reliable and by reducing uncertainty around both the lab machines and the software environment.

= Demonstration of Applied Knowledge
I applied practical knowledge of Linux services, container infrastructure, and debugging while troubleshooting Docker on the NJON. By checking the daemon state and reviewing the logs, I traced the startup failure to Docker's network controller, specifically an `iptables` failure involving the missing `addrtype` match module when Docker attempted to register its bridge driver. That analysis helped narrow the problem from a general "Docker is broken" report to a specific networking and kernel-module issue. I continued work on the containerized development environment, CI/CD pipeline, and GNC node tests, which required using my understanding of reproducible development workflows and ROS 2 package verification to support the team's broader implementation effort.

= Reflection and Lessons Learned
The biggest lesson this week was that infrastructure work on lab hardware often depends as much on careful observation and coordination as it does on direct technical fixes. The desktop setup went smoothly once we methodically verified the physical connection and retried the network steps, but the NJON issue showed that some failures should be investigated and documented before making deeper system changes. In this case, stopping after identifying the Docker daemon's `iptables` failure was the right decision because it preserved the current machine state for the people already working on related system configuration.
