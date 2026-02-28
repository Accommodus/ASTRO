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
- 3 hours: Working on improvements to the containerized dev enviornment
- 1 hour: Organizing git repository
- 2 hours: Researching how to run a ROS2 instance across multiple devices
- 2 hours: Comparing package output to reference output

In total I spent about 10 hours on the project this week.

= Contribution to the Project Development Process
I replaced the three per-OS devcontainer configurations with a unified Docker Compose setup that automatically detects the host operating system and generates the appropriate runtime configuration at launch time. This eliminates the need for teammates to manually select the correct configuration and ensures consistent behavior across Linux, macOS, and Windows. I also verified the correctness of our ROS 2 package by comparing its output against the reference implementation provided by the STAR Lab, and I reorganized the Git repository to reduce clutter and make the project layout easier to navigate.

= Demonstration of Applied Knowledge
I applied knowledge of Docker Compose service composition to structure the container runtime configuration as a base file layered with a generated, OS-specific override. I wrote a portable shell script that uses `uname` for OS detection and `mktemp` combined with an atomic `mv` to write the override file in a way that works regardless of who runs the script or what permissions exist on the repository files. I also applied my understanding of ROS 2 publish-subscribe messaging to reason about what is required for nodes running on separate physical devices to discover and communicate with one another.

= Reflection and Lessons Learned
The biggest lesson this week was that infrastructure changes need to be tested on every target platform before being merged. A refactor that worked correctly on Linux silently broke the macOS launch flow because a file ownership edge case only manifested there. Having my teammate test the change on his MacBook before the branch was merged to main caught the failure early and prevented a broken environment from reaching the rest of the team.

Progress has been slower than I initially hoped, but with the foundational setup now complete the team should be able to move considerably faster. I have found working with the team enjoyable, and adopting GitHub Issues as our task-tracking system has already proven useful for keeping work organized and visible.