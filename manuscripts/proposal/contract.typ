#import "style.typ": tbl, signature-list, subtitle

== Objectives
  Our team's primary objective is to take an existing software system from the STAR Lab and fully integrate it into the ROS2 framework. We plan to encapsulate the current infrastructure to function within ROS2 nodes, which will allow us to modernize the architecture and expand its capabilities using the framework's communication tools. We are currently evaluating potential candidates for this migration and will finalize the specific system after meeting with the professor to determine the best fit for the project scope.

== Role and Responsibilities

  #let tl = [
    Responsible for keeping the team on schedule with course deadlines and handling communication with the advisor.
    Also makes final decisions on the system architecture and how different parts of the project connect.
  ]

  #let sm = [
    Organizes weekly meetings and keeps our task board updated so we know what everyone is working on.
    Also manages the GitHub repo, including code reviews and merging pull requests to keep the main branch clean.
  ]

  #let bed = [
    Focuses on setting up the ROS2 architecture and defining the nodes and topics for system communication. Responsible for taking the pre-existing systems and wrapping them to work correctly within the ROS2 environment.
  ]

  #tbl[
    | *Role* | *Responsibilities* | *Assignee* |
    | --- | --- | --- |
    | Team Lead | #tl | Cannon Whitney |
    | Scrum Master  | #sm | Dylan Long |
    | Back End Developer  | #bed | Caleb Jackson |
  ]

== Values

  === Consensus
  For extensive changes to the code base, the group will be consulted and must sign off on the idea.

  === Boundaries
  When working in another member's sector, we must ensure that this is permitted by the member in control of that portion. 

  === Goal-Oriented
  While working on the project, it is imperative to keep the final product in mind when making changes. 

== Software Configuration Management
  For configuration management, we are utilizing Git and GitHub with a workflow centered around three types of branches: prod, dev, and feature branches. The prod branch is strictly for stable, demo-ready code, while dev serves as the main integration branch where the team synchronizes their work. All new code will be written on short-lived feature branches created off of dev. We will use Pull Requests to merge these feature branches back into dev, which allows us to perform code reviews and resolve merge conflicts before the code becomes part of the shared project. We will only merge dev into prod once we have reached specific milestones and verified that the system is stable and bug-free.

== Communication
  Constant communication via SMS, additionally, meetings will be held over Microsoft Teams for more active engagement. We will alternate being the meeting facilitator.

  === Team Meeting Times
  - Initial: 1/22 at 2:00pm
  - Subsequent: Thursday at 2:00pm
  
  === Advisor Meeting Times
  - Initial: 1/21 at 2:00pm
  - Subsequent: Wednesday at 2:00pm

== Conflict Resolution

 === Protocol
  1. *Stop:* We will pause discussion or development immediately if a disagreement creates a blocker, giving everyone a moment to collect their thoughts.
  2. *Collaborate:* We will work together to identify the root of the problem, focusing on finding a solution that best serves the project's goals.
  3. *Listen:* We will give whoever is speaking our full attention to understand their implementation logic before we start critiquing their approach or suggesting changes.

 === Consequences
 If the agreement is violated outside the scope of a group meeting, a meeting will be immediately scheduled for the soonest available time slot. Then, the meeting will follow our protocol for conflict resolution until we have reached a suitable agreement on how to resolve the violation. This meeting must involve all members of the project and has no set length or time period.

== Signatures
  #signature-list(
    ([Cannon Whitney], image("signature/CW_sig.jpg")),
    ([Caleb Jackson], image("signature/CJ_sig.jpeg")),
    ([Dylan Long], image("signature/DL_sig.png"))
  )