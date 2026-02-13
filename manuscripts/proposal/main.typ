#import "style.typ": *
#show: general

#include "cover.typ"
#pagebreak()
  
= Table of Contents
#outline()
#pagebreak()

= Abstract
  The goal of ASTRO is to make a bridge between any satellite simulation software and ROS2 to allow for seamless control. This project seeks to improve previous work and standardize it to work with any simulation software instead of specifically working with Basilisk, as previous projects have done. The final product will allow for communication between the satellite simulation and ROS 2 for self-adjustment mid-operation.

= Team Contract
#include "contract.typ"

= Introduction and Motivation
#include "intro.typ"

= Literature Survey
#include "survey.typ"

= Proposed Work
  - Create a bidirectional interface that streamlines communication between a satellite simulation machine in the STAR lab and the in-lab machine operating as a satellite
  - Accomplished using Python and C++
  - ROS 2 library
  - Basilisk library
  - In-house simulation software from STAR labs
  - Generalise the interface to work with a variety of unrelated machines
  - Incorporate user interaction during simulation

= Product Backlog
    #tbl[
      | *Item* | *Primary Assignee* |
      | Modular simulation engine | Caleb |
      | Modular flight software engine | Cannon |
      | Bridge linked to machines able to read or send data | Caleb |
      | Basic UI to configure system | Dylan |
      | Interface to send data to simulation while in operation | Dylan |
      | Loop in hardware, like robotic arm | Cannon |
    ]

= Project Plan
  - Milestone 1: Basic Functionality: #{datetime.today() + duration(days: 18)}.display()
    - Modular Simulation Engine
    - Modular Flight Software Engine
  - Milestone 2: Basic Communication: #{datetime.today() + duration(days: 36)}.display()
    - Bridge Across Machines
    - Live Simulation Interface
  - Milestone 3: Advanced Capabilities: #{datetime.today() + duration(days:54)}.display() 
    - Hardware in the Loop
    - Advanced UI
  - Milestone 4: Final Product: #{datetime.today() + duration(days:73)}.display()
    - Refined UI
    - Streamlined Connectivity
    - Generic Hardware Functionality
= References
  #bibliography("lit.yaml")