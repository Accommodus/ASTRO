/* 
Introduction and Motivation for the Project: Should include problem formation, the rationale of the project, the proposed solution, and the importance and impact of the project
*/

// Interoperate: projects can work together easier using the ROS module system. Ros 2 provides building blocks that make it easier to combine different systems. Right now, Star lab has a system they use to connect a simulation to a processor that is just an ad hoc udp client-server - simulation runs on a powerful desktop, while flight software runs on Nvidia Jetson (supposed to be like satellite computer). ROS2 will allow these two to communicate as long as their functionality is encapsolated as seperate modules and a connection is made between them. 

// Modularize: Different logging systems, unable to run sim for multiple systems. The simulation bridge system described earlier has all functionality in two executables. For example, the logging, calculations, and networking are all handled in one executable. ROS2 will provide a framework to give a seperation of concerns. The current software that the lab uses will be packaged in the ROS ecosystem.

// The simulation framework is important because all of the software of the lab revolves around satellites. All the software will benefit from connection to different simulations. We will be able to connect modules that are not directly related to the flight software (like the robotic arm) through other connected modules.

== Interoperate
The STAR laboratory at UF currently has a number of projects that are unable to interoperate without considerable work. Currently, the lab relies on an ad hoc UDP client-server architecture to connect simulations running on powerful desktops with flight software operating on Nvidia Jetson hardware. This lack of standardization limits integration. ROS 2 provides the necessary building blocks to bridge these distinct systems, allowing for seamless communication as long as functionality is encapsulated into separate modules.

== Modularize
The current infrastructure suffers from a lack of modularity, with disparate logging systems and an inability to run simulations for multiple systems simultaneously. The existing simulation bridge consolidates all functionality, like including logging, calculations, and networking, into a few monolithic executables. By migrating to the ROS 2 framework, the project aims to package the lab's current software into the ROS ecosystem, ensuring a clear separation of concerns and modernizing the architecture.

== Impact
This simulation framework is critical because the STAR laboratory's software development revolves primarily around satellite operations. Standardizing this bridge will allow all lab software to benefit from connections to various simulations, not just specific instances like Basilisk @bsk. Furthermore, this architecture will enable the integration of modules not directly related to flight software, such as robotic arms, expanding the lab's testing and operational capabilities.