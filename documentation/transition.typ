= Transition Plan and Next Steps

== Knowledge transfer

Repository documentation on `main` (`README.md`, `docs/next-phase-plan.md`, and `demo/README.md`) together with this report and the appendices record how to build, run, test, and deploy the system. Hands-on walkthroughs with the advisor and STAR Lab stakeholders should cover the ROS 2 graph (`env_data`, `actuation_cmd`), the two Compose stacks, and where to change parameters versus code. Issue and PR history on GitHub provides design rationale for major changes. New contributors should clone `main`, use the devcontainer or native ROS 2 Kilted instructions, and run `colcon test` before modifying controllers or the bridge.

== Sustainability

The project remains viable as long as the ASTRO repository is maintained: container images on GHCR, reproducible `colcon` builds, and CI workflows reduce bus factor compared to ad hoc executables. Pinning ROS distribution assumptions (Kilted today) and keeping `package.xml` / `setup.py` dependencies explicit avoids silent drift. Continued use of GitHub Issues for backlog items (telemetry, bridge validation, CI breadth) gives a single place to prioritize work after the course ends.

== Near-term next steps

Close remaining enhancement items in the issue tracker, extend automated testing as new controllers or backends land, and refresh README sections whenever behavior diverges from docs. For longer-term lab adoption, follow-on work should parameterize dynamics further, harden external-simulator validation, and add operator-facing telemetry consistent with `docs/next-phase-plan.md`. The team recommends confirming priorities with the project advisor each semester.
