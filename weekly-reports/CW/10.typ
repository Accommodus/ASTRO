#import "../../style.typ": *
#show: report

#title[Cannon Whitney's Report]

= Time Dedicated to the Project
- 2 hours: Planning and proposing improvements to the lab network ROS2 Docker system
- 3 hours: Implementing cross-compilation GitHub Actions workflows for Docker images
- 2 hours: Finalizing Tailscale networking and rmw_zenoh_cpp integration
- 1 hour: Fixing submodule dependencies for C header files

In total I spent about 8 hours on the project this week.

= Contribution to the Project Development Process
This week I focused heavily on DevOps, CI/CD, and improving the robustness of our distributed simulation network. I spent a portion of this week planning and proposing architectural improvements to the lab network's ROS2 Docker system.

To improve our build processes, I updated the GitHub Actions workflows to support cross-compilation for our Docker images. This ensures consistent artifact generation across the diverse architectures used in the lab. 

On the networking side, I merged several improvements tied to our Tailscale transition. I added Tailscale to the Docker Compose networks, completing the migration of ROS 2 discovery from CycloneDDS to rmw_zenoh_cpp to allow reliable cross-device communication over the VPN wrapper. I also added a configurable `sim_rate_ms` parameter to the `EnvNode` and our Docker configurations to allow fine-tuning of the simulation frequency.

= Demonstration of Applied Knowledge
I applied CI/CD pipeline concepts by configuring GitHub workflows for Docker image cross-compilation, enabling multi-architecture builds. On the networking front, I combined container orchestration with virtual private networks by weaving Tailscale into our Docker Compose stack, bridging ROS 2 nodes predictably despite complex host NAT constraints. 

= Reflection and Lessons Learned
Additionally, formalizing the Tailscale-based networking architecture required careful planning. I learned that having a well-thought-out plan for the lab network's ROS2 Docker system prevents ad-hoc networking patches and provides a reliable baseline for distributed simulation. Going forward, the investment in cross-compilation workflows means the team can reliably build images for their target hardware without manual intervention.
