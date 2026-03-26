FROM ghcr.io/accommodus/astro:latest

USER root
WORKDIR /astro_ws

COPY src/ src/
RUN . /opt/ros/kilted/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    colcon build --packages-select distributed_satellite_sim

# Role-aware entrypoint (starts only one node)
COPY .docker/distributed_satellite_sim.entrypoint.sh /usr/local/bin/distributed_satellite_sim-entrypoint.sh
RUN chmod +x /usr/local/bin/distributed_satellite_sim-entrypoint.sh

# Source overlay for every shell session
RUN echo "source /astro_ws/install/setup.bash" >> /home/ros/.bashrc

USER ros
ENTRYPOINT ["/usr/local/bin/distributed_satellite_sim-entrypoint.sh"]
