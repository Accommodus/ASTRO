FROM ros:kilted

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Handle existing users with UID 1000 (like 'ubuntu' in newer images)
RUN if id -u 1000 >/dev/null 2>&1; then \
        if [ "$(id -un 1000)" != "$USERNAME" ]; then \
            userdel -r $(id -un 1000); \
        fi \
    fi

# Create the user if they don't exist yet
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

# Add sudo support
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# System packages, Python toolchain, and C++ toolchain
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
        python3 python3-pip python3-venv python3-dev \
        build-essential gcc g++ cmake gdb \
        openssh-client git \
    && rm -rf /var/lib/apt/lists/*

ENV SHELL=/bin/bash
ENV ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
ENV ROS_DOMAIN_ID=42

COPY .devcontainer/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

USER $USERNAME
WORKDIR /home/ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

# Source ROS 2 in every new terminal
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc

# Add colcon build tab-completion
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc

# Source the workspace install overlay if it has been built
RUN echo 'if [ -f /home/ws/install/setup.bash ]; then source /home/ws/install/setup.bash; fi' >> /home/$USERNAME/.bashrc
