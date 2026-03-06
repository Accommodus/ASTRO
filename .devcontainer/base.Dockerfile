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

# System packages and Python toolchain
RUN apt-get update && apt-get upgrade -y \
    && apt-get install -y \
        python3-pip \
        openssh-client \
        git \
    && rm -rf /var/lib/apt/lists/*

ENV SHELL=/bin/bash

USER $USERNAME
CMD ["/bin/bash"]

# Source ROS 2 in every new terminal
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USERNAME/.bashrc

# Add colcon build tab-completion
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> /home/$USERNAME/.bashrc
