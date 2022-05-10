FROM ros:galactic-ros-base
ENV DEBIAN_FRONTEND=noninteractive

# Useful tools
RUN apt update \
 && apt install -y \
        apt-utils \
        build-essential \
        curl \
        lsb-release \
        wget \
 && apt clean

# Setup and install ignition fortress, colcon and rosdep
RUN /bin/sh -c 'wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null' \
  && apt update \
  && apt install -y ignition-fortress \
     python3-colcon-common-extensions \
     python3-colcon-core \
     python3-rosdep \
     python3-vcstool \
  && rm -rf /var/lib/apt/lists/* \
  && apt clean

# Setup working environment
ENV IGNITION_VERSION fortress

# When running a container start in the user's home folder
WORKDIR /home/${USERNAME}/

# Add the same user as outside the container
# Requires a docker build argument `USERNAME`
ARG USERNAME
RUN useradd -d /home/${USERNAME}/ -m -s /bin/bash $USERNAME

# Create project directory, set permissions and import packages
RUN mkdir -p /tmp/buoy_ws/src \
  && chown -R $USERNAME:$USERNAME /tmp/buoy_ws \
  && adduser $USERNAME sudo \
  && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME \
  && cd /tmp/buoy_ws/src/ \
  && wget https://raw.githubusercontent.com/osrf/buoy_entrypoint/main/buoy_all.yaml \
  && vcs import --skip-existing < buoy_all.yaml \
  && chown -R $USERNAME:$USERNAME .

# Run the commands below as non-root
# USER ${USERNAME}

# Install rosdep dependencies
RUN sudo apt update \
  && cd /tmp/buoy_ws \
  && rosdep update \
  && rosdep install --from-paths src --ignore-src -r -y -i \
  && sudo rm -rf /var/lib/apt/lists/* \
  && sudo apt clean

# Build the project
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash \
  && cd /tmp/buoy_ws \
  && colcon build'

# Source workspaces, add it to the bashrc
RUN /bin/sh -c 'echo ". /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc' \
  && /bin/sh -c 'echo ". /home/${USERNAME}/buoy_ws/install/setup.sh" >> /home/${USERNAME}/.bashrc'
