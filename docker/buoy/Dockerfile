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

# Setup and install ignition fortress
RUN /bin/sh -c 'wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null' \
  && apt update \
  && apt install -y ignition-fortress \
  && rm -rf /var/lib/apt/lists/* \
  && apt clean

# Install colcon and rosdep
RUN apt update \
 && apt install -y \
    python3-colcon-common-extensions \
    python3-colcon-core \
    python3-rosdep \
    python3-vcstool \
 && apt clean

# Add the same user as outside the container
# Requires a docker build argument `USERNAME`
ARG USERNAME

# Setup working environment
ENV IGNITION_VERSION fortress

# When running a container start in the user's home folder
WORKDIR /home/${USERNAME}/

### TODO(quarkytale): creating and building not required for development
# Create project directory and import packages
RUN mkdir -p /home/${USERNAME}/buoy_ws/src \
  && cd /home/${USERNAME}/buoy_ws/src/ \
  && wget https://raw.githubusercontent.com/osrf/buoy_entrypoint/main/buoy_all.yaml \
  && vcs import --skip-existing < buoy_all.yaml

WORKDIR /home/${USERNAME}/buoy_ws
COPY . src/buoy_sim

# Install rosdep dependencies
RUN apt update \
  && rosdep update \
  && rosdep install --from-paths src --ignore-src -r -y -i \
  && rm -rf /var/lib/apt/lists/* \
  && apt clean

# Build the project
RUN /bin/bash -c 'source /opt/ros/${ROS_DISTRO}/setup.bash \
  && colcon build --cmake-clean-cache'

# Source workspaces, add it to the bashrc
RUN /bin/sh -c 'echo ". /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc' \
  && /bin/sh -c 'echo "./home/${USERNAME}/buoy_ws/install/setup.sh" >> ~/.bashrc'
