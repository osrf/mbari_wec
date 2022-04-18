FROM ros:galactic-ros-base
ENV DEBIAN_FRONTEND=noninteractive

# Set Ubuntu release
ARG RELEASE=focal

# Set ROS distribution
ARG DIST=galactic


# Useful tools
RUN apt update \
 && apt install -y \
        apt-utils \
        atop \
        build-essential \
        cmake \
        cppcheck \
        curl \
        gdb \
        git \
        gnutls-bin \
        gnupg \
        g++-8 \        
        lsb-release \
        net-tools \
        pkg-config \        
        python3-dbg \
        python3-empy \
        python3-numpy \
        python3-pip \
        python3-scipy \
        python3-setuptools \
        python3-venv \
        qtbase5-dev \
        ruby \
        software-properties-common \
        sudo \
        vim \
        wget \
        xvfb \
 && apt clean

# Setup and install ignition fortress
RUN /bin/sh -c 'wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null'
RUN apt update \
  && apt install -y ignition-fortress \
  && rm -rf /var/lib/apt/lists/* \
  && apt clean

# Setup and install ROS2
RUN /bin/sh -c 'curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg' \
  && /bin/sh -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null'
RUN apt update \
  && apt install -y ros-${DIST}-desktop \
  && rm -rf /var/lib/apt/lists/* \
  && apt clean -qq

# Install colcon and rosdep
RUN /bin/sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu ${RELEASE} main" > /etc/apt/sources.list.d/ros2.list' \
 && apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
 && apt update \
 && apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-vcstool \
 && apt clean

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`
ARG user_id
ENV USERNAME developer
RUN useradd -U --uid ${user_id} -ms /bin/bash $USERNAME \
 && echo "$USERNAME:$USERNAME" | chpasswd \
 && adduser $USERNAME sudo \
 && echo "$USERNAME ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USERNAME

# Commands below run as the developer user
USER $USERNAME

# When running a container start in the user's home folder
WORKDIR /home/$USERNAME

# Setup working environment
ENV IGNITION_VERSION fortress

# Create project directory and import packages
RUN mkdir -p /home/$USERNAME/buoy_ws/src \
  && cd /home/$USERNAME/buoy_ws/src/ \
  && vcs import --skip-existing < ~/buoy_entrypoint/buoy_all.yaml

WORKDIR /home/$USERNAME/buoy_ws

# Install rosdep dependencies and build the project
RUN cd /home/$USERNAME/buoy_ws/ \
  &&  rosdep init \
  &&  rosdep update \
  &&  rosdep install --from-paths src --ignore-src -r -y -i \
  && colcon build --cmake-clean-cache

# Source workspaces
RUN source /opt/ros/${DIST}/setup.bash \
  && source /home/$USERNAME/buoy_ws/install/setup.bash
# Or add it to the bashrc
# RUN /bin/sh -c 'echo ". /opt/ros/galactic/setup.bash" >> ~/.bashrc' \
#  && /bin/sh -c 'echo ". ~/buoy_ws/install/setup.sh" >> ~/.bashrc'
