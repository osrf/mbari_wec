This is the entrypoint for the wave energy harvesting buoy project.

## Repositories

These are the repositories for the project:

* [buoy_msgs](https://github.com/osrf/buoy_msgs): ROS 2 messages to receive
  and send data to a physical or simulated buoy.
* [buoy_examples](https://github.com/osrf/buoy_examples): Controller examples
  to run against a physical or simulated buoy.
* [buoy_sim](https://github.com/osrf/buoy_sim)
    * [buoy_description](https://github.com/osrf/buoy_description/tree/main/buoy_description):
      Buoy model description.
    * [buoy_gazebo](https://github.com/osrf/buoy_description/tree/main/buoy_gazebo):
      Gazebo plugins, worlds and launch files to simulate the buoy.

## Install
### On Host System
At the moment, only source installation is supported. Use Ubuntu Focal.

1. Install [ROS 2 Galactic](https://docs.ros.org/en/galactic/index.html)

1. Install [Gazebo Fortress](https://ignitionrobotics.org/docs/fortress)

1. Install necessary tools

    `sudo apt install python3-vcstool python3-colcon-common-extensions git wget`

1. Create a workspace, for example:

    ```
    mkdir -p ~/buoy_ws/src
    cd ~/buoy_ws/src
    ```

1. Clone all source repos with the help of `vcstool`:

    ```
    wget https://raw.githubusercontent.com/osrf/buoy_entrypoint/main/buoy_all.yaml
    vcs import < buoy_all.yaml
    ```

1. Set the Gazebo version to Fortress. This is needed because we're not using an
   official ROS + Gazebo combination:

    ```
    export IGNITION_VERSION=fortress
    ```

1. Install ROS dependencies

    ```
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y -i
    ```

1. Build and install

    ```
    cd ~/buoy_ws
    colcon build
    ```

### Using docker
#### Requirements
1. Install Docker using [installation instructions.](https://docs.docker.com/engine/install/ubuntu/)
1. Complete the [Linux Postinstall steps](https://docs.docker.com/engine/install/linux-postinstall/) to allow you to manage Docker as a non-root user.
1. Install `rocker` by `sudo apt-get install python3-rocker`.

#### Usage
Clone the buoy_entrypoint repository to download the latest Dockerfile.
    ```
    git clone https://github.com/osrf/buoy_entrypoint.git
    cd ~/buoy_entrypoint/docker/
    ./build.bash buoy
    ./run.bash [-d|s] buoy:latest
    ```
where `./run.bash` option:
1. -d     Use for development with host system volume mount
1. -s     Simulation purposes only

## Run

1. Source the workspace

    `. ~/buoy_ws/install/setup.sh`

1. Launch the simulation

    `ros2 launch buoy_gazebo buoy.launch.py`

