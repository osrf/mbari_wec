This is the entrypoint for the wave energy harvesting buoy project.

## Repositories

These are the repositories for the project:

* [buoy_msgs](https://github.com/osrf/buoy_msgs): ROS 2 messages, interface API, and examples for
  receiving and sending data to a physical or simulated buoy.
    * [buoy_interfaces](https://github.com/osrf/buoy_msgs/tree/main/buoy_api_cpp): ROS 2 messages
      to recieve and send data to a physical or simulated buoy
    * [buoy_api_cpp](https://github.com/osrf/buoy_msgs/tree/main/buoy_api_cpp): C++ Interface to
      MBARI Power Buoy including Controller examples to run against a physical or simulated buoy.
    * [buoy_api_py](https://github.com/osrf/buoy_msgs/tree/main/buoy_api_py): Python Interface to
      MBARI Power Buoy including Controller examples to run against a physical or simulated buoy.
* [buoy_sim](https://github.com/osrf/buoy_sim)
    * [buoy_description](https://github.com/osrf/buoy_description/tree/main/buoy_description):
      Buoy model description.
    * [buoy_gazebo](https://github.com/osrf/buoy_description/tree/main/buoy_gazebo):
      Gazebo plugins, worlds and launch files to simulate the buoy.

## Install
### On Host System
##### Requirements
At the moment, only source installation is supported. Use Ubuntu Jammy.

1. Install [ROS 2 Humble](https://docs.ros.org/en/humble/index.html)

Buoy Sim is tested against cyclonedds rmw implementation (default changed from Galactic to Humble)
```
sudo apt install -y ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

2. Install [Gazebo Garden](https://gazebosim.org/docs/garden)

Currently, in order to use added mass, it is necessary to build gz-sim Garden from source.

When building from source, it is necessary to export the `PYTHONPATH` for gz-math python bindings when building buoy_sim
```
export PYTHONPATH=$PYTHONPATH:<path to your gz-sim workspace>/install/lib/python`
```

See [gz-math Python Get Started tutorial](https://github.com/gazebosim/gz-math/blob/gz-math7/tutorials/pythongetstarted.md). This step is needed until `PYTHONPATH` is automatically exported upstream, tracked in [this issue](https://github.com/osrf/buoy_sim/issues/81)


3. Install necessary tools

    ```
    sudo apt install python3-vcstool python3-colcon-common-extensions python3-pip git wget
    ```

##### Usage

1. Create a workspace, for example:

    ```
    mkdir -p ~/buoy_ws/src
    cd ~/buoy_ws/src
    ```

1. Clone all source repos with the help of `vcstool`:

    ```
    wget https://raw.githubusercontent.com/osrf/buoy_entrypoint/main/buoy_all.yaml
    vcs import < buoy_all.yaml
    cd ~/buoy_ws
    ```

1. Set the Gazebo version to Garden. This is needed because we're not using an
   official ROS + Gazebo combination:

    ```
    export GZ_VERSION=garden
    ```

1. Install ROS dependencies

    ```
    sudo pip3 install -U rosdep
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y -i
    ```

1. Build and install

    ```
    source /opt/ros/humble/setup.bash
    cd ~/buoy_ws
    colcon build
    ```

##### Run

1. Source the workspace

    `. ~/buoy_ws/install/setup.sh`

1. Launch the simulation

    `ros2 launch buoy_gazebo mbari_wec.launch.py`


### Using docker
##### Requirements

1. Install Docker using [installation instructions.](https://docs.docker.com/engine/install/ubuntu/).

1. Install [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).

1. Complete the [Linux Postinstall steps](https://docs.docker.com/engine/install/linux-postinstall/) to allow you to manage Docker as a non-root user.

1. Install `rocker` by `sudo apt-get install python3-rocker`.

##### Usage

1. Clone the buoy_entrypoint repository to download the latest Dockerfile.

    ```
    git clone https://github.com/osrf/buoy_entrypoint.git
    cd ~/buoy_entrypoint/docker/
    ```

1. Build the docker image

    ```
    ./build.bash buoy
    ```

1. Run the container

    ```
    ./run.bash [-d|s] buoy:latest
    ```
    where `./run.bash` option:
    * -d     Use for development with host system volume mount
    * -s     Simulation purposes only

    The development use case enables to either use host system home directory for user's custom workspace, or a fresh clone inside the docker container. If using host system workspace, follow the [On Host System](#on-host-system) instructions to build and run the project in the container.
    Regardless the script option, project source files can be found in `/tmp/buoy_ws/' in the container. Note that any changes to files in the container will have limited scope.

1. To have another window running the same docker container, run this command in a new terminal:

   ```
   ./join.bash buoy_latest_runtime
   ```

> The build and run bash scripts are a wrapper around rocker, checkout its [documentation](https://github.com/osrf/rocker) for additional options.

##### Run

Inside the docker container, run:

```
gz sim mbari_wec.sdf -r
```
