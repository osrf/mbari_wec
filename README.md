This is the entrypoint for the wave energy harvesting buoy project.

See [documentation here](https://osrf.github.io/buoy_entrypoint).

## Simulation Repositories

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

## Interfaces and Examples

There are two GitHub
[template](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-repository-from-a-template)
repositories set up (cpp/python) for a quick start on writing a
custom controller utilizing
[buoy_api_cpp](https://github.com/osrf/buoy_msgs/tree/main/buoy_api_cpp) and
[buoy_api_py](https://github.com/osrf/buoy_msgs/tree/main/buoy_api_py). Please see
[cpp examples](https://github.com/osrf/buoy_msgs/tree/main/buoy_api_cpp/examples) and
[python examples](https://github.com/osrf/buoy_msgs/tree/main/buoy_api_py/buoy_api/examples) for example
controller implementations.

* [mbari_wec_template_cpp](https://github.com/mbari-org/mbari_wec_template_cpp)
* [mbari_wec_template_py](https://github.com/mbari-org/mbari_wec_template_py)

## Install
### On Host System
#### Requirements

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

#### Build

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

### Using docker
#### Requirements

1. Install Docker using [installation instructions](https://docs.docker.com/engine/install/ubuntu/).

1. Complete the [Linux Postinstall steps](https://docs.docker.com/engine/install/linux-postinstall/) to allow you to manage Docker as a non-root user.

1. If you have an NVIDIA graphics card, it can help speed up rendering. Install [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).

#### Build

1. Clone the buoy_entrypoint repository to download the latest Dockerfile.

   ```
   git clone https://github.com/osrf/buoy_entrypoint.git
   cd ~/buoy_entrypoint/docker/
   ```

1. Build the docker image

   If you have an NVIDIA graphics card
   ```
   ./build.bash nvidia_opengl_ubuntu22
   ./build.bash buoy
   ```
   Otherwise
   ```
   ./build.bash buoy --no-nvidia
   ```

1. Run the container

   If you have an NVIDIA graphics card
   ```
   ./run.bash buoy
   ```
   Otherwise
   ```
   ./run.bash buoy --no-nvidia
   ```

1. To have another window running the same docker container, run this command in a new terminal:

   ```
   ./join.bash buoy_latest_runtime
   ```

#### Quick start

Quick start scripts are provided in the home directory:

This sources the compiled workspace:
```
./setup.bash
```

This sources the compiled workspace and launches the simulation:
```
./run_simulation.bash
```

## Run

1. In a new terminal (whether on host machine or in Docker container), source the workspace

   ```
   . ~/buoy_ws/install/setup.sh
   ```

1. Launch the simulation

   ```
   ros2 launch buoy_gazebo mbari_wec.launch.py
   ```
