This is the entrypoint for the wave energy harvesting buoy project.

See [documentation here](https://osrf.github.io/mbari_wec/main/).

And MBARI-WEC in action using Gazebo simulator here:

![](docs/docs/images/buoy_sim.gif)


# Simulation Repositories

These are the repositories for the project:

* [mbari_wec_utils](https://github.com/osrf/mbari_wec_utils): ROS 2 messages, interface API, and examples for
  receiving and sending data to a physical or simulated buoy.
    * [buoy_interfaces](https://github.com/osrf/mbari_wec_utils/tree/main/buoy_api_cpp): ROS 2 messages
      to recieve and send data to a physical or simulated buoy
    * [buoy_api_cpp](https://github.com/osrf/mbari_wec_utils/tree/main/buoy_api_cpp): C++ Interface to
      MBARI Power Buoy including Controller examples to run against a physical or simulated buoy.
    * [buoy_api_py](https://github.com/osrf/mbari_wec_utils/tree/main/buoy_api_py): Python Interface to
      MBARI Power Buoy including Controller examples to run against a physical or simulated buoy.
* [mbari_wec_gz](https://github.com/osrf/mbari_wec_gz)
    * [buoy_description](https://github.com/osrf/mbari_wec_gz/tree/main/buoy_description):
      Buoy model description.
    * [buoy_gazebo](https://github.com/osrf/mbari_wec_gz/tree/main/buoy_gazebo):
      Gazebo plugins, worlds and launch files to simulate the buoy.

# Interfaces and Examples

There are two GitHub
[template](https://docs.github.com/en/repositories/creating-and-managing-repositories/creating-a-repository-from-a-template)
repositories set up (cpp/python) for a quick start on writing a
custom controller utilizing
[buoy_api_cpp](https://github.com/osrf/mbari_wec_utils/tree/main/buoy_api_cpp) and
[buoy_api_py](https://github.com/osrf/mbari_wec_utils/tree/main/buoy_api_py). Please see
[cpp examples](https://github.com/osrf/mbari_wec_utils/tree/main/buoy_api_cpp/examples) and
[python examples](https://github.com/osrf/mbari_wec_utils/tree/main/buoy_api_py/buoy_api/examples) for example
controller implementations.

* [mbari_wec_template_cpp](https://github.com/mbari-org/mbari_wec_template_cpp)
* [mbari_wec_template_py](https://github.com/mbari-org/mbari_wec_template_py)

# Install
## On Host System
### Requirements
At the moment, MBARI WEC is supported by source installation only. Use Ubuntu Jammy (22.04).

1. Install [ROS 2 Humble](https://docs.ros.org/en/humble/index.html) (preferably binary installation)

    MBARI WEC is tested against cyclonedds rmw implementation (default changed from Galactic to Humble)
    ```
    sudo apt install -y ros-humble-rmw-cyclonedds-cpp
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ```

2. Install [Gazebo Garden](https://gazebosim.org/docs/garden) (preferably binary installation)

    Installing the binaries for Gazebo is recommended, but if building Gazebo Garden from source, it is necessary to export the `PYTHONPATH` for gz-math python bindings when building the mbari_wec_gz repository below:
    ```
    export PYTHONPATH=$PYTHONPATH:<path to your gz-sim workspace>/install/lib/python
    ```

    See [gz-math Python Get Started tutorial](https://github.com/gazebosim/gz-math/blob/gz-math7/tutorials/pythongetstarted.md). This step is needed until `PYTHONPATH` is automatically exported upstream, tracked in [this issue](https://github.com/osrf/mbari_wec_gz/issues/81)


3. Install necessary tools

    ```
    sudo apt install python3-vcstool python3-colcon-common-extensions python3-pip git wget
    ```

4. Install necessary libraries
    ```
    curl -s --compressed "https://hamilton8415.github.io/ppa/KEY.gpg" | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/ppa.gpg >/dev/null
    sudo curl -s --compressed -o /etc/apt/sources.list.d/my_list_file.list "https://hamilton8415.github.io/ppa/my_list_file.list"
    sudo apt update
    sudo apt install libfshydrodynamics=1.3.1
    ```


### Build

1. Create a workspace, for example:

    ```
    mkdir -p ~/mbari_wec_ws/src
    cd ~/mbari_wec_ws/src
    ```

2. Clone all source repos with the help of `vcstool`:

    ```
    wget https://raw.githubusercontent.com/osrf/mbari_wec/main/mbari_wec_all.yaml
    vcs import < mbari_wec_all.yaml
    cd ~/mbari_wec_ws
    ```

1. Set the Gazebo version to Garden. This is needed because we're not using an
   official ROS + Gazebo combination:

    ```
    export GZ_VERSION=garden
    ```

2. Install ROS dependencies

    ```
    sudo pip3 install -U rosdep
    sudo rosdep init
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y -i
    ```

3. Build and install

    ```
    source /opt/ros/humble/setup.bash
    cd ~/mbari_wec_ws
    colcon build
    ```

## Using Docker
### Requirements

1. Install Docker using [installation instructions](https://docs.docker.com/engine/install/ubuntu/).

2. Complete the [Linux Postinstall steps](https://docs.docker.com/engine/install/linux-postinstall/) to allow you to manage Docker as a non-root user.

3. If you have an NVIDIA graphics card, it can help speed up rendering. Install
   [nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).
    1. Follow [Installation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#with-apt-ubuntu-debian) steps for Ubuntu (currently 3 steps)
    2. Skip down to [Configuration](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#configuring-docker) and run `nvidia-ctk` to set up the nvidia container runtime

### Use Existing Image on DockerHub
MBARI maintains Docker images for the two most recent releases on their DockerHub:
  - `mbari/mbari_wec:latest`
  - `mbari/mbari_wec:previous`

1. Get `run.bash` script.

   ```
   git clone -b main https://github.com/osrf/mbari_wec.git
   cd ~/mbari_wec/docker/
   ```
   Or
   ```
   wget https://raw.githubusercontent.com/osrf/mbari_wec/main/docker/run.bash
   chmod +x run.bash
   ```

3. Run the container

   If you have an NVIDIA graphics card
   ```
   ./run.bash mbari/mbari_wec:latest
   ```
   Otherwise
   ```
   ./run.bash mbari/mbari_wec:latest --no-nvidia
   ```

### Build from Dockerfile
An alternative to using the images from MBARI's DockerHub would be to build from a Dockerfile. This is convenient if you would like to make any changes.

1. Clone the mbari_wec repository to download the latest Dockerfile.

   ```
   git clone -b main https://github.com/osrf/mbari_wec.git
   cd ~/mbari_wec/docker/
   ```

2. Build the docker image

   If you have an NVIDIA graphics card
   ```
   ./build.bash mbari_wec
   ```
   Otherwise
   ```
   ./build.bash mbari_wec --no-nvidia
   ```

3. Run the container

   If you have an NVIDIA graphics card
   ```
   ./run.bash mbari_wec_nvidia
   ```
   Otherwise
   ```
   ./run.bash mbari_wec_no_nvidia --no-nvidia
   ```

4. To have another window running the same docker container, run this command in a new terminal:

   ```
   ./join.bash <name of image>
   ```
   where the name of the image is one of `mbari_wec_nvidia` or `mbari_wec_no_nvidia`

### Quick start

Quick start scripts are provided in the home directory:

This sources the compiled workspace:
```
. setup.bash
```

This sources the compiled workspace and launches the simulation:
```
./run_simulation.bash
```

Logs from the run will be saved to `/logs` in the container which is mapped
to `~/mbari_wec/docker/logs` on the host. These logs will be in the same CSV format as generated by
the physical buoy. You may also collect rosbags in the `/logs` folder in the container. The rosbags
collected in the sim will also be the same ROS 2 messages collected on the physical buoy.

# Run

1. In a new terminal (whether on host machine or in Docker container), source the workspace

   ```
   . ~/mbari_wec_ws/install/setup.bash
   ```

1. Set `SDF_PATH` to allow `robot_state_publisher` to parse the robot description from the sdformat model:

   ```
   export SDF_PATH=$GZ_SIM_RESOURCE_PATH
   ```

1. Launch the simulation

   ```
   ros2 launch buoy_gazebo mbari_wec.launch.py
   ```

# For maintainers only: To upload to DockerHub

Make sure you have permissions to push to the
[MBARI organization on DockerHub](https://hub.docker.com/u/mbari).
This permission is given by the MBARI administrator.

Build the `mbari_wec` Docker image, as detailed above.

Find the image ID for `mbari_wec`:
```
docker images
```

Tag the image with the destination name:
```
docker tag <IMAGE ID> mbari/mbari_wec:latest
```

Push to the [`mbari/mbari_wec` public image](https://hub.docker.com/r/mbari/mbari_wec).
```
docker push mbari/mbari_wec:latest
```

You may have to log in for it to recognize your permissions:
```
docker login
```

# Publications
```
Dizon, Chris, Ryan Coe, Andrew Hamilton, Dominic Forbush, Michael Anderson, Ted Brekken, and Giorgio
Bacelli. 2024. "Analysis on Evaluations of Monterey Bay Aquarium Research Institute’s Wave Energy
Converter’s Field Data Using WEC-Sim and Gazebo: A Simulation Tool Comparison" Applied Sciences 14,
no. 23: 11169. https://doi.org/10.3390/app142311169
```
