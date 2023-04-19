[Back to [Tutorials](../../tutorials.md) list]

# Install using Docker

Docker images that include the neccessary software and dependencies have been created for convenience. 

## Requirements

1. Install Docker using [installation instructions](https://docs.docker.com/engine/install/ubuntu/).

1. Complete the [Linux Postinstall steps](https://docs.docker.com/engine/install/linux-postinstall/) to allow you to manage Docker as a non-root user.

1. If you have an NVIDIA graphics card, it can help speed up rendering. Install [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).

## Build

1. Clone the mbari_wec repository to download the latest Dockerfile.
        ```
        git clone https://github.com/osrf/mbari_wec.git
        cd ~/mbari_wec/docker/
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
        ./join.bash buoy
        ```

## Quick start

Quick start scripts are provided in the home directory:

This sources the compiled workspace:
    ```
    ./setup.bash
    ```

This sources the compiled workspace and launches the simulation:
    ```
    ./run_simulation.bash
    ```

## Run an example to test

1. In a new terminal (whether on host machine or in Docker container), source the workspace
        ```
        . ~/mbari_wec_ws/install/setup.sh
        ```

1. Launch the simulation
        ```
        ros2 launch buoy_gazebo mbari_wec.launch.py
        ```

The simulation software should now be available.  To run and test, proceed to the [Run the Simulator](../../../tutorials/#running-the-simulator) tutorial series. 


[Back to [Tutorials](../../tutorials.md) list]
