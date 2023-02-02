Docker images that include the neccessary software and dependencies have been created for convenience. 

##### Requirements

1. Install Docker using [installation instructions.](https://docs.docker.com/engine/install/ubuntu/).

1. Install [nvidia-docker](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker).

1. Complete the [Linux Postinstall steps](https://docs.docker.com/engine/install/linux-postinstall/) to allow you to manage Docker as a non-root user.

1. Install `rocker` by `sudo apt-get install python3-rocker`.

##### Usage

1. Clone the buoy_entrypoint repository to download the latest Dockerfile.

```
$ git clone https://github.com/osrf/buoy_entrypoint.git
$ cd ~/buoy_entrypoint/docker/
```

1. Build the docker image

```
$ ./build.bash buoy
```

1. Run the container

```
$ ./run.bash [-d|s] buoy:latest
```

    where `./run.bash` option:
    * -d     Use for development with host system volume mount
    * -s     Simulation purposes only

    The development use case enables to either use host system home directory for user's custom workspace, or a fresh clone inside the docker container. If using host system workspace, follow the [On Host System](#on-host-system) instructions to build and run the project in the container.
    Regardless the script option, project source files can be found in `/tmp/buoy_ws/' in the container. Note that any changes to files in the container will have limited scope.

1. To have another window running the same docker container, run this command in a new terminal:

```
$ ./join.bash buoy_latest_runtime
```

> The build and run bash scripts are a wrapper around rocker, checkout its [documentation](https://github.com/osrf/rocker) for additional options.

##### Run an example to test

Inside the docker container, run:

```
$ gz sim mbari_wec.sdf -r
```

The simulation software should now be available.  To run and test, proceed to the [Run the Simulator](../../../tutorials/#running-the-simulator) tutorial series. 