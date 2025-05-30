#!/usr/bin/env bash

#
# Copyright (C) 2023 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
#

# Runs a docker container with the image created by build.bash
# Requires:
#   docker
#   an X server
# Optional:
#   nvidia-docker
#   A joystick mounted to /dev/input/js0 or /dev/input/js1

if [ $# -lt 1 ]
then
    echo "Usage: $0 <docker image> [<dir with workspace> ...]"
    exit 1
fi

# Default to NVIDIA
#DOCKER_OPTS="--runtime=nvidia"
DOCKER_OPTS="--gpus all --env NVIDIA_DRIVER_CAPABILITIES=all"

# Parse and remove args
PARAMS=""
while (( "$#" )); do
  case "$1" in
    --no-nvidia)
        DOCKER_OPTS=""
      shift
      ;;
    -*|--*=) # unsupported flags
      echo "Error: Unsupported flag $1" >&2
      exit 1
      ;;
    *) # preserve positional arguments
      PARAMS="$PARAMS $1"
      shift
      ;;
  esac
done
# set positional arguments in their proper place
eval set -- "$PARAMS"

#IMG=$(basename $1)
# Use this for image repos with a slash "/" in the name
IMG=$1

ARGS=("$@")
WORKSPACES=("${ARGS[@]:1}")

# Make sure processes in the container can connect to the x server
# Necessary so gazebo can create a context for OpenGL rendering (even headless)
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]
then
    xauth_list=$(xauth nlist $DISPLAY | sed -e 's/^..../ffff/')
    if [ ! -z "$xauth_list" ]
    then
        touch $XAUTH
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

# Share your vim settings.
VIMRC=~/.vimrc
if [ -f $VIMRC ]
then
  DOCKER_OPTS="$DOCKER_OPTS -v $VIMRC:/home/developer/.vimrc:ro"
fi

# Share your custom terminal setup commands
GITCONFIG=~/.gitconfig
DOCKER_OPTS="$DOCKER_OPTS -v $GITCONFIG:/home/developer/.gitconfig:ro"

for WS_DIR in ${WORKSPACES[@]}
do
  WS_DIRNAME=$(basename $WS_DIR)
  if [ ! -d $WS_DIR/src ]
  then
    echo "Other! $WS_DIR"
    DOCKER_OPTS="$DOCKER_OPTS -v $WS_DIR:/home/developer/other/$WS_DIRNAME"
  else
    echo "Workspace! $WS_DIR"
    DOCKER_OPTS="$DOCKER_OPTS -v $WS_DIR/src:/home/developer/workspaces/src"
  fi
done

mkdir -p $PWD/logs  # for pbloghome

# Mount extra volumes if needed.
# E.g.:
# -v "/opt/sublime_text:/opt/sublime_text" \

# --ipc=host and --network=host are needed for no-NVIDIA Dockerfile to work
docker run -it \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XAUTHORITY=$XAUTH \
  -v "$XAUTH:$XAUTH" \
  -v "/tmp/.X11-unix:/tmp/.X11-unix" \
  -v "/etc/localtime:/etc/localtime:ro" \
  -v "/dev:/dev" \
  -v "$PWD/logs:/logs" \
  --privileged \
  --rm \
  --security-opt seccomp=unconfined \
  --ipc=host \
  --network=host \
  $DOCKER_OPTS \
  $IMG
