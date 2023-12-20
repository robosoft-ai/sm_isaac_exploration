#!/bin/bash
#
# Copyright (c) 2021-2022, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.

ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
# source $ROOT/utils/print_color.sh

function usage() {
    echo "Usage: run_dev.sh" {isaac_ros_dev directory path OPTIONAL}
    echo "Copyright (c) 2021-2022, NVIDIA CORPORATION."
}

# Read and parse config file if exists
#
# CONFIG_IMAGE_KEY (string, can be empty)

if [[ -f "${ROOT}/.isaac_ros_common-config" ]]; then
    . "${ROOT}/.isaac_ros_common-config"
fi

# if first argument exist
if [[ ! -z "$1" ]]; then
    ISAAC_ROS_DEV_DIR=`realpath "$1"`
fi

# if ISAASC_ROS_DEV_DIR is empty, set to ../..
if [[ -z "$ISAAC_ROS_DEV_DIR" ]]; then
    ISAAC_ROS_DEV_DIR=`realpath "$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )/../../"`
    if [[ ! -d "$ISAAC_ROS_DEV_DIR" ]]; then
        ISAAC_ROS_DEV_DIR=$(pwd)
    fi
    echo "isaac_ros_dev not specified, assuming $ISAAC_ROS_DEV_DIR"
else
    shift 1
fi


# if [[ -z "$ISAAC_ROS_DEV_DIR" ]]; then
#     ISAAC_ROS_DEV_DIR="$HOME/workspaces/isaac_ros-dev"
#     if [[ ! -d "$ISAAC_ROS_DEV_DIR" ]]; then
#         ISAAC_ROS_DEV_DIR=$(pwd)
#     fi
#     print_warning "isaac_ros_dev not specified, assuming $ISAAC_ROS_DEV_DIR"
# else
#     shift 1
# fi

ON_EXIT=()
function cleanup {
    for command in "${ON_EXIT[@]}"
    do
        $command
    done
}
trap cleanup EXIT

pushd . >/dev/null
cd $ROOT
ON_EXIT+=("popd")

# Prevent running as root.
if [[ $(id -u) -eq 0 ]]; then
    echo "This script cannot be executed with root privileges."
    echo "Please re-run without sudo and follow instructions to configure docker for non-root user if needed."
    exit 1
fi

# Check if user can run docker without root.
RE="\<docker\>"
if [[ ! $(groups $USER) =~ $RE ]]; then
    echo "User |$USER| is not a member of the 'docker' group and cannot run docker commands without sudo."
    echo "Run 'sudo usermod -aG docker \$USER && newgrp docker' to add user to 'docker' group, then re-run this script."
    echo "See: https://docs.docker.com/engine/install/linux-postinstall/"
    exit 1
fi

# Check if able to run docker commands.
if [[ -z "$(docker ps)" ]] ;  then
    echo "Unable to run docker commands. If you have recently added |$USER| to 'docker' group, you may need to log out and log back in for it to take effect."
    echo "Otherwise, please check your Docker installation."
    exit 1
fi

# Check if git-lfs is installed.
if [[ -z "$(git lfs)" ]] ; then
    echo "git-lfs is not insalled. Please make sure git-lfs is installed before you clone the repo."
    exit 1
fi

# Check if all LFS files are in place
git rev-parse &>/dev/null
if [[ $? -eq 0 ]]; then
    LFS_FILES_STATUS=$(cd $ISAAC_ROS_DEV_DIR && git lfs ls-files | cut -d ' ' -f2)
    for (( i=0; i<${#LFS_FILES_STATUS}; i++ )); do
        f="${LFS_FILES_STATUS:$i:1}"
        if [[ "$f" == "-" ]]; then
            echo "LFS files are missing. Please re-clone the repo after installing git-lfs."
            exit 1
        fi
    done
fi

# PLATFORM=".imp.$(uname -m)"

#*******IMAGE NAME*********
BASE_NAME="pibgeus/smacc2_isaac_exploration"
CONTAINER_NAME="$BASE_NAME-container"

# Remove any exited containers.
if [ "$(docker ps -a --quiet --filter status=exited --filter name=$CONTAINER_NAME)" ]; then
    docker rm $CONTAINER_NAME > /dev/null
fi

# Re-use existing container.
if [ "$(docker ps -a --quiet --filter status=running --filter name=$CONTAINER_NAME)" ]; then
    echo "Attaching to running container: $CONTAINER_NAME"
    docker exec -i -t -u admin --workdir /workspaces/isaac_ros-dev $CONTAINER_NAME /bin/bash $@
    exit 0
fi

# Build image
# IMAGE_KEY=ros2_humble
# if [[ ! -z "${CONFIG_IMAGE_KEY}" ]]; then
#     IMAGE_KEY=$CONFIG_IMAGE_KEY
# fi

# BASE_IMAGE_KEY=$PLATFORM.user
# if [[ ! -z "${IMAGE_KEY}" ]]; then
#     BASE_IMAGE_KEY=$PLATFORM.$IMAGE_KEY

#     # If the configured key does not have .user, append it last
#     if [[ $IMAGE_KEY != *".user"* ]]; then
#         BASE_IMAGE_KEY=$BASE_IMAGE_KEY.user
#     fi
# fi

# print "Building $(uname -m).imp.user base as image: $BASE_NAME using key .imp.$(uname -m).user"
# $ROOT/build_base_image.sh

if [ $? -ne 0 ]; then
    # echo "Failed to build base image: $BASE_NAME, aborting."
    exit 1
fi

# Map host's display socket to docker
DOCKER_ARGS+=("-v /tmp/.X11-unix:/tmp/.X11-unix")
DOCKER_ARGS+=("-v $HOME/.Xauthority:/home/admin/.Xauthority:rw")
DOCKER_ARGS+=("-e DISPLAY")
DOCKER_ARGS+=("-e NVIDIA_VISIBLE_DEVICES=all")
DOCKER_ARGS+=("-e NVIDIA_DRIVER_CAPABILITIES=all")
DOCKER_ARGS+=("-e FASTRTPS_DEFAULT_PROFILES_FILE=/usr/local/share/middleware_profiles/rtps_udp_profile.xml")
DOCKER_ARGS+=("-e ROS_DOMAIN_ID")

# if [[ $PLATFORM == "aarch64" ]]; then
#     DOCKER_ARGS+=("-v /usr/bin/tegrastats:/usr/bin/tegrastats")
#     DOCKER_ARGS+=("-v /tmp/argus_socket:/tmp/argus_socket")
#     DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusolver.so.11")
#     DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcusparse.so.11")
#     DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcurand.so.10")
#     DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libnvToolsExt.so")
#     DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcupti.so.11.4")
#     DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1:/usr/local/cuda-11.4/targets/aarch64-linux/lib/libcudla.so.1")
#     DOCKER_ARGS+=("-v /usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h:/usr/local/cuda-11.4/targets/aarch64-linux/include/nvToolsExt.h")
#     DOCKER_ARGS+=("-v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra")
#     DOCKER_ARGS+=("-v /usr/src/jetson_multimedia_api:/usr/src/jetson_multimedia_api")
#     DOCKER_ARGS+=("-v /opt/nvidia/nsight-systems-cli:/opt/nvidia/nsight-systems-cli")
#     DOCKER_ARGS+=("--pid=host")
#     DOCKER_ARGS+=("-v /opt/nvidia/vpi2:/opt/nvidia/vpi2")
#     DOCKER_ARGS+=("-v /usr/share/vpi2:/usr/share/vpi2")

#     # If jtop present, give the container access
#     if [[ $(getent group jtop) ]]; then
#         DOCKER_ARGS+=("-v /run/jtop.sock:/run/jtop.sock:ro")
#         JETSON_STATS_GID="$(getent group jtop | cut -d: -f3)"
#         DOCKER_ARGS+=("--group-add $JETSON_STATS_GID")
#     fi
# fi

# Optionally load custom docker arguments from file
DOCKER_ARGS_FILE="$ROOT/.isaac_ros_dev-dockerargs"
if [[ -f "$DOCKER_ARGS_FILE" ]]; then
    echo "Using additional Docker run arguments from $DOCKER_ARGS_FILE"
    readarray -t DOCKER_ARGS_FILE_LINES < $DOCKER_ARGS_FILE
    for arg in "${DOCKER_ARGS_FILE_LINES[@]}"; do
        DOCKER_ARGS+=($(eval "echo $arg | envsubst"))
    done
fi

echo "DOCKER ARGS: ${DOCKER_ARGS[@]}"
echo "ISAAC_ROS_DEV_DIR: $ISAAC_ROS_DEV_DIR"
echo "CONTAINER_NAME: $CONTAINER_NAME"

docker run -it --rm \
    --privileged \
    --network host \
    --ipc=host \
    ${DOCKER_ARGS[@]} \
    -v $ISAAC_ROS_DEV_DIR:/workspaces/isaac_ros-dev/src \
    -v /dev/*:/dev/* \
    -v /etc/localtime:/etc/localtime:ro \
    --name "$CONTAINER_NAME" \
    --runtime nvidia \
    --user="admin" \
    --entrypoint /usr/local/bin/scripts/workspace-entrypoint.sh \
    --workdir /workspaces/isaac_ros-dev \
    $BASE_NAME \
    /bin/bash
