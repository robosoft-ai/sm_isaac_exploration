#!/bin/bash
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

echo $DIR
ROOT_SRC_DIRECTORY=`realpath $DIR/../../`
echo $ROOT_SRC_DIRECTORY

cd $ROOT_SRC_DIRECTORY

echo "---------------------------"
echo "building isaac_ros_common base image"



PLATFORM="$(uname -m)"

BASE_NAME="isaac_ros_dev-$PLATFORM"
CONTAINER_NAME="$BASE_NAME-container"

# Remove any exited containers.
if [ "$(docker ps -a --quiet --filter status=exited --filter name=$CONTAINER_NAME)" ]; then
    docker rm $CONTAINER_NAME > /dev/null
fi

## Re-use existing container.
#if [ "$(docker ps -a --quiet --filter status=running --filter name=$CONTAINER_NAME)" ]; then
#    echo "Attaching to running container: $CONTAINER_NAME"
#    docker exec -i -t -u admin --workdir /workspaces/isaac_ros-dev $CONTAINER_NAME /bin/bash $@
#    exit 0
#fi

IMAGE_KEY=ros2_humble
if [[ ! -z "${CONFIG_IMAGE_KEY}" ]]; then
    IMAGE_KEY=$CONFIG_IMAGE_KEY
fi

BASE_IMAGE_KEY=$PLATFORM.user
if [[ ! -z "${IMAGE_KEY}" ]]; then
    BASE_IMAGE_KEY=$PLATFORM.$IMAGE_KEY

    # If the configured key does not have .user, append it last
    if [[ $IMAGE_KEY != *".user"* ]]; then
        BASE_IMAGE_KEY=$BASE_IMAGE_KEY.user
    fi
fi


# Build image
IMAGE_KEY=ros2_humble
if [[ ! -z "${CONFIG_IMAGE_KEY}" ]]; then
    IMAGE_KEY=$CONFIG_IMAGE_KEY
fi

BASE_IMAGE_KEY=$PLATFORM.user
if [[ ! -z "${IMAGE_KEY}" ]]; then
    BASE_IMAGE_KEY=$PLATFORM.$IMAGE_KEY

    # If the configured key does not have .user, append it last
    if [[ $IMAGE_KEY != *".user"* ]]; then
        BASE_IMAGE_KEY=$BASE_IMAGE_KEY.user
    fi
fi

#print_info "Building $BASE_IMAGE_KEY base as image: $BASE_NAME using key $BASE_IMAGE_KEY"

# build isaac_common base image
$ROOT_SRC_DIRECTORY/isaac_ros_common/scripts/build_base_image.sh $BASE_IMAGE_KEY $BASE_NAME '' '' ''

                     
echo "---------------------------"
cd $ROOT_SRC_DIRECTORY/..
echo "building sm_isaac_exploration base image"
# build sm_isaac_exploration image
docker build -t smacc2_isaac -f src/sm_isaac_exploration/docker/Dockerfile --build-arg architecture_=aarch64 --progress=plain .
