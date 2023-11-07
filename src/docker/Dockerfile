ARG architecture_
FROM isaac_ros_dev-${architecture_}:latest
#FROM isaac_ros_dev-x86_64:latest

RUN sudo apt-get update && \
    sudo apt-get install -y xterm && \
    sudo apt-get install -y liblttng-ust-dev && \
    sudo apt-get install -y gdb && \
    sudo apt-get clean


USER admin
WORKDIR /workspaces/isaac_ros-dev

ADD . /workspaces/isaac_ros-dev

RUN sudo chown -R $USER:$USER /workspaces/isaac_ros-dev

# Download detect model
RUN bash -c  "cd /workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_detectnet && \
    ./scripts/setup_model.sh --height 720 --width 1280 --config-file resources/isaac_sim_config.pbtxt"

# ENTRYPOINT [ "/bin/bash" ]

RUN bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --packages-up-to sm_isaac_exploration --cmake-args -DCMAKE_BUILD_TYPE=Debug"
ENTRYPOINT [ "/bin/bash" ]
