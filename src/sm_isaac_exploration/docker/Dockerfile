ARG architecture_
FROM isaac_ros_dev-${architecture_}:latest
#FROM isaac_ros_dev-x86_64:latest

RUN sudo apt-get update && \
    sudo apt-get install -y xterm && \
    sudo apt-get install -y liblttng-ust-dev && \
    sudo apt-get install -y gdb && \
    sudo apt-get install -y cmake && \
    sudo apt-get clean


ENV USER=admin
WORKDIR /workspaces/isaac_ros-dev


COPY --chown=$USER:$USER . /workspaces/isaac_ros-dev



# Download detect model
RUN bash -c  "cd /workspaces/isaac_ros-dev/src/isaac_ros_object_detection/isaac_ros_detectnet && \
    ./scripts/setup_model.sh --height 720 --width 1280 --config-file resources/isaac_sim_config.pbtxt"

# ENTRYPOINT [ "/bin/bash" ]

RUN bash -c "source /opt/ros/humble/setup.bash && cd /workspaces/isaac_ros-dev/ && colcon build --symlink-install --packages-up-to sm_isaac_exploration --cmake-args -DCMAKE_BUILD_TYPE=Release"
ENTRYPOINT [ "/bin/bash" ]
