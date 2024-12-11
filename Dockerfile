FROM ros:humble-ros-core

# Install dependencies
RUN apt-get update \
  && apt-get install -y \
    ros-humble-rmf-dev \
    python3-rosdep \
    python3-vcstool \
    python3-colcon-common-extensions \
    qtbase5-dev qtchooser qt5-qmake qtbase5-dev-tools \
    python3-pip \
  && pip3 install flask-socketio fastapi uvicorn nudged websocket-client pyjwt \
  && rm -rf /var/lib/apt/lists/*

# Create ROS 2 workspace.
WORKDIR /fleet_adapter_kabam_ws
RUN mkdir src

# Install ROS 2 dependencies.
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get upgrade -y \
    && rosdep init \
    && rosdep update --rosdistro $ROS_DISTRO \
    && rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -yr \
    && rm -rf /var/lib/apt/lists/*

# Build ROS 2 package, fleet_adapter_kabam
COPY ./ src/fleet_adapter_kabam
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
  && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release 

# Include source file to default /ros_entrypoint.sh
RUN sed -i '$isource "/fleet_adapter_kabam_ws/install/setup.bash"' /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]