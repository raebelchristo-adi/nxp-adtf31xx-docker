# Use the official ROS Humble base image for ARM v8
FROM arm64v8/ros:humble-ros-core AS base

# Install necessary dependencies
RUN apt-get update && apt-get install -y \
    software-properties-common \
    && add-apt-repository universe \
    && apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    ros-humble-ament-cmake \
    python3-rosdep \
    curl \
    gnupg2 \
    vim \
    git \
    v4l-utils \
    lsb-release \
    build-essential \
    cmake \
    pkg-config \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list' \
    && apt-get update \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS environment
ENV ROS_DISTRO=humble
ENV ROS_ROOT=/opt/ros/$ROS_DISTRO
ENV PATH=$ROS_ROOT/bin:$PATH

########################################################################################
#Building Rosdep dependencies
FROM base AS rosdep_dependencies

RUN mkdir -p /root/ros2_ws/src
#Copy the adi_3dtof_adtf31xx folder to the workspace src directory
COPY adi_3dtof_adtf31xx /root/ros2_ws/src/adi_3dtof_adtf31xx
COPY libaditof /root/ros2_ws/src/libaditof
COPY libs /root/ros2_ws/src/libs
WORKDIR /root/ros2_ws

# Install rosdep
RUN apt-get update && apt-get install -y python3-rosdep && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install dependencies using rosdep
RUN apt-get update && rosdep install --from-paths src --ignore-src -y

########################################################################################

# SKipping the ToF stage
FROM rosdep_dependencies AS final

# Entry at ros2_ws
WORKDIR /root/ros2_ws
ENV MAKEFLAGS="-j2"
#RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DNXP=1 -DWITH_GLOG_DEPENDENCY=ON -DWITH_NETWORK=ON -DWITH_PROTOBUF_DEPENDENCY=ON --packages-up-to adi_3dtof_adtf31xx"
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc
COPY ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh
RUN apt-get clean && rm -rf /var/lib/apt/lists/*

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
