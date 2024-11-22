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

# Source ROS setup script
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
########################################################################################
#Building Rosdep dependencies
FROM base AS rosdep_dependencies

# Create a workspace directory
RUN mkdir -p /home/analog/ros2_ws/src

# Copy the adi_3dtof_adtf31xx folder to the workspace src directory
# COPY adi_3dtof_adtf31xx /home/analog/ros2_ws/src/adi_3dtof_adtf31xx
# WORKDIR /home/analog/ros2_ws
# Mount ros2_ws from /home/analog/ros2_ws to /root
VOLUME ["/home/analog/ros2_ws:/home/analog/ros2_ws"]

# Install rosdep
RUN apt-get update && apt-get install -y python3-rosdep && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install dependencies using rosdep
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y

########################################################################################
FROM rosdep_dependencies AS tof_dependencies
#Build ToF Dependencies from source

#Build Glog
WORKDIR /home/analog/
RUN git clone --branch v0.6.0 --depth 1 https://github.com/google/glog \
    && cd glog \
    && mkdir build_0_6_0 && cd build_0_6_0 \
    && cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog .. \
    && cmake --build . --target install

#Build Libwebsockets
WORKDIR /home/analog/
RUN git clone --branch v3.1-stable --depth 1 https://github.com/warmcat/libwebsockets \
    && cd libwebsockets \
    && mkdir build_3_1 && cd build_3_1 \
    && cmake -DLWS_WITH_SSL=OFF -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=/opt/websockets .. \
    && cmake --build . --target install

#Build Protobuf
WORKDIR /home/analog/
RUN git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf \
    && cd protobuf \
    && mkdir build_3_9_0 && cd build_3_9_0 \
    && cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/opt/protobuf ../cmake \
    && cmake --build . --target install

ENV CMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets"

# Build ToF
WORKDIR /home/analog/
RUN git clone --branch v5.0.0 --depth 1 https://github.com/analogdevicesinc/ToF \
    && cd ToF \
    && git submodule update --init \
    && mkdir build && cd build \
    && cmake -DWITH_EXAMPLES=on -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DCMAKE_BUILD_TYPE=Release .. \
    && make -j4
WORKDIR /home/analog/
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/glog/lib:/opt/protobuf/lib:/opt/websockets/lib

# Clean up the source directories
RUN rm -rf /home/analog/glog /home/analog/libwebsockets /home/analog/protobuf
########################################################################################
FROM tof_dependencies AS ros_build
#build the workspace
ENV MAKEFLAGS='-j 2'
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers console_direct+" \
    && { \
        echo "Build successful, running additional commands."; \
        export FASTRTPS_DEFAULT_PROFILES_FILE="/home/analog/ros2_ws/src/adi_3dtof_adtf31xx/rmw_config/rmw_settings.xml"; \
        source /opt/ros/humble/setup.bash && ros2 daemon stop; \
        chmod +x /home/analog/ros2_ws/src/adi_3dtof_adtf31xx/rmw_config/setup_rmw_settings.sh; \
        source /home/analog/ros2_ws/src/adi_3dtof_adtf31xx/rmw_config/setup_rmw_settings.sh; \
    } || { \
        echo "Build failed, skipping additional commands."; \
    }

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]