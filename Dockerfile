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

RUN mkdir -p /root/ros2_ws/src
#Copy the adi_3dtof_adtf31xx folder to the workspace src directory
COPY adi_3dtof_adtf31xx /root/ros2_ws/src/adi_3dtof_adtf31xx
WORKDIR /root/ros2_ws

# Install rosdep
RUN apt-get update && apt-get install -y python3-rosdep && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Install dependencies using rosdep
RUN apt-get update && rosdep install --from-paths src --ignore-src -y

########################################################################################
FROM rosdep_dependencies AS tof_dependencies
#Build ToF Dependencies from source

#Build Glog
WORKDIR /root/
RUN mkdir workspace
WORKDIR /root/workspace
RUN git clone --branch v0.6.0 --depth 1 https://github.com/google/glog \
    && cd glog \
    && mkdir build_0_6_0 && cd build_0_6_0 \
    && cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog .. \
    && cmake --build . --target install

#Build Libwebsockets
WORKDIR /root/workspace
RUN git clone --branch v3.1-stable --depth 1 https://github.com/warmcat/libwebsockets \
    && cd libwebsockets \
    && mkdir build_3_1 && cd build_3_1 \
    && cmake -DLWS_WITH_SSL=OFF -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=/opt/websockets .. \
    && cmake --build . --target install

#Build Protobuf
WORKDIR /root/workspace
RUN git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf \
    && cd protobuf \
    && mkdir build_3_9_0 && cd build_3_9_0 \
    && cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/opt/protobuf ../cmake \
    && cmake --build . --target install

ENV CMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets"

WORKDIR /root/workspace
RUN mkdir libs
COPY libtofi_compute.so /root/workspace/libs/libtofi_compute.so
COPY libtofi_config.so /root/workspace/libs/libtofi_config.so

# Build ToF
RUN git clone --branch v5.0.0 --depth 1 https://github.com/analogdevicesinc/ToF \
    && cd ToF \
    && mkdir build && cd build \
    && cmake -DNXP=1 -DWITH_EXAMPLES=on -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DCMAKE_BUILD_TYPE=Release .. \
    && make -j4
WORKDIR /root/
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/glog/lib:/opt/protobuf/lib:/opt/websockets/lib
RUN unset CMAKE_PREFIX_PATH
########################################################################################

# Set the entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]