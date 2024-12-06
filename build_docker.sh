#!/bin/bash
# Check if the required folders and file exist
if [ -d "adi_3dtof_adtf31xx" ] && [ -f "libs/libtofi_compute.so" ] && [ -f "libs/libtofi_config.so" ] && [ -f "ros_entrypoint.sh" ]; then
    echo "Required folders and file are present. Building Docker image..."
    docker build --target final -t adtf31xx .
else
    echo "One or more required folders or file are missing. Aborting Docker build."
    exit 1
fi

mkdir -p $HOME/ros2_ws/src