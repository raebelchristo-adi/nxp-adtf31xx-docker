#!/bin/bash
set -e

#setup environment
source /root/.bashrc

#start in workspace directory
cd /root/ros2_ws
exec bash -i -c $@