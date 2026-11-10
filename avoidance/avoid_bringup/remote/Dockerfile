# kymavoid on the build machine: ROS Noetic + Gazebo 11 (the host is Ubuntu 22.04 with ROS 2 only).
# Built there by `remote.sh setup`; see remote.sh.
FROM osrf/ros:noetic-desktop-full

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-noetic-rosbridge-server \
      python3-numpy python3-opencv python3-yaml \
      psmisc procps \
    && rm -rf /var/lib/apt/lists/*
