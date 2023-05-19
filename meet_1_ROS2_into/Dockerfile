# Get the base image from Docker Hub
FROM osrf/ros:humble-desktop-full

# Update apps on the base image
RUN apt-get update && apt-get install -y \
    tmux \
    vim \
    ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

