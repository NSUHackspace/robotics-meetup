# Get the base image from Docker Hub
FROM osrf/ros:humble-desktop-full

# Update apps on the base image
RUN apt-get -y update && apt-get install -y

# Install some tools
RUN apt-get -y install tmux vim 

# Install some tools
RUN apt-get -y install ros-humble-gazebo-ros-pkgs

