FROM ros:kinetic-ros-core

WORKDIR / 

# Prevent 'apt install' from prompting queries 
ARG DEBIAN_FRONTEND=noninteractive
RUN apt update 
RUN apt install -qq cmake build-essential libeigen3-dev

RUN echo "source /opt/ros/kinetic/setup.bash >> ~/.bashrc"


CMD "sleep 99999999999"

