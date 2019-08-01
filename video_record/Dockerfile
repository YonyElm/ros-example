FROM ros:kinetic-ros-core

WORKDIR / 

# Prevent 'apt install' from prompting queries 
ARG DEBIAN_FRONTEND=noninteractive
RUN apt update && apt install -qy iputils-ping net-tools apt-utils
RUN apt install -qy ros-kinetic-grid-map ros-kinetic-cv-bridge cmake build-essential libeigen3-dev libboost-all-dev

# Build software
ADD src /src
RUN bash -c 'mkdir /build; \
    cd /build; \
    source /opt/ros/kinetic/setup.bash; \
    cmake ../src/; \
    make'
RUN rm -r /src
ADD entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

ENTRYPOINT []
CMD "/entrypoint.sh"

