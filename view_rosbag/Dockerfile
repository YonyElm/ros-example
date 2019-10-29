FROM ros:kinetic-ros-core

WORKDIR / 

# Prevent 'apt-get install' from prompting queries 
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -qq iputils-ping net-tools apt-utils
RUN apt-get install -qq ros-kinetic-grid-map ros-kinetic-cv-bridge cmake build-essential libeigen3-dev libboost-all-dev

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

## Keep version
ARG GIT_COMMIT = ${GIT_COMMIT}
ARG GIT_BRANCH = ${GIT_BRANCH}
ARG GIT_REPO_URL = ${GIT_REPO_URL}
RUN echo [[$(date)]] repository URL: ${GIT_REPO_URL} > /version_log.txt
RUN echo [[$(date)]] video_record branch: ${GIT_BRANCH} > /version_log.txt
RUN echo [[$(date)]] video_record commit: ${GIT_COMMIT} > /version_log.txt

ENTRYPOINT []
CMD "/entrypoint.sh"

