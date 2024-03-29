FROM ros:kinetic-ros-core

WORKDIR / 

# Prevent 'apt-get install' from prompting queries 
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -qq --no-install-recommends iputils-ping net-tools apt-utils
RUN apt-get install -qq --no-install-recommends ros-kinetic-grid-map ros-kinetic-cv-bridge cmake build-essential libeigen3-dev libboost-all-dev

# Build software
ARG CPP_MAKE_FLAGS=${CPP_MAKE_FLAGS}
ADD src /src
RUN bash -c 'mkdir /build; \
    cd /build; \
    source /opt/ros/kinetic/setup.bash; \
    cmake ../src/ -DCMAKE_BUILD_TYPE=Debug; \
    export CPP_MAKE_FLAGS=${CPP_MAKE_FLAGS}; \
    make $CPP_MAKE_FLAGS'
RUN rm -r /src
ADD entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

## Keep version
ARG GIT_COMMIT=${GIT_COMMIT}
ARG GIT_BRANCH=${GIT_BRANCH}
ARG GIT_REPO_URL=${GIT_REPO_URL}
ARG GIT_TAG=${GIT_TAG}
RUN echo [[$(date)]] repository URL: ${GIT_REPO_URL} >> /version_log.txt
RUN echo [[$(date)]] view_rosbag branch: ${GIT_BRANCH} >> /version_log.txt
RUN echo [[$(date)]] view_rosbag commit: ${GIT_COMMIT} >> /version_log.txt
RUN echo [[$(date)]] view_rosbag tag: ${GIT_TAG} >> /version_log.txt

ENTRYPOINT []
CMD "/entrypoint.sh"

