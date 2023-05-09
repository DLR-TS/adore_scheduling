ARG project

FROM ros:noetic-ros-core-focal AS adore_if_ros_scheduling_msg_requirements_base

ARG project
ARG REQUIREMENTS_FILE="requirements.${project}.ubuntu20.04.system"


RUN mkdir -p /tmp/${project}
WORKDIR /tmp/${project}
copy files/${REQUIREMENTS_FILE} /tmp/${project}

RUN apt-get update && \
    apt-get install --no-install-recommends -y checkinstall && \
    xargs apt-get install --no-install-recommends -y < ${REQUIREMENTS_FILE} && \
    rm -rf /var/lib/apt/lists/*

COPY ${project} /tmp/${project}/${project}


FROM adore_if_ros_scheduling_msg_requirements_base AS adore_if_ros_scheduling_msg_builder

ARG project
WORKDIR /tmp/${project}/${project}
RUN mkdir -p build 
SHELL ["/bin/bash", "-c"]
WORKDIR /tmp/${project}/${project}/build

#cmake .. -DCATKIN_DEVEL_PREFIX="/tmp/${project}/build/install" && \

RUN source /opt/ros/noetic/setup.bash && \
    cmake .. && \
    cmake --build .  --config Release --target install -- -j $(nproc) && \
    cpack -G DEB && find . -type f -name "*.deb" | xargs mv -t . && \
    cd /tmp/${project}/${project}/build && ln -s devel install && \
    mv CMakeCache.txt CMakeCache.txt.build 


FROM alpine:3.14

ARG project
#COPY --from=adore_if_ros_scheduling_msg_builder /tmp/${project} /tmp/${project}
COPY --from=adore_if_ros_scheduling_msg_builder /tmp/${project}/${project} /tmp/${project}/${project}
