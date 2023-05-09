ARG PROJECT
ARG ADORE_IF_ROS_SCHEDULING_MSG_TAG
ARG ADORE_IF_ROS_SCHEDULING_TAG
ARG LIBADORE_SCHEDULING_TAG

FROM adore_if_ros_scheduling_msg:${ADORE_IF_ROS_SCHEDULING_MSG_TAG} AS adore_if_ros_scheduling_msg
FROM libadore_scheduling:${LIBADORE_SCHEDULING_TAG} AS libadore_scheduling
FROM adore_if_ros_scheduling:${ADORE_IF_ROS_SCHEDULING_TAG} AS adore_if_ros_scheduling

FROM ros:noetic-ros-core-focal AS adore_scheduler_requirements_base

ARG PROJECT
ARG REQUIREMENTS_FILE="requirements.${PROJECT}.ubuntu20.04.system"


RUN mkdir -p /tmp/${PROJECT}
WORKDIR /tmp/${PROJECT}

ARG INSTALL_PREFIX=/tmp/${PROJECT}/${PROJECT}/build/install
RUN mkdir -p "${INSTALL_PREFIX}"

ARG LIB=libadore_scheduling
COPY --from=libadore_scheduling /tmp/${LIB} /tmp/${LIB}
SHELL ["/bin/bash", "-c"]
WORKDIR /tmp/${LIB}/${LIB}/build
RUN cmake --install . --prefix ${INSTALL_PREFIX}

ARG LIB=adore_if_ros_scheduling_msg
COPY --from=adore_if_ros_scheduling_msg /tmp/${LIB} /tmp/${LIB}
SHELL ["/bin/bash", "-c"]
WORKDIR /tmp/${LIB}/${LIB}/build
RUN cmake --install . --prefix ${INSTALL_PREFIX}

ARG LIB=adore_if_ros_scheduling
COPY --from=adore_if_ros_scheduling /tmp/${LIB} /tmp/${LIB}
SHELL ["/bin/bash", "-c"]
WORKDIR /tmp/${LIB}/${LIB}/build
RUN cmake --install . --prefix ${INSTALL_PREFIX}

WORKDIR /tmp/${PROJECT}
copy files/${REQUIREMENTS_FILE} /tmp/${PROJECT}

RUN apt-get update && \
    apt-get install --no-install-recommends -y checkinstall && \
    xargs apt-get install --no-install-recommends -y < ${REQUIREMENTS_FILE} && \
    rm -rf /var/lib/apt/lists/*

COPY ${PROJECT} /tmp/${PROJECT}/${PROJECT}


FROM adore_scheduler_requirements_base AS adore_scheduler_builder

ARG PROJECT


WORKDIR /tmp/${PROJECT}/${PROJECT}
RUN mkdir -p build 
SHELL ["/bin/bash", "-c"]
WORKDIR /tmp/${PROJECT}/${PROJECT}/build

RUN source /opt/ros/noetic/setup.bash && \
    cmake .. -DCMAKE_PREFIX_PATH=install -DCMAKE_INSTALL_PREFIX:PATH=install && \
    cmake --build . --config Release --target install -- -j $(nproc) && \
    cpack -G DEB && find . -type f -name "*.deb" | xargs mv -t .  && \
    mv CMakeCache.txt CMakeCache.txt.build 


#FROM alpine:3.14

#ARG PROJECT
#COPY --from=scheduler_builder /tmp/${PROJECT}/${PROJECT} /tmp/${PROJECT}/${PROJECT}

