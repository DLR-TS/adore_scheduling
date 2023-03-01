ARG PROJECT

FROM ros:noetic-ros-core-focal AS lib_adore_scheduling_requirements_base

ARG PROJECT
ARG REQUIREMENTS_FILE="requirements.${PROJECT}.ubuntu20.04.system"


RUN mkdir -p /tmp/${PROJECT}
WORKDIR /tmp/${PROJECT}
copy files/${REQUIREMENTS_FILE} /tmp/${PROJECT}

RUN apt-get update && \
    apt-get install --no-install-recommends -y checkinstall && \
    xargs apt-get install --no-install-recommends -y < ${REQUIREMENTS_FILE} && \
    rm -rf /var/lib/apt/lists/*

COPY ${PROJECT} /tmp/${PROJECT}/${PROJECT}


FROM lib_adore_scheduling_requirements_base AS lib_adore_scheduling_builder

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


FROM alpine:3.14

ARG PROJECT
#COPY --from=lib_adore_scheduling_builder /tmp/${PROJECT} /tmp/${PROJECT}
COPY --from=lib_adore_scheduling_builder /tmp/${PROJECT}/${PROJECT} /tmp/${PROJECT}/${PROJECT}






#RUN source /opt/ros/noetic/setup.bash && \
#    cmake .. && \
#    cmake --build .  --config Release --target install -- -j $(nproc) && \
#    cpack -G DEB && find . -type f -name "*.deb" | xargs mv -t . && \
#    cd /tmp/${PROJECT}/${PROJECT}/build && ln -s devel install && \
#    mv CMakeCache.txt CMakeCache.txt.build 
#
#
#FROM alpine:3.14
#
#ARG PROJECT
##COPY --from=adore_if_ros_scheduling_msg_builder /tmp/${PROJECT} /tmp/${PROJECT}
#COPY --from=adore_if_ros_scheduling_msg_builder /tmp/${PROJECT}/${PROJECT} /tmp/${PROJECT}/${PROJECT}
