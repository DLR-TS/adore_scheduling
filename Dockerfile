ARG PROJECT

FROM ros:noetic-ros-core-focal AS adore_if_ros_scheduling_builder

ARG PROJECT
ARG REQUIREMENTS_FILE="requirements.${PROJECT}.ubuntu20.04.system"

RUN mkdir -p /tmp/${PROJECT}
WORKDIR /tmp/${PROJECT}
copy files/${REQUIREMENTS_FILE} /tmp/${PROJECT}

RUN apt-get update && \
    apt-get install --no-install-recommends -y checkinstall && \
    xargs apt-get install --no-install-recommends -y < ${REQUIREMENTS_FILE} && \
    rm -rf /var/lib/apt/lists/*

COPY ${PROJECT} /tmp/${PROJECT}


#RUN cd adore_if_ros_scheduling_msg && mkdir -p build 
#SHELL ["/bin/bash", "-c"]
#WORKDIR /tmp/${PROJECT}/adore_if_ros_scheduling_msg/build

#RUN source /opt/ros/noetic/setup.bash && \
#    cmake .. && \
#    cmake --build .  --config Release --target install -- -j $(nproc) && \
#    #cpack -G DEB && find . -type f -name "*.deb" | xargs mv -t . && \
#    cd /tmp/${PROJECT}/adore_if_ros_scheduling_msg/build && ln -s devel install 

#WORKDIR /tmp/${PROJECT}

#RUN cd lib_adore_scheduling && mkdir -p build 
#SHELL ["/bin/bash", "-c"]
#WORKDIR /tmp/${PROJECT}/lib_adore_scheduling/build

#RUN source /opt/ros/noetic/setup.bash && \
#    cmake .. && \
#    cmake --build .  --config Release --target install -- -j $(nproc) && \
#    #cpack -G DEB && find . -type f -name "*.deb" | xargs mv -t . && \
#    cd /tmp/${PROJECT}/lib_adore_scheduling/build && ln -s devel install 


#WORKDIR /tmp/${PROJECT}

#RUN cd lib_adore_if_ros_scheduling && mkdir -p build 
#SHELL ["/bin/bash", "-c"]
#WORKDIR /tmp/${PROJECT}/lib_adore_if_ros_scheduling/build

#RUN source /opt/ros/noetic/setup.bash && \
#    cmake .. && \
#    cmake --build .  --config Release --target install -- -j $(nproc) && \
#    #cpack -G DEB && find . -type f -name "*.deb" | xargs mv -t . && \
#    cd /tmp/${PROJECT}/lib_adore_if_ros_scheduling/build && ln -s devel install 


WORKDIR /tmp/${PROJECT}

ARG CMAKE_PREFIX_PATH="/tmp/${PROJECT}/build/install"

RUN mkdir -p build 
SHELL ["/bin/bash", "-c"]
WORKDIR /tmp/${PROJECT}/build

RUN source /opt/ros/noetic/setup.bash && \
    cmake .. \ 
             -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
             -DCMAKE_BUILD_TYPE=Release \
             -DCMAKE_INSTALL_PREFIX="install" \
             -DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH} && \
    cmake --build . -v --config Release --target install -- -j $(nproc) && \
    #cpack -G DEB && find . -type f -name "*.deb" | xargs mv -t . && \
    cd /tmp/${PROJECT}/build && ln -s devel install | true

