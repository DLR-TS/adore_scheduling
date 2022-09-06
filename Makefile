SHELL:=/bin/bash

.DEFAULT_GOAL := all


ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=


ADORE_IF_ROS_SCHEDULING_PROJECT="adore_if_ros_scheduling"
ADORE_IF_ROS_SCHEDULING_VERSION="latest"
ADORE_IF_ROS_SCHEDULING_TAG="${ADORE_IF_ROS_SCHEDULING_PROJECT}:${ADORE_IF_ROS_SCHEDULING_VERSION}"

.PHONY: all
all: build

.PHONY: set_env 
set_env: 
	$(eval PROJECT := ${ADORE_IF_ROS_SCHEDULING_PROJECT}) 
	$(eval TAG := ${ADORE_IF_ROS_SCHEDULING_TAG})

.PHONY: build 
build: set_env clean
	docker build --network host \
                 --tag $(shell echo ${TAG} | tr A-Z a-z) \
                 --build-arg PROJECT=${PROJECT} .
	docker cp $$(docker create --rm $(shell echo ${TAG} | tr A-Z a-z)):/tmp/${PROJECT}/${PROJECT}/build ${ROOT_DIR}/${PROJECT}/${PROJECT}
	docker cp $$(docker create --rm $(shell echo ${TAG} | tr A-Z a-z)):/tmp/${PROJECT}/adore_if_ros_scheduling_msg/adore_if_ros_scheduling_msg/build ${ROOT_DIR}/${PROJECT}/adore_if_ros_scheduling_msg/adore_if_ros_scheduling_msg
	docker cp $$(docker create --rm $(shell echo ${TAG} | tr A-Z a-z)):/tmp/${PROJECT}/lib_adore_if_ros_scheduling/lib_adore_if_ros_scheduling/build ${ROOT_DIR}/${PROJECT}/lib_adore_if_ros_scheduling/lib_adore_if_ros_scheduling
	docker cp $$(docker create --rm $(shell echo ${TAG} | tr A-Z a-z)):/tmp/${PROJECT}/lib_adore_scheduling/lib_adore_scheduling/build ${ROOT_DIR}/${PROJECT}/lib_adore_scheduling/lib_adore_scheduling

.PHONY: clean
clean: set_env
	rm -rf "${ROOT_DIR}/${PROJECT}/${PROJECT}/build"
	rm -rf "${ROOT_DIR}/${PROJECT}/adore_if_ros_scheduling_msg/adore_if_ros_scheduling_msg/build"
	rm -rf "${ROOT_DIR}/${PROJECT}/lib_adore_if_ros_scheduling/lib_adore_if_ros_scheduling/build"
	rm -rf "${ROOT_DIR}/${PROJECT}/lib_adore_scheduling/lib_adore_scheduling/build"
	docker rm $$(docker ps -a -q --filter "ancestor=${TAG}") 2> /dev/null || true
	docker rmi $$(docker images -q ${PROJECT}) 2> /dev/null || true

