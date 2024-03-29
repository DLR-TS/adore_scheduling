
SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")
MAKEFILE_PATH:=$(shell dirname "$(abspath "$(lastword $(MAKEFILE_LIST)"))")

include adore_if_ros_scheduling.mk

MAKEFLAGS += --no-print-directory


.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

.PHONY: all
all: build

.PHONY: set_env 
set_env: 
	$(eval PROJECT := ${ADORE_IF_ROS_SCHEDULING_PROJECT}) 
	$(eval TAG := ${ADORE_IF_ROS_SCHEDULING_TAG})

.PHONY: build 
build: set_env build_adore_if_ros_scheduling_msg build_lib_adore_scheduling ## Build adore_if_ros_scheduling 
	docker build --network host \
                 --tag ${PROJECT}:${TAG} \
                 --build-arg PROJECT=${PROJECT} \
                 --build-arg LIB_ADORE_SCHEDULING_TAG=${LIB_ADORE_SCHEDULING_TAG} \
                 --build-arg ADORE_IF_ROS_SCHEDULING_MSG_TAG=${ADORE_IF_ROS_SCHEDULING_MSG_TAG} .
	docker cp $$(docker create --rm ${PROJECT}:${TAG}):/tmp/${PROJECT}/${PROJECT}/build "${ROOT_DIR}/${PROJECT}"

.PHONY: clean
clean: clean_adore_if_ros_scheduling_msg clean_lib_adore_scheduling set_env ## Clean adore_if_ros_scheduling build artifacts 
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	docker rm $$(docker ps -a -q --filter "ancestor=${PROJECT}:${TAG}") --force 2> /dev/null || true
	docker rmi $$(docker images -q ${PROJECT}:${TAG}) --force 2> /dev/null || true

