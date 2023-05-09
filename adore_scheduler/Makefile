
SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")
MAKEFILE_PATH:=$(shell dirname "$(abspath "$(lastword $(MAKEFILE_LIST)"))")

include adore_scheduler.mk

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

.PHONY: all
all: help 

.PHONY: set_env 
set_env: 
	$(eval PROJECT := ${ADORE_SCHEDULER_PROJECT}) 
	$(eval TAG := ${ADORE_SCHEDULER_TAG})

.PHONY: build 
build: root_check docker_group_check set_env clean ## Build adore_scheduler 
	echo "${ADORE_IF_ROS_SCHEDULING_MSG_TAG}"
	exit 1
	docker build --network host \
                 --tag ${PROJECT}:${TAG} \
                 --build-arg PROJECT=${PROJECT} \
				 --build-arg LIBADORE_SCHEDULING_TAG=${LIBADORE_SCHEDULING_TAG} \
				 --build-arg ADORE_IF_ROS_SCHEDULING_TAG=${ADORE_IF_ROS_SCHEDULING_TAG} \
				 --build-arg ADORE_IF_ROS_SCHEDULING_MSG_TAG=${ADORE_IF_ROS_SCHEDULING_MSG_TAG} .
	docker cp $$(docker create --rm ${PROJECT}:${TAG}):/tmp/${PROJECT}/${PROJECT}/build "${ROOT_DIR}/${PROJECT}"

.PHONY: clean_submodules
clean_submodules: clean_adore_if_ros_scheduling clean_libadore_scheduling clean_adore_if_ros_scheduling_msg

.PHONY: clean
clean: set_env clean_submodules ## Clean adore_scheduler build artifacts 
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	docker rm $$(docker ps -a -q --filter "ancestor=${PROJECT}:${TAG}") --force 2> /dev/null || true
	docker rmi $$(docker images -q ${PROJECT}:${TAG}) --force 2> /dev/null || true
