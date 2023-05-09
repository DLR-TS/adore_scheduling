
SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")

include adore_scheduler.mk

MAKEFLAGS += --no-print-directory


.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

.PHONY: all
all: build

.PHONY: set_env 
set_env: 
	$(eval PROJECT := ${ADORE_SCHEDULER_PROJECT}) 
	$(eval TAG := ${ADORE_SCHEDULER_TAG})

.PHONY: build 
build: root_check docker_group_check set_env build_adore_if_ros_scheduling clean ## Build scheduler 
	docker build --network host \
                 --tag ${PROJECT}:${TAG} \
                 --build-arg PROJECT=${PROJECT} \
				 --build-arg LIB_ADORE_SCHEDULING_TAG=${LIB_ADORE_SCHEDULING_TAG} \
				 --build-arg ADORE_IF_ROS_SCHEDULING_TAG=${ADORE_IF_ROS_SCHEDULING_TAG} \
				 --build-arg ADORE_IF_ROS_SCHEDULING_MSG_TAG=${ADORE_IF_ROS_SCHEDULING_MSG_TAG} .
	docker cp $$(docker create --rm ${PROJECT}:${TAG}):/tmp/${PROJECT}/${PROJECT}/build "${ROOT_DIR}/${PROJECT}"

.PHONY: clean
clean: set_env ## Clean scheduler build artifacts 
	rm -rf "${ROOT_DIR}/${PROJECT}/build"
	docker rm $$(docker ps -a -q --filter "ancestor=${PROJECT}:${TAG}") --force 2> /dev/null || true
	docker rmi $$(docker images -q ${PROJECT}:${TAG}) --force 2> /dev/null || true
