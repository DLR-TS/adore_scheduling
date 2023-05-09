SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

#include ${ROOT_DIR}/adore_if_ros_scheduling_msg/adore_if_ros_scheduling_msg.mk
#include ${ROOT_DIR}/libadore_scheduling/libadore_scheduling.mk
include ${ROOT_DIR}/adore_scheduling.mk
CPP_PROJECT_DIRECTORY=${ROOT_DIR}

.PHONY: all
all: help

.PHONY: set_env 
set_env: 
	$(eval PROJECT := ${ADORE_SCHEDULING_PROJECT}) 
	$(eval TAG := ${ADORE_SCHEDULING_TAG})

.PHONY: build 
build: set_env clean build_adore_scheduler ## Build adore_scheduling

.PHONY:

.PHONY: clean
clean: set_env clean_adore_scheduler ## Clean adore_scheduling

