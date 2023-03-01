SHELL:=/bin/bash

.DEFAULT_GOAL := all


ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

include ${ROOT_DIR}/adore_scheduling.mk


.PHONY: all
all: build

.PHONY: set_env 
set_env: 
	$(eval PROJECT := ${ADORE_SCHEDULING_PROJECT}) 
	$(eval TAG := ${ADORE_SCHEDULING_TAG})

.PHONY: build 
build: set_env clean build_adore_scheduling
	
.PHONY: clean
clean: set_env clean_adore_scheduling
	

