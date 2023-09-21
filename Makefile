SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")

include ${ROOT_DIR}/adore_scheduling.mk

MAKEFLAGS += --no-print-directory


.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

SUBMODULES_PATH?=${ROOT_DIR}

include ${ADORE_SCHEDULING_SUBMODULES_PATH}/ci_teststand/ci_teststand.mk

CPP_PROJECT_DIRECTORY=${ROOT_DIR}


set_env: 
	$(eval PROJECT := ${ADORE_SCHEDULING_PROJECT}) 
	$(eval TAG := ${ADORE_SCHEDULING_TAG})

.PHONY: build 
build: set_env clean build_adore_scheduler
	mkdir -p ${ROOT_DIR}/${PROJECT}/build
	docker tag adore_scheduler:${TAG} ${PROJECT}:${TAG}

.PHONY: clean
clean: set_env clean_adore_scheduler
	docker rm $$(docker ps -a -q --filter "ancestor=${PROJECT}:${TAG}") --force 2> /dev/null || true
	docker rmi $$(docker images -q ${PROJECT}:${TAG}) --force 2> /dev/null || true
	rm -rf ${ROOT_DIR}/${PROJECT}

.PHONY: ci_build
ci_build: build

.PHONY: test
test: ci_test

