
SHELL:=/bin/bash

.DEFAULT_GOAL := all

ROOT_DIR:=$(shell dirname "$(realpath $(firstword $(MAKEFILE_LIST)))")

include libadore_scheduling.mk

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
DOCKER_BUILDKIT?=1
DOCKER_CONFIG?=

.PHONY: all
all: help 

.PHONY: set_env 
set_env: 
	$(eval project := ${libadore_scheduling_PROJECT}) 
	$(eval tag := ${libadore_scheduling_TAG})

.PHONY: build 
build: root_check docker_group_check set_env clean ## Build libadore_scheduling 
	docker build --network host \
                 --tag ${project}:${tag} \
                 --build-arg project=${project} .
	docker cp $$(docker create --rm ${project}:${tag}):/tmp/${project}/${project}/build "${ROOT_DIR}/${project}"

.PHONY: clean
clean: set_env ## Clean libadore_scheduling build artifacts 
	rm -rf "${ROOT_DIR}/${project}/build"
	docker rm $$(docker ps -a -q --filter "ancestor=${project}:${tag}") --force 2> /dev/null || true
	docker rmi $$(docker images -q ${project}:${tag}) --force 2> /dev/null || true
