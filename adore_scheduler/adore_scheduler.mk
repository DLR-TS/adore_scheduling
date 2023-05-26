# This Makefile contains useful targets that can be included in downstream projects.

ifeq ($(filter adore_scheduler.mk, $(notdir $(MAKEFILE_LIST))), adore_scheduler.mk)

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
ADORE_SCHEDULER_PROJECT=adore_scheduler

ADORE_SCHEDULER_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
ifeq ($(SUBMODULES_PATH),)
    ADORE_SCHEDULER_SUBMODULES_PATH:=${ADORE_SCHEDULER_MAKEFILE_PATH}/..
else
    ADORE_SCHEDULER_SUBMODULES_PATH:=$(shell realpath ${SUBMODULES_PATH})
endif
MAKE_GADGETS_PATH:=${ADORE_SCHEDULER_SUBMODULES_PATH}/make_gadgets
ifeq ($(wildcard $(MAKE_GADGETS_PATH)/*),)
    $(info INFO: To clone submodules use: 'git submodules update --init --recursive')
    $(info INFO: To specify alternative path for submodules use: SUBMODULES_PATH="<path to submodules>" make build')
    $(info INFO: Default submodule path is: ${ADORE_SCHEDULER_MAKEFILE_PATH}')
    $(error "ERROR: ${MAKE_GADGETS_PATH} does not exist. Did you clone the submodules?")
endif
REPO_DIRECTORY:=${ADORE_SCHEDULER_MAKEFILE_PATH}

ADORE_SCHEDULER_TAG:=$(shell cd ${MAKE_GADGETS_PATH} && make get_sanitized_branch_name REPO_DIRECTORY=${REPO_DIRECTORY})

ADORE_SCHEDULER_IMAGE=${ADORE_SCHEDULER_PROJECT}:${ADORE_SCHEDULER_TAG}

ADORE_SCHEDULER_CMAKE_BUILD_PATH="${ADORE_SCHEDULER_PROJECT}/build"
ADORE_SCHEDULER_CMAKE_INSTALL_PATH="${ADORE_SCHEDULER_CMAKE_BUILD_PATH}/install"

include ${MAKE_GADGETS_PATH}/make_gadgets.mk
include ${MAKE_GADGETS_PATH}/docker/docker-tools.mk

ADORE_SCHEDULER_PATH:=$(shell realpath "${ADORE_SCHEDULER_MAKEFILE_PATH}/..")
include ${ADORE_SCHEDULER_PATH}/adore_if_ros_scheduling/adore_if_ros_scheduling.mk

.PHONY: build_adore_scheduler 
build_adore_scheduler: ## Build scheduler
	cd "${ADORE_SCHEDULER_MAKEFILE_PATH}" && make build

.PHONY: clean_adore_scheduler
clean_adore_scheduler: ## Clean scheduler build artifacts
	cd "${ADORE_SCHEDULER_MAKEFILE_PATH}" && make clean

.PHONY: branch_adore_scheduler
branch_adore_scheduler: ## Returns the current docker safe/sanitized branch for scheduler
	@printf "%s\n" ${ADORE_SCHEDULER_TAG}

.PHONY: image_adore_scheduler
image_adore_scheduler: ## Returns the current docker image name for scheduler
	@printf "%s\n" ${ADORE_SCHEDULER_IMAGE}

endif
