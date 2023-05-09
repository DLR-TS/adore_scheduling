# This Makefile contains useful targets that can be included in downstream projects.

project=adore_scheduler
PROJECT=$(shell echo ${project}| tr '[:lower:]' '[:upper:]')

ifeq ($(call if,$(wildcard ${PROJECT}),),)
${PROJECT}:

.EXPORT_ALL_VARIABLES:
${PROJECT}_PROJECT=${project}

${PROJECT}_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
ifeq ($(SUBMODULES_PATH),)
    ${PROJECT}_SUBMODULES_PATH:=$(shell realpath "${${PROJECT}_MAKEFILE_PATH}/..")
else
    ${PROJECT}_SUBMODULES_PATH:=$(shell realpath ${SUBMODULES_PATH})
endif

_SUBMODULES_PATH:=${${PROJECT}_SUBMODULES_PATH}

MAKE_GADGETS_PATH:=${${PROJECT}_SUBMODULES_PATH}/make_gadgets
ifeq ($(wildcard $(MAKE_GADGETS_PATH)/*),)
    $(info INFO: To clone submodules use: 'git submodules update --init --recursive')
    $(info INFO: To specify alternative path for submodules use: SUBMODULES_PATH="<path to submodules>" make build')
    $(info INFO: Default submodule path is: ${${PROJECT}_MAKEFILE_PATH}')
    $(error "ERROR: ${MAKE_GADGETS_PATH} does not exist. Did you clone the submodules?")
endif

REPO_DIRECTORY:=${${PROJECT}_MAKEFILE_PATH}
${PROJECT}_TAG:=$(shell cd ${MAKE_GADGETS_PATH} && make get_sanitized_branch_name REPO_DIRECTORY=${REPO_DIRECTORY})
${PROJECT}_IMAGE:=${${PROJECT}_PROJECT}:${${PROJECT}_TAG}

${PROJECT}_CMAKE_BUILD_PATH:="${${PROJECT}_PROJECT}/build"
${PROJECT}_CMAKE_INSTALL_PATH:="${${PROJECT}_CMAKE_BUILD_PATH}/install"

define PROJECT_RULE
	$(eval TARGET_PREFIX := clean) 
	$(eval TARGET := $@)
    PROJECT := $(shell echo ${TARGET} | sed 's|${TARGET_PREFIX}_||g' | tr '[:lower:]' '[:upper:]')
endef

.PHONY: build_${project} 
build_${project}: ## Build ${project} 
	@$(eval $(PROJECT_RULE))
	cd "${${PROJECT}_MAKEFILE_PATH}" && make build

.PHONY: clean_${project}
clean_${project}: ## Clean ${project} build artifacts
	@$(eval $(PROJECT_RULE))
	cd "${${PROJECT}_MAKEFILE_PATH}" && make clean

.PHONY: branch_${project}
branch_${project}: ## Returns the current docker safe/sanitized branch for ${project} 
	@$(eval $(PROJECT_RULE))
	@printf "%s\n" ${${PROJECT}_TAG}

.PHONY: image_${project}
image_${project}: ## Returns the current docker image name for ${project}
	@$(eval $(PROJECT_RULE))
	@printf "%s\n" ${${PROJECT}_IMAGE}

include ${MAKE_GADGETS_PATH}/make_gadgets.mk
include ${MAKE_GADGETS_PATH}/docker/docker-tools.mk

#include ${ADORE_SCHEDULER_SUBMODULES_PATH}/adore_if_ros_scheduling_msg/adore_if_ros_scheduling_msg.mk
#include ${ADORE_SCHEDULER_SUBMODULES_PATH}/libadore_scheduling/libadore_scheduling.mk
include ${ADORE_SCHEDULER_SUBMODULES_PATH}/adore_if_ros_scheduling/adore_if_ros_scheduling.mk


endif
