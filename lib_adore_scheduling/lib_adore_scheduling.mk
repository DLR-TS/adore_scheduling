# This Makefile contains useful targets that can be included in downstream projects.

ifeq ($(filter lib_adore_scheduling.mk, $(notdir $(MAKEFILE_LIST))), lib_adore_scheduling.mk)

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
LIB_ADORE_SCHEDULING_PROJECT=lib_adore_scheduling

LIB_ADORE_SCHEDULING_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
ifeq ($(SUBMODULES_PATH),)
    LIB_ADORE_SCHEDULING_SUBMODULES_PATH:=${LIB_ADORE_SCHEDULING_MAKEFILE_PATH}/..
else
    LIB_ADORE_SCHEDULING_SUBMODULES_PATH:=$(shell realpath ${SUBMODULES_PATH})
endif
MAKE_GADGETS_PATH:=${LIB_ADORE_SCHEDULING_SUBMODULES_PATH}/make_gadgets
ifeq ($(wildcard $(MAKE_GADGETS_PATH)/*),)
    $(info INFO: To clone submodules use: 'git submodules update --init --recursive')
    $(info INFO: To specify alternative path for submodules use: SUBMODULES_PATH="<path to submodules>" make build')
    $(info INFO: Default submodule path is: ${LIB_ADORE_SCHEDULING_MAKEFILE_PATH}')
    $(error "ERROR: ${MAKE_GADGETS_PATH} does not exist. Did you clone the submodules?")
endif
REPO_DIRECTORY:=${LIB_ADORE_SCHEDULING_MAKEFILE_PATH}

LIB_ADORE_SCHEDULING_TAG:=$(shell cd ${MAKE_GADGETS_PATH} && make get_sanitized_branch_name REPO_DIRECTORY=${REPO_DIRECTORY})

LIB_ADORE_SCHEDULING_IMAGE=${LIB_ADORE_SCHEDULING_PROJECT}:${LIB_ADORE_SCHEDULING_TAG}

LIB_ADORE_SCHEDULING_CMAKE_BUILD_PATH="${LIB_ADORE_SCHEDULING_PROJECT}/build"
LIB_ADORE_SCHEDULING_CMAKE_INSTALL_PATH="${LIB_ADORE_SCHEDULING_CMAKE_BUILD_PATH}/install"

include ${MAKE_GADGETS_PATH}/make_gadgets.mk
include ${MAKE_GADGETS_PATH}/docker/docker-tools.mk

.PHONY: build_lib_adore_scheduling 
build_lib_adore_scheduling: ## Build lib_adore_scheduling
	cd "${LIB_ADORE_SCHEDULING_MAKEFILE_PATH}" && make

.PHONY: clean_lib_adore_scheduling
clean_lib_adore_scheduling: ## Clean lib_adore_scheduling build artifacts
	cd "${LIB_ADORE_SCHEDULING_MAKEFILE_PATH}" && make clean

.PHONY: branch_lib_adore_scheduling
branch_lib_adore_scheduling: ## Returns the current docker safe/sanitized branch for lib_adore_scheduling
	@printf "%s\n" ${LIB_ADORE_SCHEDULING_TAG}

.PHONY: image_lib_adore_scheduling
image_lib_adore_scheduling: ## Returns the current docker image name for lib_adore_scheduling
	@printf "%s\n" ${LIB_ADORE_SCHEDULING_IMAGE}

endif
