# This Makefile contains useful targets that can be included in downstream projects.

ifeq ($(filter adore_scheduling.mk, $(notdir $(MAKEFILE_LIST))), adore_scheduling.mk)

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
ADORE_SCHEDULING_PROJECT=adore_scheduling

ADORE_SCHEDULING_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
ifeq ($(SUBMODULES_PATH),)
    ADORE_SCHEDULING_SUBMODULES_PATH:=${ADORE_SCHEDULING_MAKEFILE_PATH}
else
    ADORE_SCHEDULING_SUBMODULES_PATH:=$(shell realpath ${SUBMODULES_PATH})
endif
MAKE_GADGETS_PATH:=${ADORE_SCHEDULING_SUBMODULES_PATH}/make_gadgets
ifeq ($(wildcard $(MAKE_GADGETS_PATH)/*),)
    $(info INFO: To clone submodules use: 'git submodule update --init --recursive')
    $(info INFO: To specify alternative path for submodules use: SUBMODULES_PATH="<path to submodules>" make build')
    $(info INFO: Default submodule path is: ${ADORE_SCHEDULING_MAKEFILE_PATH}')
    $(error "ERROR: ${MAKE_GADGETS_PATH} does not exist. Did you clone the submodules?")
endif
REPO_DIRECTORY:=${ADORE_SCHEDULING_MAKEFILE_PATH}

ADORE_SCHEDULING_TAG:=$(shell cd ${MAKE_GADGETS_PATH} && make get_sanitized_branch_name REPO_DIRECTORY=${REPO_DIRECTORY})

ADORE_SCHEDULING_IMAGE=${ADORE_SCHEDULING_PROJECT}:${ADORE_SCHEDULING_TAG}

ADORE_SCHEDULING_CMAKE_BUILD_PATH="${ADORE_SCHEDULING_PROJECT}/build"
ADORE_SCHEDULING_CMAKE_INSTALL_PATH="${ADORE_SCHEDULING_CMAKE_BUILD_PATH}/install"


include ${ADORE_SCHEDULING_MAKEFILE_PATH}/adore_scheduler/adore_scheduler.mk
include ${MAKE_GADGETS_PATH}/make_gadgets.mk

include ${ADORE_SCHEDULING_SUBMODULES_PATH}/cpplint_docker/cpplint_docker.mk
include ${ADORE_SCHEDULING_SUBMODULES_PATH}/cppcheck_docker/cppcheck_docker.mk
include ${ADORE_SCHEDULING_SUBMODULES_PATH}/lizard_docker/lizard_docker.mk

.PHONY: build_adore_scheduling 
build_adore_scheduling: ## Build adore_scheduling
	cd "${ADORE_SCHEDULING_MAKEFILE_PATH}" && make build

.PHONY: clean_adore_scheduling
clean_adore_scheduling: ## Clean adore_scheduling build artifacts
	cd "${ADORE_SCHEDULING_MAKEFILE_PATH}" && make clean

.PHONY: branch_adore_scheduling
branch_adore_scheduling: ## Returns the current docker safe/sanitized branch for adore_scheduling 
	@printf "%s\n" ${ADORE_SCHEDULING_TAG}

.PHONY: image_adore_scheduling
image_adore_scheduling: ## Returns the current docker image name for adore_scheduling
	@printf "%s\n" ${ADORE_SCHEDULING_IMAGE}

.PHONY: images_adore_scheduling
images_adore_scheduling: ## Returns the current docker image name for adore_scheduling
	@printf "%s\n" "${ADORE_SCHEDULING_IMAGE}"
	@printf "%s\n" "adore_if_ros_scheduling:${ADORE_SCHEDULING_TAG}"
	@printf "%s\n" "lib_adore_scheduling:${ADORE_SCHEDULING_TAG}"
	@printf "%s\n" "adore_if_ros_scheduling_msg:${ADORE_SCHEDULING_TAG}"

endif
