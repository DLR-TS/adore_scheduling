# This Makefile contains useful targets that can be included in downstream projects.

ifeq ($(filter adore_if_ros_scheduling.mk, $(notdir $(MAKEFILE_LIST))), adore_if_ros_scheduling.mk)

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
ADORE_IF_ROS_SCHEDULING_PROJECT=adore_if_ros_scheduling

ADORE_IF_ROS_SCHEDULING_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
ifeq ($(SUBMODULES_PATH),)
    ADORE_IF_ROS_SCHEDULING_SUBMODULES_PATH:=${ADORE_IF_ROS_SCHEDULING_MAKEFILE_PATH}/..
else
    ADORE_IF_ROS_SCHEDULING_SUBMODULES_PATH:=$(shell realpath ${SUBMODULES_PATH})
endif
MAKE_GADGETS_PATH:=${ADORE_IF_ROS_SCHEDULING_SUBMODULES_PATH}/make_gadgets
ifeq ($(wildcard $(MAKE_GADGETS_PATH)/*),)
    $(info INFO: To clone submodules use: 'git submodules update --init --recursive')
    $(info INFO: To specify alternative path for submodules use: SUBMODULES_PATH="<path to submodules>" make build')
    $(info INFO: Default submodule path is: ${ADORE_IF_ROS_SCHEDULING_MAKEFILE_PATH}')
    $(error "ERROR: ${MAKE_GADGETS_PATH} does not exist. Did you clone the submodules?")
endif
REPO_DIRECTORY:=${ADORE_IF_ROS_SCHEDULING_MAKEFILE_PATH}

ADORE_IF_ROS_SCHEDULING_TAG:=$(shell cd ${MAKE_GADGETS_PATH} && make get_sanitized_branch_name REPO_DIRECTORY=${REPO_DIRECTORY})

ADORE_IF_ROS_SCHEDULING_IMAGE=${ADORE_IF_ROS_SCHEDULING_PROJECT}:${ADORE_IF_ROS_SCHEDULING_TAG}

ADORE_IF_ROS_SCHEDULING_CMAKE_BUILD_PATH="${ADORE_IF_ROS_SCHEDULING_PROJECT}/build"
ADORE_IF_ROS_SCHEDULING_CMAKE_INSTALL_PATH="${ADORE_IF_ROS_SCHEDULING_CMAKE_BUILD_PATH}/install"

include ${MAKE_GADGETS_PATH}/make_gadgets.mk
include ${MAKE_GADGETS_PATH}/docker/docker-tools.mk

ADORE_SCHEDULER_PATH:=$(shell realpath "${ADORE_IF_ROS_SCHEDULING_MAKEFILE_PATH}/..")
include ${ADORE_SCHEDULER_PATH}/adore_if_ros_scheduling_msg/adore_if_ros_scheduling_msg.mk
include ${ADORE_SCHEDULER_PATH}/lib_adore_scheduling/lib_adore_scheduling.mk

.PHONY: build_adore_if_ros_scheduling 
build_adore_if_ros_scheduling: ## Build adore_if_ros_scheduling
	cd "${ADORE_IF_ROS_SCHEDULING_MAKEFILE_PATH}" && $(MAKE) build

.PHONY: clean_adore_if_ros_scheduling
clean_adore_if_ros_scheduling: ## Clean adore_if_ros_scheduling build artifacts
	cd "${ADORE_IF_ROS_SCHEDULING_MAKEFILE_PATH}" && $(MAKE) clean

.PHONY: branch_adore_if_ros_scheduling
branch_adore_if_ros_scheduling: ## Returns the current docker safe/sanitized branch for adore_if_ros_scheduling
	@printf "%s\n" ${ADORE_IF_ROS_SCHEDULING_TAG}

.PHONY: image_adore_if_ros_scheduling
image_adore_if_ros_scheduling: ## Returns the current docker image name for adore_if_ros_scheduling
	@printf "%s\n" ${ADORE_IF_ROS_SCHEDULING_IMAGE}

endif
