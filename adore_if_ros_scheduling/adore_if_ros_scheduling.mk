# This Makefile contains useful targets that can be included in downstream projects.

ifndef adore_if_ros_scheduling_MAKEFILE_PATH

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
adore_if_ros_scheduling_project=adore_if_ros_scheduling
ADORE_IF_ROS_SCHEDULING_PROJECT=${adore_if_ros_scheduling_project}

adore_if_ros_scheduling_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
make_gadgets_PATH:=${adore_if_ros_scheduling_MAKEFILE_PATH}/../make_gadgets
REPO_DIRECTORY:=${adore_if_ros_scheduling_MAKEFILE_PATH}

adore_if_ros_scheduling_tag:=$(shell cd ${make_gadgets_PATH} && make get_sanitized_branch_name REPO_DIRECTORY=${REPO_DIRECTORY})
ADORE_IF_ROS_SCHEDULING_TAG=${adore_if_ros_scheduling_tag}

adore_if_ros_scheduling_image=${adore_if_ros_scheduling_project}:${adore_if_ros_scheduling_tag}
ADORE_IF_ROS_SCHEDULING_IMAGE=${adore_if_ros_scheduling_image}


include ${adore_if_ros_scheduling_MAKEFILE_PATH}/../adore_if_ros_scheduling_msg/adore_if_ros_scheduling_msg.mk
include ${adore_if_ros_scheduling_MAKEFILE_PATH}/../lib_adore_scheduling/lib_adore_scheduling.mk

.PHONY: build_adore_if_ros_scheduling 
build_adore_if_ros_scheduling: ## Build adore_if_ros_scheduling
	cd "${adore_if_ros_scheduling_MAKEFILE_PATH}" && $(MAKE)

.PHONY: clean_adore_if_ros_scheduling
clean_adore_if_ros_scheduling: ## Clean adore_if_ros_scheduling build artifacts
	cd "${adore_if_ros_scheduling_MAKEFILE_PATH}" && $(MAKE) clean

.PHONY: branch_adore_if_ros_scheduling
branch_adore_if_ros_scheduling: ## Returns the current docker safe/sanitized branch for adore_if_ros_scheduling
	@printf "%s\n" ${adore_if_ros_scheduling_tag}

.PHONY: image_adore_if_ros_scheduling
image_adore_if_ros_scheduling: ## Returns the current docker image name for adore_if_ros_scheduling
	@printf "%s\n" ${adore_if_ros_scheduling_image}

.PHONY: update_adore_if_ros_scheduling
update_adore_if_ros_scheduling:
	cd "${adore_if_ros_scheduling_MAKEFILE_PATH}" && git pull

endif