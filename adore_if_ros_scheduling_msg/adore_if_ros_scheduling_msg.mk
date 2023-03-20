# This Makefile contains useful targets that can be included in downstream projects.

ifndef adore_if_ros_scheduling_msg_MAKEFILE_PATH

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
adore_if_ros_scheduling_msg_project=adore_if_ros_scheduling_msg
ADORE_IF_ROS_SCHEDULING_MSG_PROJECT=${adore_if_ros_scheduling_msg_project}

adore_if_ros_scheduling_msg_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
make_gadgets_PATH:=${adore_if_ros_scheduling_msg_MAKEFILE_PATH}/../make_gadgets
REPO_DIRECTORY:=${adore_if_ros_scheduling_msg_MAKEFILE_PATH}

adore_if_ros_scheduling_msg_tag:=$(shell cd ${make_gadgets_PATH} && make get_sanitized_branch_name REPO_DIRECTORY=${REPO_DIRECTORY})
ADORE_IF_ROS_SCHEDULING_MSG_TAG=${adore_if_ros_scheduling_msg_tag}

adore_if_ros_scheduling_msg_image=${adore_if_ros_scheduling_msg_project}:${adore_if_ros_scheduling_msg_tag}
ADORE_IF_ROS_SCHEDULING_MSG_IMAGE=${adore_if_ros_scheduling_msg_image}

.PHONY: build_adore_if_ros_scheduling_msg 
build_adore_if_ros_scheduling_msg: ## Build adore_if_ros_scheduling_msg
	cd "${adore_if_ros_scheduling_msg_MAKEFILE_PATH}" && make

.PHONY: clean_adore_if_ros_scheduling_msg
clean_adore_if_ros_scheduling_msg: ## Clean adore_if_ros_scheduling_msg build artifacts
	cd "${adore_if_ros_scheduling_msg_MAKEFILE_PATH}" && make clean

.PHONY: branch_adore_if_ros_scheduling_msg
branch_adore_if_ros_scheduling_msg: ## Returns the current docker safe/sanitized branch for adore_if_ros_scheduling_msg
	@printf "%s\n" ${adore_if_ros_scheduling_msg_tag}

.PHONY: image_adore_if_ros_scheduling_msg
image_adore_if_ros_scheduling_msg: ## Returns the current docker image name for adore_if_ros_scheduling_msg
	@printf "%s\n" ${adore_if_ros_scheduling_msg_image}

.PHONY: update_adore_if_ros_scheduling_msg
update_adore_if_ros_scheduling_msg:
	cd "${adore_if_ros_scheduling_msg_MAKEFILE_PATH}" && git pull

endif