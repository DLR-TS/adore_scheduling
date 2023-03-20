# This Makefile contains useful targets that can be included in downstream projects.

ifndef adore_scheduler_MAKEFILE_PATH

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
adore_scheduler_project=adore_scheduler
ADORE_SCHEDULER_PROJECT=${adore_scheduler_project}

adore_scheduler_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
make_gadgets_PATH:=${adore_scheduler_MAKEFILE_PATH}/../make_gadgets
REPO_DIRECTORY:=${adore_scheduler_MAKEFILE_PATH}

adore_scheduler_tag:=$(shell cd ${make_gadgets_PATH} && make get_sanitized_branch_name REPO_DIRECTORY=${REPO_DIRECTORY})
ADORE_SCHEDULER_TAG=${adore_scheduler_tag}

adore_scheduler_image=${adore_scheduler_project}:${adore_scheduler_tag}
ADORE_SCHEDULER_IMAGE=${adore_scheduler_image}

include ${adore_scheduler_MAKEFILE_PATH}/../adore_if_ros_scheduling_msg/adore_if_ros_scheduling_msg.mk
include ${adore_scheduler_MAKEFILE_PATH}/../lib_adore_scheduling/lib_adore_scheduling.mk
include ${adore_scheduler_MAKEFILE_PATH}/../adore_if_ros_scheduling/adore_if_ros_scheduling.mk


.PHONY: build_adore_scheduler 
build_adore_scheduler: ## Build scheduler
	cd "${adore_scheduler_MAKEFILE_PATH}" && make

.PHONY: clean_adore_scheduler
clean_adore_scheduler: ## Clean scheduler build artifacts
	cd "${adore_scheduler_MAKEFILE_PATH}" && make clean

.PHONY: branch_adore_scheduler
branch_adore_scheduler: ## Returns the current docker safe/sanitized branch for scheduler
	@printf "%s\n" ${adore_scheduler_tag}

.PHONY: image_adore_scheduler
image_adore_scheduler: ## Returns the current docker image name for scheduler
	@printf "%s\n" ${adore_scheduler_image}

.PHONY: update_adore_scheduler
update_adore_scheduler:
	cd "${adore_scheduler_MAKEFILE_PATH}" && git pull

endif