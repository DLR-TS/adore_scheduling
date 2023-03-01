# This Makefile contains useful targets that can be included in downstream projects.

ifndef adore_scheduling_MAKEFILE_PATH

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
adore_scheduling_project=adore_scheduling
ADORE_SCHEDULING_PROJECT=${adore_scheduling_project}

adore_scheduling_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
make_gadgets_PATH:=${adore_scheduling_MAKEFILE_PATH}/make_gadgets
REPO_DIRECTORY:=${adore_scheduling_MAKEFILE_PATH}

adore_scheduling_tag:=$(shell cd ${make_gadgets_PATH} && make get_sanitized_branch_name REPO_DIRECTORY=${REPO_DIRECTORY})
ADORE_SCHEDULING_TAG=${adore_scheduling_tag}

adore_scheduling_image=${adore_scheduling_project}:${adore_scheduling_tag}
ADORE_SCHEDULING_IMAGE=${adore_scheduling_image}

include ${adore_scheduling_MAKEFILE_PATH}/adore_scheduler/adore_scheduler.mk


.PHONY: build_adore_scheduling 
build_adore_scheduling: build_lib_adore_scheduling build_adore_if_ros_scheduling_msg build_adore_if_ros_scheduling build_adore_scheduler ## Build adore_scheduling
	

.PHONY: clean_adore_scheduling
clean_adore_scheduling: clean_lib_adore_scheduling clean_adore_if_ros_scheduling_msg clean_adore_if_ros_scheduling clean_adore_scheduler ## Clean adore_scheduling build artifacts
	

.PHONY: branch_adore_scheduling
branch_adore_scheduling: branch_lib_adore_scheduling branch_adore_if_ros_scheduling_msg branch_adore_if_ros_scheduling branch_adore_scheduler ## Returns the current docker safe/sanitized branch for the projects within adore_scheduling
	
.PHONY: image_adore_scheduling
image_adore_scheduling: image_lib_adore_scheduling image_adore_if_ros_scheduling_msg image_adore_if_ros_scheduling image_adore_scheduler ## Returns the current docker image names for the projects within adore_scheduling
	

.PHONY: update_adore_scheduling
update_adore_scheduling:
	cd "${adore_scheduling_MAKEFILE_PATH}" && git pull

endif