# This Makefile contains useful targets that can be included in downstream projects.

ifndef lib_adore_scheduling_MAKEFILE_PATH

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
lib_adore_scheduling_project=lib_adore_scheduling
LIB_ADORE_SCHEDULING_PROJECT=${lib_adore_scheduling_project}

lib_adore_scheduling_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
make_gadgets_PATH:=${lib_adore_scheduling_MAKEFILE_PATH}/../make_gadgets
REPO_DIRECTORY:=${lib_adore_scheduling_MAKEFILE_PATH}

lib_adore_scheduling_tag:=$(shell cd ${make_gadgets_PATH} && make get_sanitized_branch_name REPO_DIRECTORY=${REPO_DIRECTORY})
LIB_ADORE_SCHEDULING_TAG=${lib_adore_scheduling_tag}

lib_adore_scheduling_image=${lib_adore_scheduling_project}:${lib_adore_scheduling_tag}
LIB_ADORE_SCHEDULING_IMAGE=${lib_adore_scheduling_image}

.PHONY: build_lib_adore_scheduling 
build_lib_adore_scheduling: ## Build lib_adore_scheduling
	cd "${lib_adore_scheduling_MAKEFILE_PATH}" && make

.PHONY: clean_lib_adore_scheduling
clean_lib_adore_scheduling: ## Clean lib_adore_scheduling build artifacts
	cd "${lib_adore_scheduling_MAKEFILE_PATH}" && make clean

.PHONY: branch_lib_adore_scheduling
branch_lib_adore_scheduling: ## Returns the current docker safe/sanitized branch for lib_adore_scheduling
	@printf "%s\n" ${lib_adore_scheduling_tag}

.PHONY: image_lib_adore_scheduling
image_lib_adore_scheduling: ## Returns the current docker image name for lib_adore_scheduling
	@printf "%s\n" ${lib_adore_scheduling_image}

.PHONY: update_lib_adore_scheduling
update_lib_adore_scheduling:
	cd "${lib_adore_scheduling_MAKEFILE_PATH}" && git pull

endif