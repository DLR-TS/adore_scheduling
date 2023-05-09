# This Makefile contains useful targets that can be included in downstream projects.

PROJECT:=$(shell echo ${project}| tr '[:lower:]' '[:upper:]')

$(info INFO: loading module: ${project})

ifeq ($(filter .${project}.mk, $(notdir $(MAKEFILE_LIST))), .${project}.mk)

.EXPORT_ALL_VARIABLES:
MAKEFLAGS += --no-print-directory

${project}_project:=${project}

${project}_makefile_path:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
${project}_path:=${${project}_makefile_path}

ifeq ($(submodules_path),)
    ${project}_submodules_path:=$(shell realpath "${${project}_path}")
else
    ${project}_submodules_path:=$(shell realpath ${submodules_path})
endif

make_gadgets_path:=${${project}_submodules_path}/make_gadgets
ifeq ($(wildcard $(make_gadgets_path)/*),)
    $(info INFO: To clone submodules use: 'git submodules update --init --recursive')
    $(info INFO: To specify alternative path for submodules use: SUBMODULES_PATH="<path to submodules>" make build')
    $(info INFO: Default submodule path is: ${${project}_makefile_path}')
    $(error "ERROR: ${make_gadgets_path} does not exist. Did you clone the submodules?")
endif

REPO_DIRECTORY:=${${project}_makefile_path}
${project}_tag:=$(shell cd ${make_gadgets_path} && make get_sanitized_branch_name REPO_DIRECTORY=${REPO_DIRECTORY})
${project}_image:=${${project}_project}:${${project}_tag}

${project}_CMAKE_BUILD_PATH:="${${project}_project}/build"
${project}_CMAKE_INSTALL_PATH:="${${project}_CMAKE_BUILD_PATH}/install"

include ${make_gadgets_path}/make_gadgets.mk
include ${make_gadgets_path}/docker/docker-tools.mk

.PHONY: build_${project} 
build_${project}: ## Build ${project} 
	cd "${${project}_makefile_path}" && make build

.PHONY: clean_${project}
clean_${project}: ## Clean ${project} build artifacts
	cd "${${project}_makefile_path}" && make clean

.PHONY: branch_${project}
branch_${project}: ## Returns the current docker safe/sanitized branch for ${project} 
	@printf "%s\n" ${${project}_tag}

.PHONY: image_${project}
image_${project}: ## Returns the current docker image name for ${project}
	@printf "%s\n" ${${project}_image}

endif
