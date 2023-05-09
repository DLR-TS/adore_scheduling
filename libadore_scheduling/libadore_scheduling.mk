# This Makefile contains useful targets that can be included in downstream projects.

project:=libadore_scheduling
PROJECT:=$(shell echo ${project}| tr '[:lower:]' '[:upper:]')

ifeq ($(call if,$(wildcard ${PROJECT}),),)
${PROJECT}:

MAKEFLAGS += --no-print-directory

.EXPORT_ALL_VARIABLES:
${project}_PROJECT:=${project}

${project}_MAKEFILE_PATH:=$(shell realpath "$(shell dirname "$(lastword $(MAKEFILE_LIST))")")
ifeq ($(SUBMODULES_PATH),)
    ${project}_SUBMODULES_PATH:=$(shell realpath "${${project}_MAKEFILE_PATH}/..")
else
    ${project}_SUBMODULES_PATH:=$(shell realpath ${SUBMODULES_PATH})
endif

_SUBMODULES_PATH:=${${project}_SUBMODULES_PATH}

MAKE_GADGETS_PATH:=${${project}_SUBMODULES_PATH}/make_gadgets
ifeq ($(wildcard $(MAKE_GADGETS_PATH)/*),)
    $(info INFO: To clone submodules use: 'git submodules update --init --recursive')
    $(info INFO: To specify alternative path for submodules use: SUBMODULES_PATH="<path to submodules>" make build')
    $(info INFO: Default submodule path is: ${${project}_MAKEFILE_PATH}')
    $(error "ERROR: ${MAKE_GADGETS_PATH} does not exist. Did you clone the submodules?")
endif

REPO_DIRECTORY:=${${project}_MAKEFILE_PATH}
${project}_TAG:=$(shell cd ${MAKE_GADGETS_PATH} && make get_sanitized_branch_name REPO_DIRECTORY=${REPO_DIRECTORY})
${project}_IMAGE:=${${project}_PROJECT}:${${project}_TAG}

${project}_CMAKE_BUILD_PATH:="${${project}_PROJECT}/build"
${project}_CMAKE_INSTALL_PATH:="${${project}_CMAKE_BUILD_PATH}/install"

define project_RULE
	$(eval TARGET_PREFIX := clean) 
	$(eval TARGET := $@)
    project := $(shell echo ${TARGET} | sed 's|${TARGET_PREFIX}_||g')
endef

.PHONY: build_${project} 
build_${project}: ## Build ${project} 
	@$(eval $(project_RULE))
	cd "${${project}_MAKEFILE_PATH}" && make build

.PHONY: clean_${project}
clean_${project}: $(eval $(project_RULE))## Clean ${project} build artifacts
	@$(eval $(project_RULE))
	cd "${${project}_MAKEFILE_PATH}" && make clean

.PHONY: branch_${project}
branch_${project}: ## Returns the current docker safe/sanitized branch for ${project} 
	@$(eval $(project_RULE))
	@printf "%s\n" ${${project}_TAG}

.PHONY: image_${project}
image_${project}: ## Returns the current docker image name for ${project}
	@$(eval $(project_RULE))
	@printf "%s\n" ${${project}_IMAGE}

include ${MAKE_GADGETS_PATH}/make_gadgets.mk
include ${MAKE_GADGETS_PATH}/docker/docker-tools.mk

endif
