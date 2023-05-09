include ../make_module/utils.mk

.EXPORT_ALL_VARIABLES:
$(call load_environment)

include include.mk

include ../make_module/makefile_templates/ros_msg_project_makefile_template.mk

.PHONY: test 
test:
	$(call call_targets,target=clean,submodules=$(submodules))
