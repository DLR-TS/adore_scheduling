include ../make_module/utils.mk

.EXPORT_ALL_VARIABLES:
$(call init)


include .${project}.mk
include ${${project}_submodules_path}/make_gadgets/make_gadgets.mk
include ${${project}_submodules_path}/adore_if_ros_scheduling_msg/include.mk
