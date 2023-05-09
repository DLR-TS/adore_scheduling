
$(eval include ../make_module/utils.mk)
$(eval $(call load_environment))
$(eval include .${project}.mk )
$(eval include ../make_module/utils.mk)
$(eval include ${${project}_submodules_path}/make_gadgets/make_gadgets.mk) 
