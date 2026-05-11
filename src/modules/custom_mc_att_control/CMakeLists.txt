############################################################################
#
#   Custom Multicopter Attitude Controller - flat single-file replica of
#   mc_att_control. All quaternion math is contained in the .cpp.
#
############################################################################

px4_add_module(
	MODULE modules__custom_mc_att_control
	MAIN custom_mc_att_control
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		custom_mc_att_control.cpp
		custom_mc_att_control.hpp
	DEPENDS
		mathlib
		px4_work_queue
		SlewRate
)
