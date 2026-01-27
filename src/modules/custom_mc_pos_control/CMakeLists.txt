############################################################################
#
#   Custom Minimal Position Controller for X500 Quadcopter
#
############################################################################

px4_add_module(
	MODULE modules__custom_mc_pos_control
	MAIN custom_mc_pos_control
	COMPILE_FLAGS
	SRCS
		custom_mc_pos_control.cpp
		custom_mc_pos_control.hpp
	DEPENDS
		mathlib
		geo
		px4_work_queue
)
