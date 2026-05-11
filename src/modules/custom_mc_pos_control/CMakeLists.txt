############################################################################
#
#   Custom Multicopter Position Controller - flat single-file replica of
#   mc_pos_control's core PositionControl + ControlMath. All cascaded PID,
#   tilt-limit, thrust-to-attitude conversion and EKF reset handling is
#   in the .cpp.
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
		SlewRate
)
