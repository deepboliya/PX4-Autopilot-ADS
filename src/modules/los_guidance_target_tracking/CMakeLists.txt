px4_add_module(
	MODULE modules__los_guidance_target_tracking
	MAIN los_guidance_target_tracking
	SRCS
		LosGuidanceTargetTracking.cpp
	DEPENDS
		px4_work_queue
)
