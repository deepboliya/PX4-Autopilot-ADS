px4_add_module(
	MODULE modules__los_measurements_listener
	MAIN los_measurements_listener
	SRCS
		LosMeasurementsListener.cpp
		LosMeasurementsListener.hpp
	DEPENDS
		px4_work_queue
)
