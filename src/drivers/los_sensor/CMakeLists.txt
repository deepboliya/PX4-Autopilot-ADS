px4_add_module(
        MODULE drivers__los_sensor
        MAIN los_sensor_driver
        SRCS
                los_sensor_driver.cpp
                los_sensor_driver.hpp
        DEPENDS
                px4_work_queue
)