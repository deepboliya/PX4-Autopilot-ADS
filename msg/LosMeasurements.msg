# Line-of-sight (LOS) bearing estimates from external perception
# Publisher: uxrce_dds_client (DDS bridge)
# Subscriber: los_measurement_listener

uint64 timestamp # [us] Time since system start

float32 alpha # [rad] LOS bearing angle alpha
float32 beta # [rad] LOS bearing angle beta

float32 alpha_rate # [rad/s] Time derivative of alpha
float32 beta_rate # [rad/s] Time derivative of beta

uint32 inference_time_us # [us] Perception inference latency used to produce this estimate
