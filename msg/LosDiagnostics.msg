# Diagnostics from the UART LOS sensor driver (los_sensor_driver)
# Published once per second alongside los_sensor frames.

uint64 timestamp           # [us] PX4 hrt µs since boot

uint32 total_frames        # cumulative valid frames received
uint32 crc_errors          # cumulative frames rejected on CRC mismatch
uint32 packet_loss         # cumulative gaps detected from sequence-number jumps

bool   stale               # true if no frame has been received for >1 s
bool   sync_valid          # true once the clock-offset estimator has initialised
