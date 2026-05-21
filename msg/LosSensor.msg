uint64 timestamp                # time since system start (microseconds)

uint32 device_id                # unique device ID

uint16 seq                      # frame sequence number

float32 azimuth                 # azimuth angle (rad)
float32 elevation               # elevation angle (rad)

float32 azimuth_rate            # azimuth angular rate (rad/s)
float32 elevation_rate          # elevation angular rate (rad/s)

float32 azimuth_var             # azimuth variance (rad^2)
float32 elevation_var           # elevation variance (rad^2)

float32 azimuth_rate_var        # azimuth rate variance (rad/s)^2
float32 elevation_rate_var      # elevation rate variance (rad/s)^2

uint8 quality                   # LOS quality (0-255)
uint8 status_flags              # LOS status flags

uint32 jetson_latency_us        # Jetson cap.read() to link.send() latency (us)
uint32 total_latency_us         # End-to-end: inference start on Jetson -> PX4 RX (us, after clock sync)
