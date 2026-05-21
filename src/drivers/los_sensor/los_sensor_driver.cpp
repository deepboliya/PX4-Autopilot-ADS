#include "los_sensor_driver.hpp"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>

#include <cstdlib>

using namespace time_literals;

ModuleBase::Descriptor LOSSensorDriver::desc{
	LOSSensorDriver::task_spawn,
	LOSSensorDriver::custom_command,
	LOSSensorDriver::print_usage
};

// ==========================================================
// CRC32 — poly 0xEDB88320, init 0xFFFFFFFF, final XOR ~
// Identical implementation must run on the Jetson side.
// ==========================================================
uint32_t LOSSensorDriver::crc32_compute(const uint8_t *data, size_t length)
{
	uint32_t crc = 0xFFFFFFFF;

	for (size_t i = 0; i < length; i++) {
		crc ^= data[i];

		for (int j = 0; j < 8; j++) {
			if (crc & 1) {
				crc = (crc >> 1) ^ 0xEDB88320;
			} else {
				crc >>= 1;
			}
		}
	}

	return ~crc;
}

// ==========================================================
// BAUD CONVERSION — numeric baud -> termios speed_t
// ==========================================================
speed_t LOSSensorDriver::baud_to_speed(uint32_t baud)
{
	switch (baud) {
	case 9600:    return B9600;
	case 19200:   return B19200;
	case 38400:   return B38400;
	case 57600:   return B57600;
	case 115200:  return B115200;
	case 230400:  return B230400;
	case 460800:  return B460800;
	case 921600:  return B921600;
#ifdef B1500000
	case 1500000: return B1500000;
#endif
#ifdef B2000000
	case 2000000: return B2000000;
#endif
#ifdef B3000000
	case 3000000: return B3000000;
#endif
	default:      return 0;
	}
}

// ==========================================================
// SEND SYNC PACKET — pushes T1 into the ring for later matching
// ==========================================================
bool LOSSensorDriver::send_sync_packet()
{
	SyncPacket pkt{};
	pkt.magic          = SYNC_MAGIC;
	pkt.type           = SYNC_TYPE_REQUEST;
	pkt.seq            = _sync_seq++;
	pkt.px4_tx_time_us = hrt_absolute_time();

	int ret = ::write(_fd, &pkt, sizeof(pkt));

	if (ret == (int)sizeof(pkt)) {
		// Push T1 into the ring so the offset matcher can find it later.
		_t1_ring[_t1_ring_idx] = pkt.px4_tx_time_us;
		_t1_ring_idx = (_t1_ring_idx + 1) % T1_RING_SIZE;

		PX4_INFO("SYNC TX seq=%u t1=%llu",
			 pkt.seq,
			 (unsigned long long)pkt.px4_tx_time_us);
		return true;
	}

	PX4_WARN("SYNC TX failed");
	return false;
}

// ==========================================================
// CONSTRUCTOR
// ==========================================================
LOSSensorDriver::LOSSensorDriver(const char *device, uint32_t baud) :
	ScheduledWorkItem("los_sensor_driver", px4::wq_configurations::hp_default),
	_baud(baud)
{
	strncpy(_device, device, sizeof(_device) - 1);
	_device[sizeof(_device) - 1] = '\0';
}

// ==========================================================
// DESTRUCTOR
// ==========================================================
LOSSensorDriver::~LOSSensorDriver()
{
	ScheduleClear();

	if (_fd >= 0) {
		::close(_fd);
	}

	perf_free(_crc_errors);
	perf_free(_frames_ok);
	perf_free(_sequence_errors);
}

// ==========================================================
// INIT — 200 Hz Run() cadence (see plan §3c for trade-off analysis)
// ==========================================================
bool LOSSensorDriver::init()
{
	ScheduleOnInterval(5000_us);  // 200 Hz; mean kernel-buffer-to-Run() delay = 2.5 ms
	return true;
}

// ==========================================================
// FRAME PARSER — validates magic / version / CRC over 58 B
// ==========================================================
bool LOSSensorDriver::parse_frame(LOSFrame *frame)
{
	if (_rx_index < LOS_FRAME_SIZE) {
		return false;
	}

	memcpy(frame, _rx_buffer, LOS_FRAME_SIZE);
	_rx_index = 0;

	if (frame->magic != LOS_MAGIC) {
		PX4_WARN("Invalid magic 0x%02x", frame->magic);
		return false;
	}

	if (frame->version != LOS_VERSION) {
		PX4_WARN("Invalid version %u", frame->version);
		return false;
	}

	const uint32_t computed_crc = crc32_compute(
		reinterpret_cast<uint8_t *>(frame),
		LOS_FRAME_SIZE - sizeof(uint32_t));

	if (computed_crc != frame->crc32) {
		perf_count(_crc_errors);
		_crc_error_count++;
		PX4_WARN("CRC mismatch recv=0x%08lx calc=0x%08lx",
			 (unsigned long)frame->crc32,
			 (unsigned long)computed_crc);
		return false;
	}

	perf_count(_frames_ok);
	return true;
}

// ==========================================================
// MAIN RUN LOOP
// ==========================================================
void LOSSensorDriver::Run()
{
	// --- Open UART (once) ---
	if (_fd < 0) {
		_fd = ::open(_device, O_RDWR | O_NOCTTY | O_NONBLOCK);

		if (_fd < 0) {
			PX4_ERR("Failed to open %s", _device);
			return;
		}

		struct termios uart_config{};

		if (tcgetattr(_fd, &uart_config) < 0) {
			PX4_ERR("tcgetattr failed");
			return;
		}

		const speed_t speed = baud_to_speed(_baud);

		if (speed == 0) {
			PX4_ERR("Unsupported baud %lu", (unsigned long)_baud);
			::close(_fd);
			_fd = -1;
			return;
		}

		cfsetispeed(&uart_config, speed);
		cfsetospeed(&uart_config, speed);

		uart_config.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
		uart_config.c_cflag |=  (CS8 | CLOCAL | CREAD);
		uart_config.c_lflag &= ~(ECHO | ICANON | ISIG | IEXTEN);
		uart_config.c_iflag &= ~(IXON | IXOFF | IXANY);
		uart_config.c_oflag &= ~OPOST;

		tcsetattr(_fd, TCSANOW, &uart_config);

		PX4_INFO("LOS driver opened %s @ %lu baud", _device, (unsigned long)_baud);
	}

	// --- Send a sync packet every 1 s ---
	if (hrt_elapsed_time(&_last_test_tx) > 1_s) {
		send_sync_packet();
		_last_test_tx = hrt_absolute_time();
	}

	// --- Stale detection ---
	if (!_los_stale &&
	    _last_frame_time != 0 &&
	    hrt_elapsed_time(&_last_frame_time) > 1_s) {
		_los_stale = true;
		PX4_WARN("LOS DATA STALE");
	}

	// --- Read whatever is in the kernel UART buffer ---
	uint8_t temp_buffer[64];
	const int bytes_read = ::read(_fd, temp_buffer, sizeof(temp_buffer));

	if (bytes_read > 0) {
		// --- Byte-stream parser: anchor on LOS_MAGIC, fill 58 B ---
		for (int i = 0; i < bytes_read; i++) {
			if (_rx_index == 0 && temp_buffer[i] != LOS_MAGIC) {
				continue;
			}

			_rx_buffer[_rx_index++] = temp_buffer[i];

			if (_rx_index != LOS_FRAME_SIZE) {
				continue;
			}

			LOSFrame frame{};

			if (!parse_frame(&frame)) {
				continue;
			}

			// --- Sequence-gap detection ---
			if (!_first_packet) {
				const uint16_t expected = _last_seq + 1;
				if (frame.seq != expected) {
					perf_count(_sequence_errors);
					_packet_loss += (uint16_t)(frame.seq - expected);
					PX4_WARN("Packet loss expected=%u got=%u",
						 expected, frame.seq);
				}
			} else {
				_first_packet = false;
			}
			_last_seq = frame.seq;

			// --- Offset estimation (4-timestamp PING-PONG, T1 ring search) ---
			//   T1 = an entry in _t1_ring   (PX4 hrt µs at sync send)
			//   T2 = frame.t_sync_jetson_us (Jetson clock at sync RX)
			//   T3 = frame.jetson_capture_time_us (Jetson clock at TX)
			//   T4 = hrt_absolute_time()    (PX4 hrt µs at frame RX)
			// offset = (T1+T4)/2 − (T2+T3)/2   — Jetson processing delay (T3−T2) cancels.
			const uint64_t T4 = hrt_absolute_time();
			double  raw_offset_us = 0.0;
			bool    valid_sync_sample = false;

			if (frame.t_sync_jetson_us != 0) {
				const uint64_t T2 = frame.t_sync_jetson_us;
				const uint64_t T3 = frame.jetson_capture_time_us;
				const int64_t  jetson_wait = (int64_t)T3 - (int64_t)T2;

				if (jetson_wait >= 0 && jetson_wait < 100000) {
					// Pick the T1 in the ring whose implied wire RTT is the smallest
					// plausible (positive, < 10 ms) value.  Robust to sync loss.
					uint64_t best_T1 = 0;
					int64_t  best_wire_rtt = 10001;

					for (size_t ri = 0; ri < T1_RING_SIZE; ri++) {
						const uint64_t candidate_t1 = _t1_ring[ri];

						if (candidate_t1 == 0) {
							continue;
						}

						const int64_t total_rtt = (int64_t)T4 - (int64_t)candidate_t1;
						const int64_t wire_rtt  = total_rtt - jetson_wait;

						if (wire_rtt > 0 && wire_rtt < 10000 && wire_rtt < best_wire_rtt) {
							best_wire_rtt = wire_rtt;
							best_T1       = candidate_t1;
						}
					}

					if (best_T1 != 0) {
						raw_offset_us =
							((double)best_T1 + (double)T4) * 0.5
							- ((double)T2 + (double)T3) * 0.5;
						valid_sync_sample = true;
					}
				}
			}

			// --- Filter the offset; latch sync_valid on first good sample ---
			if (valid_sync_sample) {
				if (!_offset_initialized) {
					_filtered_offset_us = raw_offset_us;
					_offset_initialized = true;
					// Reset monotonicity baseline so the first sync-corrected timestamp
					// isn't clamped to a stale pre-sync placeholder.
					_last_converted_timestamp = 0;
				} else {
					_filtered_offset_us = 0.90 * _filtered_offset_us + 0.10 * raw_offset_us;
				}
			}

			// --- Convert jetson_capture_time_us to PX4 hrt µs ---
			int64_t converted_timestamp_us;

			if (_offset_initialized) {
				converted_timestamp_us =
					(int64_t)frame.jetson_capture_time_us + (int64_t)_filtered_offset_us;

				if (converted_timestamp_us <= 0) {
					converted_timestamp_us = (int64_t)hrt_absolute_time();
				}
			} else {
				// Pre-sync: stamp with PX4 receive time so downstream age checks behave.
				converted_timestamp_us = (int64_t)hrt_absolute_time();
			}

			// --- Time-reversal protection ---
			if ((uint64_t)converted_timestamp_us <= _last_converted_timestamp) {
				converted_timestamp_us = _last_converted_timestamp + 1;
			}
			_last_converted_timestamp = (uint64_t)converted_timestamp_us;

			// --- End-to-end latency (inference start on Jetson -> PX4 RX, in PX4 hrt µs) ---
			//
			//   total = (T4 - T3 - offset) + L_j  =  px4_side_overhead + jetson_side_latency
			//
			// Only meaningful once the clock offset is initialised; pre-sync we report
			// just the Jetson half so the field is never garbage.
			uint32_t total_latency_us;

			if (_offset_initialized) {
				const int64_t px4_overhead =
					(int64_t)T4
					- (int64_t)frame.jetson_capture_time_us
					- (int64_t)_filtered_offset_us;

				// Clamp negative values caused by offset jitter (should never be
				// more than a few µs negative in a settled link; treat as 0).
				const int64_t overhead_clamped = (px4_overhead < 0) ? 0 : px4_overhead;

				const uint64_t total =
					(uint64_t)overhead_clamped + (uint64_t)frame.jetson_latency_us;

				total_latency_us = (total > UINT32_MAX) ? UINT32_MAX : (uint32_t)total;
			} else {
				total_latency_us = (uint32_t)frame.jetson_latency_us;
			}

			// --- Publish the uORB sample ---
			los_sensor_s msg{};
			msg.timestamp          = (uint64_t)converted_timestamp_us;
			msg.device_id          = 12345;
			msg.seq                = frame.seq;
			msg.azimuth            = frame.azimuth;
			msg.elevation          = frame.elevation;
			msg.azimuth_rate       = frame.azimuth_rate;
			msg.elevation_rate     = frame.elevation_rate;
			msg.azimuth_var        = frame.azimuth_var;
			msg.elevation_var      = frame.elevation_var;
			msg.azimuth_rate_var   = frame.azimuth_rate_var;
			msg.elevation_rate_var = frame.elevation_rate_var;
			msg.quality            = frame.quality;
			msg.status_flags       = frame.status_flags;
			msg.jetson_latency_us  = (uint32_t)frame.jetson_latency_us;
			msg.total_latency_us   = total_latency_us;
			_pub.publish(msg);

			// Track per-1s-window latency min/avg/max for the DIAG line.
			const uint32_t lat = (uint32_t)frame.jetson_latency_us;
			if (lat < _lat_min_us) { _lat_min_us = lat; }
			if (lat > _lat_max_us) { _lat_max_us = lat; }
			_lat_sum_us += lat;
			_lat_count++;

			// Mirror the aggregate for end-to-end latency.
			if (total_latency_us < _tot_min_us) { _tot_min_us = total_latency_us; }
			if (total_latency_us > _tot_max_us) { _tot_max_us = total_latency_us; }
			_tot_sum_us += total_latency_us;

			_total_frames++;
			_last_frame_time = hrt_absolute_time();
			_los_stale       = false;
		}
	}

	// --- Diagnostics: publish at 1 Hz regardless of frame arrival ---
	if (hrt_elapsed_time(&_last_diag_pub) > 1_s) {
		los_diagnostics_s diag{};
		diag.timestamp    = hrt_absolute_time();
		diag.total_frames = _total_frames;
		diag.crc_errors   = _crc_error_count;
		diag.packet_loss  = _packet_loss;
		diag.stale        = _los_stale;
		diag.sync_valid   = _offset_initialized;
		_diag_pub.publish(diag);

		_last_diag_pub = hrt_absolute_time();

		// Compute averages for the (short) DIAG print.  Min/max are still
		// tracked in _lat_min_us / _lat_max_us / _tot_min_us / _tot_max_us
		// so they can be exposed via uORB later without re-instrumenting.
		const uint32_t lat_avg_print = (_lat_count > 0)
			? (uint32_t)(_lat_sum_us / _lat_count)
			: 0;
		const uint32_t tot_avg_print = (_lat_count > 0)
			? (uint32_t)(_tot_sum_us / _lat_count)
			: 0;

		PX4_INFO("DIAG frames=%lu crc=%lu loss=%lu stale=%u sync=%u "
			 "lat_j_avg=%lu us lat_tot_avg=%lu us n=%lu",
			 (unsigned long)diag.total_frames,
			 (unsigned long)diag.crc_errors,
			 (unsigned long)diag.packet_loss,
			 (unsigned)diag.stale,
			 (unsigned)diag.sync_valid,
			 (unsigned long)lat_avg_print,
			 (unsigned long)tot_avg_print,
			 (unsigned long)_lat_count);

		// Reset aggregates for the next 1s window.
		_lat_min_us = UINT32_MAX;
		_lat_max_us = 0;
		_lat_sum_us = 0;
		_lat_count  = 0;

		_tot_min_us = UINT32_MAX;
		_tot_max_us = 0;
		_tot_sum_us = 0;
	}
}

// ==========================================================
// TASK SPAWN — parses -d <device> and -b <baud> from argv
// ==========================================================
int LOSSensorDriver::task_spawn(int argc, char *argv[])
{
	const char *device = "/dev/ttyS3";
	uint32_t    baud   = LOS_DEFAULT_BAUD;

	int        myoptind = 1;
	int        ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:b:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		case 'b':
			baud = (uint32_t)atoi(myoptarg);
			break;

		default:
			return print_usage("unknown option");
		}
	}

	LOSSensorDriver *instance = new LOSSensorDriver(device, baud);

	if (!instance) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	desc.object.store(instance);
	desc.task_id = task_id_is_work_queue;

	if (!instance->init()) {
		delete instance;
		desc.object.store(nullptr);
		desc.task_id = -1;
		return PX4_ERROR;
	}

	PX4_INFO("LOS sensor driver started on %s @ %lu baud",
		 device, (unsigned long)baud);

	return PX4_OK;
}

// ==========================================================
// CUSTOM COMMAND
// ==========================================================
int LOSSensorDriver::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

// ==========================================================
// PRINT USAGE
// ==========================================================
int LOSSensorDriver::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
UART driver for an external LOS bearing sensor (e.g. a Jetson running
the YOLOv5 detector in cv_los_publisher_uart.py).  Reads 58-byte
LOSFrames at ~30 Hz, computes a clock offset against the companion via
a 4-timestamp PING-PONG, and publishes to the `los_sensor` uORB topic
in PX4 hrt µs.  Link health is reported on `los_diagnostics`.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("los_sensor_driver", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "UART device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 921600, 9600, 3000000, "Baud rate", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

// ==========================================================
// MAIN ENTRY
// ==========================================================
extern "C" __EXPORT int los_sensor_driver_main(int argc, char *argv[])
{
	return ModuleBase::main(LOSSensorDriver::desc, argc, argv);
}
