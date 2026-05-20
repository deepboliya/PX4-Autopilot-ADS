#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/log.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/los_sensor.h>
#include <uORB/topics/los_diagnostics.h>

#include <lib/perf/perf_counter.h>

#include <drivers/drv_hrt.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

using namespace time_literals;

// Wire-protocol constants.  Companion (Jetson) side must use identical
// values -- see los_sender_standalone.cpp / cv_los_publisher_uart.py.
static constexpr uint8_t  LOS_MAGIC         = 0xA5;
static constexpr uint8_t  LOS_VERSION       = 1;
static constexpr uint8_t  SYNC_MAGIC        = 0x5A;
static constexpr uint8_t  SYNC_TYPE_REQUEST = 1;
static constexpr uint32_t LOS_DEFAULT_BAUD  = 921600;

#pragma pack(push, 1)

// Sent FMU -> Jetson once per second so the companion can stamp T2
// (its clock at sync RX) into the next outgoing LOS frame.
struct SyncPacket
{
	uint8_t  magic;             // SYNC_MAGIC (0x5A)
	uint8_t  type;              // SYNC_TYPE_REQUEST (1)
	uint16_t seq;
	uint64_t px4_tx_time_us;    // T1: PX4 hrt µs at send
};

// Sent Jetson -> FMU at ~30 Hz.  Frame layout MUST stay byte-identical
// to the companion-side struct.
struct LOSFrame
{
	uint8_t  magic;                  // LOS_MAGIC (0xA5)
	uint8_t  version;                // LOS_VERSION (1)
	uint16_t seq;

	float    azimuth;
	float    elevation;
	float    azimuth_rate;
	float    elevation_rate;
	float    azimuth_var;
	float    elevation_var;
	float    azimuth_rate_var;
	float    elevation_rate_var;

	uint8_t  quality;
	uint8_t  status_flags;

	uint64_t t_sync_jetson_us;       // T2: Jetson clock at last sync RX
	uint64_t jetson_capture_time_us; // T3: Jetson clock at frame build

	uint32_t crc32;                  // poly 0xEDB88320, init 0xFFFFFFFF, final XOR ~
};

#pragma pack(pop)

static constexpr size_t LOS_FRAME_SIZE = 58;
static_assert(sizeof(LOSFrame) == LOS_FRAME_SIZE, "LOSFrame size mismatch");
static_assert(sizeof(SyncPacket) == 12, "SyncPacket size mismatch");

class LOSSensorDriver :
	public ModuleBase,
	public px4::ScheduledWorkItem
{
public:
	LOSSensorDriver(const char *device, uint32_t baud);
	~LOSSensorDriver() override;

	static Descriptor desc;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	bool parse_frame(LOSFrame *frame);
	bool send_sync_packet();
	static uint32_t crc32_compute(const uint8_t *data, size_t length);

	// Map a numeric baud rate to the termios speed_t constant.
	// Returns 0 if unsupported.
	static speed_t baud_to_speed(uint32_t baud);

	int     _fd{-1};
	char    _device[64]{};
	uint32_t _baud{LOS_DEFAULT_BAUD};

	uint8_t _rx_buffer[LOS_FRAME_SIZE]{};
	size_t  _rx_index{0};

	// Sync request bookkeeping.
	hrt_abstime _last_test_tx{0};
	uint16_t    _sync_seq{0};

	// 16-slot ring of recently-sent T1 timestamps.  When an LOS frame
	// arrives the matcher picks the T1 whose implied wire RTT is the
	// most plausible -- robust to sync packet loss without a wire-format
	// change.
	static constexpr size_t T1_RING_SIZE = 16;
	uint64_t _t1_ring[T1_RING_SIZE]{};
	size_t   _t1_ring_idx{0};

	// Sequence-gap detection.
	bool     _first_packet{true};
	uint16_t _last_seq{0};

	// Clock-sync state.
	double   _filtered_offset_us{0.0};
	bool     _offset_initialized{false};
	uint64_t _last_converted_timestamp{0};

	// Stale-data detection.
	hrt_abstime _last_frame_time{0};
	bool        _los_stale{false};

	// Diagnostics counters.
	uint32_t _total_frames{0};
	uint32_t _crc_error_count{0};
	uint32_t _packet_loss{0};
	hrt_abstime _last_diag_pub{0};

	perf_counter_t _crc_errors      = perf_alloc(PC_COUNT, "los_crc_errors");
	perf_counter_t _frames_ok       = perf_alloc(PC_COUNT, "los_frames_ok");
	perf_counter_t _sequence_errors = perf_alloc(PC_COUNT, "los_seq_errors");

	uORB::Publication<los_sensor_s>      _pub{ORB_ID(los_sensor)};
	uORB::Publication<los_diagnostics_s> _diag_pub{ORB_ID(los_diagnostics)};
};

extern "C" __EXPORT int los_sensor_driver_main(int argc, char *argv[]);
