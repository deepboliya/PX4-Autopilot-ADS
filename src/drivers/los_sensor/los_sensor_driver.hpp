#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/log.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/los_sensor.h>

#include <lib/perf/perf_counter.h>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>

using namespace time_literals;

static constexpr uint8_t LOS_MAGIC   = 0xA5;
static constexpr uint8_t LOS_VERSION = 1;
static constexpr uint32_t LOS_BAUD   = 921600;

#pragma pack(push, 1)

struct LOSFrame
{
    uint8_t  magic;
    uint8_t  version;
    uint16_t seq;

    float azimuth;
    float elevation;

    float azimuth_rate;
    float elevation_rate;

    float azimuth_var;
    float elevation_var;

    float azimuth_rate_var;
    float elevation_rate_var;

    uint8_t  quality;
    uint8_t  status_flags;

    uint32_t crc32;
};

#pragma pack(pop)

static constexpr size_t LOS_FRAME_SIZE = 42;

static_assert(
    sizeof(LOSFrame) == LOS_FRAME_SIZE,
    "LOSFrame size mismatch"
);

// V1.17 ARCHITECTURE: Inherit from ModuleBase without the <T> template
class LOSSensorDriver :
    public ModuleBase,
    public px4::ScheduledWorkItem
{
public:
    LOSSensorDriver(const char *device);
    ~LOSSensorDriver() override;

    // V1.17 ARCHITECTURE: Explicitly declare the descriptor
    static Descriptor desc;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    bool init();

private:
    void Run() override;

    bool parse_frame(LOSFrame *frame);

    static uint32_t crc32_compute(
        const uint8_t *data,
        size_t length
    );

private:
    int _fd{-1};

    char _device[32]{};

    static constexpr size_t BUFFER_SIZE = 512;

    uint8_t _buffer[BUFFER_SIZE]{};
    size_t _buffer_len{0};

    uint8_t _rx_buffer[LOS_FRAME_SIZE]{};
    size_t _rx_index{0};

    uint16_t _last_seq{0};
    bool _first_packet{true};

    perf_counter_t _crc_errors =
        perf_alloc(PC_COUNT, "los_crc_errors");

    perf_counter_t _frames_ok =
        perf_alloc(PC_COUNT, "los_frames_ok");

    perf_counter_t _sequence_errors =
        perf_alloc(PC_COUNT, "los_seq_errors");

    uORB::Publication<los_sensor_s> _pub
    {
        ORB_ID(los_sensor)
    };
};