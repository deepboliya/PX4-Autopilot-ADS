#include "los_sensor_driver.hpp"

#include <px4_platform_common/px4_config.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

// ==========================================================
// CRC32 FUNCTION
// ==========================================================

uint32_t LOSSensorDriver::crc32_compute(
    const uint8_t *data,
    size_t length
)
{
    uint32_t crc = 0xFFFFFFFF;

    for (size_t i = 0; i < length; i++) {

        crc ^= data[i];

        for (int j = 0; j < 8; j++) {

            if (crc & 1) {

                crc =
                    (crc >> 1)
                    ^ 0xEDB88320;

            } else {

                crc >>= 1;
            }
        }
    }

    return ~crc;
}

// ==========================================================
// CONSTRUCTOR
// ==========================================================

LOSSensorDriver::LOSSensorDriver(
    const char *device
) :
    ScheduledWorkItem(
        "los_sensor_driver",
        px4::wq_configurations::hp_default
    )
{
    strncpy(
        _device,
        device,
        sizeof(_device) - 1
    );

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
// INIT
// ==========================================================

bool LOSSensorDriver::init()
{
    ScheduleOnInterval(5000_us);

    return true;
}

// ==========================================================
// FRAME PARSER
// ==========================================================

bool LOSSensorDriver::parse_frame(
    LOSFrame *frame
)
{
    if (_rx_index < LOS_FRAME_SIZE) {

        return false;
    }

    memcpy(
        frame,
        _rx_buffer,
        LOS_FRAME_SIZE
    );

    _rx_index = 0;

    // ======================================================
    // MAGIC CHECK
    // ======================================================

    if (frame->magic != LOS_MAGIC) {

        PX4_WARN("Invalid magic");

        return false;
    }

    // ======================================================
    // VERSION CHECK
    // ======================================================

    if (frame->version != LOS_VERSION) {

        PX4_WARN("Invalid version");

        return false;
    }

    // ======================================================
    // CRC CHECK
    // ======================================================

    uint32_t computed_crc =
        crc32_compute(
            reinterpret_cast<uint8_t *>(frame),
            LOS_FRAME_SIZE - sizeof(uint32_t)
        );

    if (computed_crc != frame->crc32) {

        perf_count(_crc_errors);

        PX4_WARN(
            "CRC mismatch recv=0x%08lx calc=0x%08lx",
            (unsigned long)frame->crc32,
            (unsigned long)computed_crc
        );

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
    // ======================================================
    // OPEN UART
    // ======================================================

    if (_fd < 0) {

        _fd = ::open(
            _device,
            O_RDWR | O_NOCTTY | O_NONBLOCK
        );

        if (_fd < 0) {

            PX4_ERR(
                "Failed to open %s",
                _device
            );

            return;
        }

        struct termios uart_config {};

        if (tcgetattr(_fd, &uart_config) < 0) {

            PX4_ERR("tcgetattr failed");

            return;
        }

        // ==================================================
        // UART CONFIG
        // ==================================================

        cfsetispeed(&uart_config, B921600);
        cfsetospeed(&uart_config, B921600);

        uart_config.c_cflag &=
            ~(PARENB | CSTOPB | CSIZE);

        uart_config.c_cflag |=
            (CS8 | CLOCAL | CREAD);

        uart_config.c_lflag &=
            ~(ECHO | ICANON | ISIG | IEXTEN);

        uart_config.c_iflag &=
            ~(IXON | IXOFF | IXANY);

        uart_config.c_oflag &= ~OPOST;

        tcsetattr(
            _fd,
            TCSANOW,
            &uart_config
        );

        PX4_INFO(
            "LOS Driver opened %s",
            _device
        );
    }

    // ======================================================
    // READ UART
    // ======================================================

    uint8_t temp_buffer[64];

    int bytes_read =
        ::read(
            _fd,
            temp_buffer,
            sizeof(temp_buffer)
        );

    if (bytes_read <= 0) {

        return;
    }

    // ======================================================
    // BYTE STREAM PARSER
    // ======================================================

    for (int i = 0; i < bytes_read; i++) {

        // ==================================================
        // WAIT FOR MAGIC BYTE
        // ==================================================

        if (_rx_index == 0 &&
            temp_buffer[i] != LOS_MAGIC) {

            continue;
        }

        // ==================================================
        // STORE BYTE
        // ==================================================

        _rx_buffer[_rx_index++] =
            temp_buffer[i];

        // ==================================================
        // FRAME COMPLETE
        // ==================================================

        if (_rx_index == LOS_FRAME_SIZE) {

            LOSFrame frame {};

            if (!parse_frame(&frame)) {

                continue;
            }

            // ==============================================
            // PACKET LOSS DETECTION
            // ==============================================

            if (!_first_packet) {

                uint16_t expected =
                    _last_seq + 1;

                if (frame.seq != expected) {

                    perf_count(_sequence_errors);

                    PX4_WARN(
                        "Packet loss expected=%u got=%u",
                        expected,
                        frame.seq
                    );
                }

            } else {

                _first_packet = false;
            }

            _last_seq = frame.seq;

            // ==============================================
            // CREATE uORB MESSAGE
            // ==============================================

            los_sensor_s msg {};

            msg.timestamp =
                hrt_absolute_time();

            msg.device_id = 12345;

            msg.seq =
                frame.seq;

            msg.azimuth =
                frame.azimuth;

            msg.elevation =
                frame.elevation;

            msg.azimuth_rate =
                frame.azimuth_rate;

            msg.elevation_rate =
                frame.elevation_rate;

            msg.azimuth_var =
                frame.azimuth_var;

            msg.elevation_var =
                frame.elevation_var;

            msg.azimuth_rate_var =
                frame.azimuth_rate_var;

            msg.elevation_rate_var =
                frame.elevation_rate_var;

            msg.quality =
                frame.quality;

            msg.status_flags =
                frame.status_flags;

            // ==============================================
            // PUBLISH
            // ==============================================

            _pub.publish(msg);

            PX4_INFO(
                "LOS seq=%u az=%.2f el=%.2f",
                frame.seq,
                (double)frame.azimuth,
                (double)frame.elevation
            );
        }
    }
}

// ==========================================================
// TASK SPAWN
// ==========================================================

int LOSSensorDriver::task_spawn(
    int argc,
    char *argv[]
)
{
    const char *device = "/dev/ttyS3";

    if (argc >= 2) {

        device = argv[1];
    }

    LOSSensorDriver *instance =
        new LOSSensorDriver(device);

    if (!instance) {

        PX4_ERR("alloc failed");

        return PX4_ERROR;
    }

    _object.store(instance);
    _task_id = task_id_is_work_queue;

    if (!instance->init()) {

        delete instance;

        _object.store(nullptr);

        _task_id = -1;

        return PX4_ERROR;
    }

    PX4_INFO(
        "LOS sensor driver started on %s",
        device
    );

    return PX4_OK;
}

// ==========================================================
// CUSTOM COMMAND
// ==========================================================

int LOSSensorDriver::custom_command(
    int argc,
    char *argv[]
)
{
    return print_usage("unknown command");
}

// ==========================================================
// PRINT USAGE
// ==========================================================

int LOSSensorDriver::print_usage(
    const char *reason
)
{
    if (reason) {

        PX4_WARN("%s", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
LOS Sensor UART Driver
)DESCR_STR"
    );

    PRINT_MODULE_USAGE_NAME(
        "los_sensor_driver",
        "driver"
    );

    PRINT_MODULE_USAGE_COMMAND(
        "start"
    );

    PRINT_MODULE_USAGE_ARG(
        "<device>",
        "UART device",
        false
    );

    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

// ==========================================================
// MAIN ENTRY
// ==========================================================

extern "C" __EXPORT int
los_sensor_driver_main(
    int argc,
    char *argv[]
)
{
    return LOSSensorDriver::main(
        argc,
        argv
    );
}
