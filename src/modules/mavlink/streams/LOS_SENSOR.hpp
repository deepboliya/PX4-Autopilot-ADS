#ifndef LOS_SENSOR_HPP
#define LOS_SENSOR_HPP

#include <uORB/topics/los_sensor.h>

class MavlinkStreamLosSensor : public MavlinkStream
{
public:
    static MavlinkStream *new_instance(Mavlink *mavlink)
    {
        return new MavlinkStreamLosSensor(mavlink);
    }

    static constexpr const char *get_name_static()
    {
        return "LOS_SENSOR";
    }

    static constexpr uint16_t get_id_static()
    {
        return MAVLINK_MSG_ID_LOS_SENSOR;
    }

    const char *get_name() const override
    {
        return get_name_static();
    }

    uint16_t get_id() override
    {
        return get_id_static();
    }
    unsigned get_size() override
    {
        return _los_sensor_sub.advertised() ?
               MAVLINK_MSG_ID_LOS_SENSOR_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
    }

private:

    explicit MavlinkStreamLosSensor(Mavlink *mavlink) : MavlinkStream(mavlink) {}

    uORB::Subscription _los_sensor_sub{ORB_ID(los_sensor)};

    bool send() override
    {
        los_sensor_s sensor{};

        if (_los_sensor_sub.update(&sensor)) {

            mavlink_los_sensor_t msg{};

            msg.timestamp = sensor.timestamp;
            msg.device_id = sensor.device_id;
            msg.seq = sensor.seq;

            msg.azimuth = sensor.azimuth;
            msg.elevation = sensor.elevation;

            msg.azimuth_rate = sensor.azimuth_rate;
            msg.elevation_rate = sensor.elevation_rate;

            msg.azimuth_var = sensor.azimuth_var;
            msg.elevation_var = sensor.elevation_var;

            msg.azimuth_rate_var = sensor.azimuth_rate_var;
            msg.elevation_rate_var = sensor.elevation_rate_var;

            msg.quality = sensor.quality;
            msg.status_flags = sensor.status_flags;

            msg.jetson_latency_us = sensor.jetson_latency_us;
            msg.total_latency_us = sensor.total_latency_us;

            mavlink_msg_los_sensor_send_struct(_mavlink->get_channel(), &msg);

            return true;
        }

        return false;
    }
};

#endif // LOS_SENSOR_HPP