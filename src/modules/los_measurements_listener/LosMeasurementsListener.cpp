#include "LosMeasurementsListener.hpp"

#include <inttypes.h>
#include <math.h>

namespace
{
void compute_mean_stddev(double sum, double sum_sq, uint8_t count, double &mean, double &stddev)
{
	mean = 0.0;
	stddev = 0.0;

	if (count == 0) {
		return;
	}

	mean = sum / static_cast<double>(count);
	double variance = (sum_sq / static_cast<double>(count)) - (mean * mean);

	if (variance < 0.0) {
		variance = 0.0;
	}

	stddev = sqrt(variance);
}
}

ModuleBase::Descriptor LosMeasurementsListener::desc{task_spawn, custom_command, print_usage};

LosMeasurementsListener::LosMeasurementsListener() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool LosMeasurementsListener::init()
{
	if (!_los_measurements_sub.registerCallback()) {
		PX4_ERR("los_measurements callback registration failed");
		return false;
	}

	// Keep a periodic heartbeat print on console even if no new samples arrive.
	ScheduleOnInterval(1'000'000);

	return true;
}

void LosMeasurementsListener::Run()
{
	if (should_exit()) {
		_los_measurements_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	los_measurements_s los_measurements{};

	while (_los_measurements_sub.updated()) {
		if (_los_measurements_sub.copy(&los_measurements)) {
			++_messages_received;
			_latest_sample = los_measurements;
			_has_sample = true;
			_last_sample_timestamp = los_measurements.timestamp;

			const hrt_abstime receive_timestamp = hrt_absolute_time();
			const uint64_t latency_us = (receive_timestamp >= los_measurements.timestamp) ?
						   (receive_timestamp - los_measurements.timestamp) : 0;

			if (_stats_window_count == STATS_WINDOW_SIZE) {
				// Evict the oldest sample (head points to oldest when full).
				const uint64_t old_latency_us = _latency_window_us[_stats_window_head];
				_latency_sum_us -= old_latency_us;
				_latency_sum_sq_us -= static_cast<double>(old_latency_us) * static_cast<double>(old_latency_us);

			} else {
				++_stats_window_count;
			}

			_latency_window_us[_stats_window_head] = latency_us;
			_latency_sum_us += latency_us;
			_latency_sum_sq_us += static_cast<double>(latency_us) * static_cast<double>(latency_us);
			_stats_window_head = static_cast<uint8_t>((_stats_window_head + 1) % STATS_WINDOW_SIZE);

			if (_has_prev_receive_timestamp) {
				const uint64_t dt_us = (receive_timestamp >= _prev_receive_timestamp) ?
						       (receive_timestamp - _prev_receive_timestamp) : 0;

				if (_dt_window_count == STATS_WINDOW_SIZE) {
					const uint64_t old_dt_us = _dt_window_us[_dt_window_head];
					_dt_sum_us -= old_dt_us;
					_dt_sum_sq_us -= static_cast<double>(old_dt_us) * static_cast<double>(old_dt_us);

				} else {
					++_dt_window_count;
				}

				_dt_window_us[_dt_window_head] = dt_us;
				_dt_sum_us += dt_us;
				_dt_sum_sq_us += static_cast<double>(dt_us) * static_cast<double>(dt_us);
				_dt_window_head = static_cast<uint8_t>((_dt_window_head + 1) % STATS_WINDOW_SIZE);
			}

			_prev_receive_timestamp = receive_timestamp;
			_has_prev_receive_timestamp = true;
		}
	}

	// const hrt_abstime now = hrt_absolute_time();

	// if ((_last_report_timestamp == 0) || ((now - _last_report_timestamp) >= 1'000'000)) {
	// 	const uint64_t messages_since_last_report = _messages_received - _messages_received_last_report;
	// 	_messages_received_last_report = _messages_received;
	// 	_messages_last_1s = messages_since_last_report;
	// 	_last_report_timestamp = now;

	// 	if (_has_sample) {
	// 		double dt_mean_us = 0.0;
	// 		double dt_std_us = 0.0;
	// 		double lat_mean_us = 0.0;
	// 		double lat_std_us = 0.0;

	// 		compute_mean_stddev(static_cast<double>(_dt_sum_us), _dt_sum_sq_us, _dt_window_count, dt_mean_us, dt_std_us);
	// 		compute_mean_stddev(static_cast<double>(_latency_sum_us), _latency_sum_sq_us, _stats_window_count, lat_mean_us,
	// 				   lat_std_us);

	// 		// PX4_INFO("1s=%" PRIu64 " a=%.3f b=%.3f ar=%.3f br=%.3f dt=%.1f/%.1f lat=%.1f/%.1f",
	// 		// 	 _messages_last_1s,
	// 		// 	 (double)_latest_sample.alpha,
	// 		// 	 (double)_latest_sample.beta,
	// 		// 	 (double)_latest_sample.alpha_rate,
	// 		// 	 (double)_latest_sample.beta_rate,
	// 		// 	 dt_mean_us,
	// 		// 	 dt_std_us,
	// 		// 	 lat_mean_us,
	// 		// 	 lat_std_us);

	// 	} else {
	// 		// PX4_INFO("1s=%" PRIu64 " wait", _messages_last_1s);
	// 	}
	// }
}

int LosMeasurementsListener::task_spawn(int argc, char *argv[])
{
	LosMeasurementsListener *instance = new LosMeasurementsListener();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int LosMeasurementsListener::print_status()
{
	PX4_INFO("rx=%" PRIu64 " 1s=%" PRIu64, _messages_received, _messages_last_1s);

	if (_has_sample) {
		double dt_mean_us = 0.0;
		double dt_std_us = 0.0;
		double lat_mean_us = 0.0;
		double lat_std_us = 0.0;

		compute_mean_stddev(static_cast<double>(_dt_sum_us), _dt_sum_sq_us, _dt_window_count, dt_mean_us, dt_std_us);
		compute_mean_stddev(static_cast<double>(_latency_sum_us), _latency_sum_sq_us, _stats_window_count, lat_mean_us,
				   lat_std_us);

		PX4_INFO("a=%.3f b=%.3f ar=%.3f br=%.3f lat_now=%.1f dt_now=%.1f",
			 (double)_latest_sample.alpha,
			 (double)_latest_sample.beta,
			 (double)_latest_sample.alpha_rate,
			 (double)_latest_sample.beta_rate,
			 (double)_latency_window_us[_stats_window_head],
			 (double)_dt_window_us[_dt_window_head]);
		PX4_INFO("dt m=%.1f s=%.1f n=%u", dt_mean_us, dt_std_us, (unsigned)_dt_window_count);
		PX4_INFO("lat m=%.1f s=%.1f n=%u", lat_mean_us, lat_std_us, (unsigned)_stats_window_count);
		PX4_INFO("last sample timestamp: %" PRIu64 " us", _last_sample_timestamp);
	}

	return 0;
}

int LosMeasurementsListener::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int LosMeasurementsListener::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Simple listener that subscribes to `los_measurements` and prints incoming data.

Use this to validate DDS-to-uORB transport for external perception LOS estimates.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("los_measurement_listener", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int los_measurement_listener_main(int argc, char *argv[])
{
	return ModuleBase::main(LosMeasurementsListener::desc, argc, argv);
}
