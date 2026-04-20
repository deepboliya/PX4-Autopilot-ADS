#include "LosMeasurementsListener.hpp"

#include <inttypes.h>

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
		}
	}

	const hrt_abstime now = hrt_absolute_time();

	if ((_last_report_timestamp == 0) || ((now - _last_report_timestamp) >= 1'000'000)) {
		const uint64_t messages_since_last_report = _messages_received - _messages_received_last_report;
		_messages_received_last_report = _messages_received;
		_last_report_timestamp = now;

		if (_has_sample) {
			const uint64_t usable_latency_us = (now >= _latest_sample.timestamp) ? (now - _latest_sample.timestamp) : 0;
			const int64_t transport_minus_inference_us = static_cast<int64_t>(usable_latency_us)
								    - static_cast<int64_t>(_latest_sample.inference_time_us);

			PX4_INFO("LOS total=%" PRIu64 " (+%" PRIu64 "/1s) alpha=%.4f beta=%.4f alpha_rate=%.4f beta_rate=%.4f inference=%" PRIu32 " us ts=%" PRIu64,
				 _messages_received,
				 messages_since_last_report,
				 (double)_latest_sample.alpha,
				 (double)_latest_sample.beta,
				 (double)_latest_sample.alpha_rate,
				 (double)_latest_sample.beta_rate,
				 _latest_sample.inference_time_us,
				 _latest_sample.timestamp);

			PX4_INFO("LOS usable_latency=%" PRIu64 " us transport_minus_inference=%" PRId64 " us",
				 usable_latency_us,
				 transport_minus_inference_us);

		} else {
			PX4_INFO("LOS total=%" PRIu64 " (+%" PRIu64 "/1s) waiting for first sample", _messages_received,
				 messages_since_last_report);
			PX4_INFO_RAW("LOS total=%" PRIu64 " (+%" PRIu64 "/1s) waiting for first sample\n", _messages_received,
				     messages_since_last_report);
		}
	}
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
	PX4_INFO("messages received: %" PRIu64, _messages_received);
	PX4_INFO("messages in last report window: %" PRIu64, _messages_received - _messages_received_last_report);

	if (_has_sample) {
		const hrt_abstime now = hrt_absolute_time();
		const uint64_t usable_latency_us = (now >= _latest_sample.timestamp) ? (now - _latest_sample.timestamp) : 0;
		const int64_t transport_minus_inference_us = static_cast<int64_t>(usable_latency_us)
							    - static_cast<int64_t>(_latest_sample.inference_time_us);

		PX4_INFO("last sample timestamp: %" PRIu64 " us", _last_sample_timestamp);
		PX4_INFO("latest inference_time: %" PRIu32 " us", _latest_sample.inference_time_us);
		PX4_INFO("usable_latency: %" PRIu64 " us", usable_latency_us);
		PX4_INFO("transport_minus_inference: %" PRId64 " us", transport_minus_inference_us);
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
