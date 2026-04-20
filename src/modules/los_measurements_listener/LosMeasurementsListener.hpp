#pragma once

#include <drivers/drv_hrt.h>
#include <stdint.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/los_measurements.h>

class LosMeasurementsListener : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	LosMeasurementsListener();
	~LosMeasurementsListener() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	uORB::SubscriptionCallbackWorkItem _los_measurements_sub{this, ORB_ID(los_measurements)};
	static constexpr uint8_t STATS_WINDOW_SIZE{100};

	los_measurements_s _latest_sample{};
	bool _has_sample{false};

	uint64_t _messages_received{0};
	uint64_t _messages_received_last_report{0};
	uint64_t _messages_last_1s{0};
	hrt_abstime _last_sample_timestamp{0};
	hrt_abstime _last_report_timestamp{0};

	uint8_t _stats_window_count{0};
	uint8_t _stats_window_head{0};
	uint64_t _latency_sum_us{0};
	double _latency_sum_sq_us{0.0};
	uint64_t _latency_window_us[STATS_WINDOW_SIZE]{};

	uint8_t _dt_window_count{0};
	uint8_t _dt_window_head{0};
	uint64_t _dt_sum_us{0};
	double _dt_sum_sq_us{0.0};
	uint64_t _dt_window_us[STATS_WINDOW_SIZE]{};

	hrt_abstime _prev_receive_timestamp{0};
	bool _has_prev_receive_timestamp{false};
};
