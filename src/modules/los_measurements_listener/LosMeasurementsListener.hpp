#pragma once

#include <drivers/drv_hrt.h>
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

	los_measurements_s _latest_sample{};
	bool _has_sample{false};

	uint64_t _messages_received{0};
	uint64_t _messages_received_last_report{0};
	hrt_abstime _last_sample_timestamp{0};
	hrt_abstime _last_report_timestamp{0};
};
