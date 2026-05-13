#pragma once

#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/los_measurements.h> // for UXRCE-DDS based LosMeasurements.msg topic
#include <uORB/topics/los_sensor.h> // for UART based Los_sensor.msg topic
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_rates_setpoint.h>

class LosGuidanceTargetTracking : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static ModuleBase::Descriptor desc;

	LosGuidanceTargetTracking();
	~LosGuidanceTargetTracking() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;
	void parameters_updated();
	bool compute_acceleration_command_ned(matrix::Vector3f &acceleration_ned) const;
	void publish_offboard_setpoint(const matrix::Vector3f &acceleration_ned);

	uORB::SubscriptionCallbackWorkItem _los_measurements_sub{this, ORB_ID(los_measurements)}; // for UXRCE-DDS based LosMeasurements.msg topic
	uORB::SubscriptionCallbackWorkItem _los_sensor_sub{this, ORB_ID(los_sensor)}; // for UART based LOS measurements
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_rates_setpoint_s> _rates_sp_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	los_measurements_s _latest_sample{};
	bool _has_sample{false};
	hrt_abstime _last_sample_timestamp{0};

	matrix::Quatf _q_attitude{1.f, 0.f, 0.f, 0.f};
	bool _has_attitude{false};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::LOS_GD_ACC_MAX>) _param_los_gd_acc_max,
		(ParamFloat<px4::params::LOS_GD_GIMB_PIT>) _param_los_gd_gimb_pit
	)
};
