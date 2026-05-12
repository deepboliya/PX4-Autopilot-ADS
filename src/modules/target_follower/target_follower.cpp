#include "target_follower.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/events.h>

TargetFollower::TargetFollower() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool TargetFollower::init()
{
	if (!_los_measurements_sub.registerCallback()) {
		PX4_ERR("los_measurements callback registration failed");
		return false;
	}
	return true;
}

void TargetFollower::Run()
{
	if (should_exit()) {
		_los_measurements_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	los_measurements_s los_meas;

	if (_los_measurements_sub.update(&los_meas)) {
		// Control yaw to reduce alpha
		// Proportional controller for yaw rate based on alpha
		vehicle_rates_setpoint_s rates_sp{};
		rates_sp.timestamp = hrt_absolute_time();
		rates_sp.roll = 0.0f;
		rates_sp.pitch = 0.0f;
		// If alpha is positive, target is to the right, we need to turn right (positive yaw rate)
		rates_sp.yaw = los_meas.alpha * 1.5f; // P-gain of 1.5
		_rates_sp_pub.publish(rates_sp);

		// Provide PWM to servo connected to channel 5 to reduce beta
		// We use VEHICLE_CMD_DO_SET_ACTUATOR to output PWM on channel 5
		// Beta is mapped to servo value. Assuming -1 to 1 corresponds to min/max PWM.
		vehicle_command_s vehicle_cmd{};
		vehicle_cmd.timestamp = hrt_absolute_time();
		vehicle_cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR;
		
		// Param 1 to 6 handle actuators 1 to 6. Index 0 means 1-6. Channel 5 -> param5
		vehicle_cmd.param1 = NAN;
		vehicle_cmd.param2 = NAN;
		vehicle_cmd.param3 = NAN;
		vehicle_cmd.param4 = NAN;
		
		// To decrease beta, move the servo proportionally
		// Let's assume servo input is -1 to 1
		float servo_val = los_meas.beta * 0.5f; 
		if (servo_val > 1.0f) servo_val = 1.0f;
		if (servo_val < -1.0f) servo_val = -1.0f;
		
		vehicle_cmd.param5 = servo_val; // servo on channel 5
		vehicle_cmd.param6 = NAN;
		vehicle_cmd.param7 = 0; // index 0 means actuator 1-6
		
		vehicle_cmd.target_system = 1;
		vehicle_cmd.target_component = 1;
		vehicle_cmd.source_system = 1;
		vehicle_cmd.source_component = 1;
		vehicle_cmd.from_external = false;
		
		_vehicle_command_pub.publish(vehicle_cmd);
	}
}

int TargetFollower::task_spawn(int argc, char *argv[])
{
	TargetFollower *instance = new TargetFollower();
	if (instance) {
		_object.alloc(instance);
		_task_id = task_id_is_work_queue;
		if (instance->init()) {
			return PX4_OK;
		} else {
			PX4_ERR("alloc failed");
		}
	}
	return PX4_ERROR;
}

int TargetFollower::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int TargetFollower::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Target Follower module. Subscribe to los_measurements, control yaw and servo pitch to channel 5.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("target_follower", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int target_follower_main(int argc, char *argv[])
{
	return TargetFollower::main(argc, argv);
}
