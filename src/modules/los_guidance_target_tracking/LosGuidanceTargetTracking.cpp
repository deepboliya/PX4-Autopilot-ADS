#include "LosGuidanceTargetTracking.hpp"

#include <inttypes.h>
#include <math.h>

using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

ModuleBase::Descriptor LosGuidanceTargetTracking::desc{task_spawn, custom_command, print_usage};

LosGuidanceTargetTracking::LosGuidanceTargetTracking() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	parameters_updated();
}

bool LosGuidanceTargetTracking::init()
{
	// Register callbacks on BOTH subscriptions so either transport wakes
	// the work item.  UART path is preferred in Run() (see below).
	if (!_los_sensor_sub.registerCallback()) {
		PX4_ERR("los_sensor callback registration failed");
		return false;
	}

	if (!_los_measurements_sub.registerCallback()) {
		PX4_ERR("los_measurements callback registration failed");
		return false;
	}

	return true;
}

void LosGuidanceTargetTracking::parameters_updated()
{
	updateParams();
}

void LosGuidanceTargetTracking::Run()
{
	if (should_exit()) {
		_los_sensor_sub.unregisterCallback();
		_los_measurements_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pu;
		_parameter_update_sub.copy(&pu);
		parameters_updated();
	}

	vehicle_attitude_s att;
	if (_vehicle_attitude_sub.update(&att)) {
		_q_attitude = Quatf(att.q);
		_has_attitude = true;
	}

	los_measurements_s los_meas;
	los_sensor_s los_sensor_meas;

	// UART path is primary.  DDS path is fallback so the controller keeps
	// working if the UART driver or the Jetson sender goes silent.
	if (_los_sensor_sub.update(&los_sensor_meas)) {
		_latest_sample.alpha = los_sensor_meas.azimuth;
		_latest_sample.beta = los_sensor_meas.elevation;
		_has_sample = true;
		_last_sample_timestamp = los_sensor_meas.timestamp;

		// Control yaw to reduce alpha using a proportional controller
		vehicle_rates_setpoint_s rates_sp{};
		rates_sp.timestamp = hrt_absolute_time();
		rates_sp.roll = 0.0f;
		rates_sp.pitch = 0.0f;
		rates_sp.yaw = los_sensor_meas.azimuth * 1.5f;
		_rates_sp_pub.publish(rates_sp);

		// Control servo angle using PWM to reduce beta
		vehicle_command_s vehicle_cmd{};
		vehicle_cmd.timestamp = hrt_absolute_time();
		vehicle_cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR;

		vehicle_cmd.param1 = NAN;
		vehicle_cmd.param2 = NAN;
		vehicle_cmd.param3 = NAN;
		vehicle_cmd.param4 = NAN;

		float servo_val = los_sensor_meas.elevation * 0.5f;
		if (servo_val > 1.0f) servo_val = 1.0f;
		if (servo_val < -1.0f) servo_val = -1.0f;

		vehicle_cmd.param5 = servo_val; // Command servo on channel 5
		vehicle_cmd.param6 = NAN;
		vehicle_cmd.param7 = 0; // Index 0 (controls actuators 1-6)

		vehicle_cmd.target_system = 1;
		vehicle_cmd.target_component = 1;
		vehicle_cmd.source_system = 1;
		vehicle_cmd.source_component = 1;
		vehicle_cmd.from_external = false;

		_vehicle_command_pub.publish(vehicle_cmd);

		Vector3f acceleration_ned;
		if (compute_acceleration_command_ned(acceleration_ned)) {
			publish_offboard_setpoint(acceleration_ned);
		}
	}
	else if (_los_measurements_sub.update(&los_meas)) {
		_latest_sample = los_meas;
		_has_sample = true;
		_last_sample_timestamp = los_meas.timestamp;

		// Control yaw to reduce alpha using a proportional controller
		vehicle_rates_setpoint_s rates_sp{};
		rates_sp.timestamp = hrt_absolute_time();
		rates_sp.roll = 0.0f;
		rates_sp.pitch = 0.0f;
		// If alpha is positive, target is to the right, we turn right (positive yaw rate)
		rates_sp.yaw = los_meas.alpha * 1.5f;
		_rates_sp_pub.publish(rates_sp);

		// Control servo angle using PWM to reduce beta
		vehicle_command_s vehicle_cmd{};
		vehicle_cmd.timestamp = hrt_absolute_time();
		vehicle_cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR;

		vehicle_cmd.param1 = NAN;
		vehicle_cmd.param2 = NAN;
		vehicle_cmd.param3 = NAN;
		vehicle_cmd.param4 = NAN;

		float servo_val = los_meas.beta * 0.5f;
		if (servo_val > 1.0f) servo_val = 1.0f;
		if (servo_val < -1.0f) servo_val = -1.0f;

		vehicle_cmd.param5 = servo_val; // Command servo on channel 5
		vehicle_cmd.param6 = NAN;
		vehicle_cmd.param7 = 0; // Index 0 (controls actuators 1-6)

		vehicle_cmd.target_system = 1;
		vehicle_cmd.target_component = 1;
		vehicle_cmd.source_system = 1;
		vehicle_cmd.source_component = 1;
		vehicle_cmd.from_external = false;

		_vehicle_command_pub.publish(vehicle_cmd);

		Vector3f acceleration_ned;
		if (compute_acceleration_command_ned(acceleration_ned)) {
			publish_offboard_setpoint(acceleration_ned);
		}
	}
}

int LosGuidanceTargetTracking::task_spawn(int argc, char *argv[])
{
	LosGuidanceTargetTracking *instance = new LosGuidanceTargetTracking();

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

int LosGuidanceTargetTracking::print_status()
{
	PX4_INFO("Running");

	if (_has_sample) {
		PX4_INFO("last bearing a=%.3f b=%.3f age=%" PRIu64 " us",
			 (double)_latest_sample.alpha,
			 (double)_latest_sample.beta,
			 (uint64_t)(hrt_absolute_time() - _last_sample_timestamp));
	}

	return 0;
}

int LosGuidanceTargetTracking::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int LosGuidanceTargetTracking::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Targets tracking module utilizing los_measurements to control yaw and a servo to converge alpha and beta angles to zero.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("los_guidance_target_tracking", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

bool LosGuidanceTargetTracking::compute_acceleration_command_ned(Vector3f &acceleration_ned) const
{
	if (!_has_sample || !_has_attitude) {
		return false;
	}

	const float alpha = _latest_sample.alpha;
	const float beta  = _latest_sample.beta;

	const float cos_a = cosf(alpha);
	const float sin_a = sinf(alpha);
	const float cos_b = cosf(beta);
	const float sin_b = sinf(beta);

	const Vector3f los_gimbal{cos_a * cos_b, sin_a * cos_b, -sin_b};

	const Quatf q_body_from_gimbal(Eulerf(0.f, _param_los_gd_gimb_pit.get(), 0.f));
	const Vector3f los_body = q_body_from_gimbal.rotateVector(los_gimbal);

	const Vector3f los_ned = _q_attitude.rotateVector(los_body);

	const float norm = los_ned.norm();

	if (!PX4_ISFINITE(norm) || norm < 1e-6f) {
		return false;
	}

	acceleration_ned = los_ned * (_param_los_gd_acc_max.get() / norm);
	return PX4_ISFINITE(acceleration_ned(0))
	       && PX4_ISFINITE(acceleration_ned(1))
	       && PX4_ISFINITE(acceleration_ned(2));
}

void LosGuidanceTargetTracking::publish_offboard_setpoint(const Vector3f &acceleration_ned)
{
	const hrt_abstime now = hrt_absolute_time();

	offboard_control_mode_s ocm{};
	ocm.timestamp = now;
	ocm.position = false;
	ocm.velocity = false;
	ocm.acceleration = true;
	ocm.attitude = false;
	ocm.body_rate = false;
	ocm.thrust_and_torque = false;
	ocm.direct_actuator = false;
	_offboard_control_mode_pub.publish(ocm);

	trajectory_setpoint_s sp{};
	sp.timestamp = now;
	sp.position[0] = NAN;
	sp.position[1] = NAN;
	sp.position[2] = NAN;
	sp.velocity[0] = NAN;
	sp.velocity[1] = NAN;
	sp.velocity[2] = NAN;
	sp.acceleration[0] = acceleration_ned(0);
	sp.acceleration[1] = acceleration_ned(1);
	sp.acceleration[2] = acceleration_ned(2);
	sp.jerk[0] = NAN;
	sp.jerk[1] = NAN;
	sp.jerk[2] = NAN;
	sp.yaw = NAN;
	sp.yawspeed = NAN;
	_trajectory_setpoint_pub.publish(sp);
}

extern "C" __EXPORT int los_guidance_target_tracking_main(int argc, char *argv[])
{
	return ModuleBase::main(LosGuidanceTargetTracking::desc, argc, argv);
}
