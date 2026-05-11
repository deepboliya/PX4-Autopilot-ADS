/****************************************************************************
 *
 *   Custom Multicopter Rate Controller - flat single-file replica of
 *   mc_rate_control. All rate-loop math (PID, non-linear I factor with
 *   integrator clamp, saturation handling, feed-forward, yaw torque LP,
 *   battery scaling) is implemented inline below.
 *
 ****************************************************************************/

#include "custom_mc_rate_control.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;
using namespace time_literals;

ModuleBase::Descriptor CustomMCRateControl::desc{task_spawn, custom_command, print_usage};

CustomMCRateControl::CustomMCRateControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	_loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle");
	_loop_interval_perf = perf_alloc(PC_INTERVAL, MODULE_NAME ": interval");

	parameters_updated();
	_controller_status_pub.advertise();
}

CustomMCRateControl::~CustomMCRateControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool CustomMCRateControl::init()
{
	if (!_vehicle_angular_velocity_sub.registerCallback()) {
		PX4_ERR("vehicle_angular_velocity callback registration failed");
		return false;
	}

	return true;
}

void CustomMCRateControl::parameters_updated()
{
	// Convert parallel (P + I/s + sD) form to the ideal (K * [1 + 1/sTi + sTd]) form
	// by multiplying P/I/D by the per-axis overall gain K.
	const Vector3f rate_k(_param_mc_rollrate_k.get(),
			      _param_mc_pitchrate_k.get(),
			      _param_mc_yawrate_k.get());

	_gain_p = rate_k.emult(Vector3f(_param_mc_rollrate_p.get(),
					_param_mc_pitchrate_p.get(),
					_param_mc_yawrate_p.get()));

	_gain_i = rate_k.emult(Vector3f(_param_mc_rollrate_i.get(),
					_param_mc_pitchrate_i.get(),
					_param_mc_yawrate_i.get()));

	_gain_d = rate_k.emult(Vector3f(_param_mc_rollrate_d.get(),
					_param_mc_pitchrate_d.get(),
					_param_mc_yawrate_d.get()));

	_lim_int = Vector3f(_param_mc_rr_int_lim.get(),
			    _param_mc_pr_int_lim.get(),
			    _param_mc_yr_int_lim.get());

	_gain_ff = Vector3f(_param_mc_rollrate_ff.get(),
			    _param_mc_pitchrate_ff.get(),
			    _param_mc_yawrate_ff.get());

	_output_lpf_yaw.setCutoffFreq(_param_mc_yaw_tq_cutoff.get());
}

void CustomMCRateControl::Run()
{
	if (should_exit()) {
		_vehicle_angular_velocity_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s pu;
		_parameter_update_sub.copy(&pu);
		updateParams();
		parameters_updated();
	}

	vehicle_angular_velocity_s angular_velocity;

	if (_vehicle_angular_velocity_sub.update(&angular_velocity)) {
		const hrt_abstime now = angular_velocity.timestamp_sample;
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.000125f, 0.02f);
		_last_run = now;

		// Construct rate / accel vectors directly from message buffers (zero copy)
		const Vector3f rates{angular_velocity.xyz};
		const Vector3f angular_accel{angular_velocity.xyz_derivative};

		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		// Latch new rate setpoint
		vehicle_rates_setpoint_s vehicle_rates_setpoint;

		if (_vehicle_rates_setpoint_sub.update(&vehicle_rates_setpoint)) {
			_rates_setpoint(0) = PX4_ISFINITE(vehicle_rates_setpoint.roll)  ? vehicle_rates_setpoint.roll  : rates(0);
			_rates_setpoint(1) = PX4_ISFINITE(vehicle_rates_setpoint.pitch) ? vehicle_rates_setpoint.pitch : rates(1);
			_rates_setpoint(2) = PX4_ISFINITE(vehicle_rates_setpoint.yaw)   ? vehicle_rates_setpoint.yaw   : rates(2);
			_thrust_setpoint = Vector3f(vehicle_rates_setpoint.thrust_body);
		}

		// Only do work when rate control is requested.
		if (_vehicle_control_mode.flag_control_rates_enabled) {

			// Reset integrator when disarmed (mirrors mc_rate_control disarm reset path).
			if (!_vehicle_control_mode.flag_armed) {
				_rate_int.zero();
			}

			// Pull control allocator saturation flags so the integrator can stop winding up
			// in directions where actuators are already saturated.
			control_allocator_status_s control_allocator_status;

			if (_control_allocator_status_sub.update(&control_allocator_status)) {
				Vector<bool, 3> sat_pos;
				Vector<bool, 3> sat_neg;

				if (!control_allocator_status.torque_setpoint_achieved) {
					for (size_t i = 0; i < 3; i++) {
						if (control_allocator_status.unallocated_torque[i] > FLT_EPSILON) {
							sat_pos(i) = true;

						} else if (control_allocator_status.unallocated_torque[i] < -FLT_EPSILON) {
							sat_neg(i) = true;
						}
					}
				}

				_sat_pos = sat_pos;
				_sat_neg = sat_neg;
			}

			// =====================================================================
			// Inline rate-loop math (replica of RateControl::update / updateIntegral)
			// =====================================================================
			Vector3f rate_error = _rates_setpoint - rates;

			// PID with feed-forward: tau = P*err + I - D*ang_accel + FF*rate_sp
			Vector3f torque_setpoint =
				_gain_p.emult(rate_error)
				+ _rate_int
				- _gain_d.emult(angular_accel)
				+ _gain_ff.emult(_rates_setpoint);

			// Integral update: do not integrate when "landed/disarmed" (here gated by armed flag).
			// The non-linear I factor reduces the I gain as the rate error grows. With the
			// hard-coded reference of 400 deg/s, up to 100 deg/s the factor is ~1 (no effect),
			// at 200 deg/s the factor is ~0.75, at 400 deg/s it is 0.
			if (_vehicle_control_mode.flag_armed) {
				for (int i = 0; i < 3; i++) {
					// Stop integrating into a saturated direction
					if (_sat_pos(i)) {
						rate_error(i) = math::min(rate_error(i), 0.f);
					}

					if (_sat_neg(i)) {
						rate_error(i) = math::max(rate_error(i), 0.f);
					}

					float i_factor = rate_error(i) / math::radians(400.f);
					i_factor = math::max(0.0f, 1.f - i_factor * i_factor);

					const float rate_i = _rate_int(i) + i_factor * _gain_i(i) * rate_error(i) * dt;

					if (PX4_ISFINITE(rate_i)) {
						_rate_int(i) = math::constrain(rate_i, -_lim_int(i), _lim_int(i));
					}
				}
			}

			// Low-pass filter the yaw torque to reduce high-frequency content caused by
			// rotor acceleration coupling.
			torque_setpoint(2) = _output_lpf_yaw.update(torque_setpoint(2), dt);

			// Build status (use stored integrator after the update step above so the
			// reported value matches the one that will be applied next cycle).
			rate_ctrl_status_s rate_ctrl_status{};
			rate_ctrl_status.rollspeed_integ  = _rate_int(0);
			rate_ctrl_status.pitchspeed_integ = _rate_int(1);
			rate_ctrl_status.yawspeed_integ   = _rate_int(2);
			rate_ctrl_status.timestamp = hrt_absolute_time();
			_controller_status_pub.publish(rate_ctrl_status);

			// Build thrust + torque output (zero-copy via copyTo for thrust vector).
			vehicle_thrust_setpoint_s vehicle_thrust_setpoint{};
			vehicle_torque_setpoint_s vehicle_torque_setpoint{};

			_thrust_setpoint.copyTo(vehicle_thrust_setpoint.xyz);
			vehicle_torque_setpoint.xyz[0] = PX4_ISFINITE(torque_setpoint(0)) ? torque_setpoint(0) : 0.f;
			vehicle_torque_setpoint.xyz[1] = PX4_ISFINITE(torque_setpoint(1)) ? torque_setpoint(1) : 0.f;
			vehicle_torque_setpoint.xyz[2] = PX4_ISFINITE(torque_setpoint(2)) ? torque_setpoint(2) : 0.f;

			// Battery scaling: keep parity with mc_rate_control behavior.
			if (_param_mc_bat_scale_en.get()) {
				if (_battery_status_sub.updated()) {
					battery_status_s battery_status;

					if (_battery_status_sub.copy(&battery_status) && battery_status.connected
					    && battery_status.scale > 0.f) {
						_battery_status_scale = battery_status.scale;
					}
				}

				if (_battery_status_scale > 0.f) {
					for (int i = 0; i < 3; i++) {
						vehicle_thrust_setpoint.xyz[i] =
							math::constrain(vehicle_thrust_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
						vehicle_torque_setpoint.xyz[i] =
							math::constrain(vehicle_torque_setpoint.xyz[i] * _battery_status_scale, -1.f, 1.f);
					}
				}
			}

			const hrt_abstime t = hrt_absolute_time();

			vehicle_thrust_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_thrust_setpoint.timestamp = t;
			_vehicle_thrust_setpoint_pub.publish(vehicle_thrust_setpoint);

			vehicle_torque_setpoint.timestamp_sample = angular_velocity.timestamp_sample;
			vehicle_torque_setpoint.timestamp = t;
			_vehicle_torque_setpoint_pub.publish(vehicle_torque_setpoint);

			updateActuatorControlsStatus(vehicle_torque_setpoint, dt);
		}
	}

	perf_end(_loop_perf);
}

void CustomMCRateControl::updateActuatorControlsStatus(const vehicle_torque_setpoint_s &torque_sp, float dt)
{
	for (int i = 0; i < 3; i++) {
		_control_energy[i] += torque_sp.xyz[i] * torque_sp.xyz[i] * dt;
	}

	_energy_integration_time += dt;

	if (_energy_integration_time > 500e-3f) {
		actuator_controls_status_s status{};
		status.timestamp = torque_sp.timestamp;

		for (int i = 0; i < 3; i++) {
			status.control_power[i] = _control_energy[i] / _energy_integration_time;
			_control_energy[i] = 0.f;
		}

		_actuator_controls_status_pub.publish(status);
		_energy_integration_time = 0.f;
	}
}

int CustomMCRateControl::task_spawn(int argc, char *argv[])
{
	CustomMCRateControl *instance = new CustomMCRateControl();

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

int CustomMCRateControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CustomMCRateControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Flat single-file multicopter rate controller, functionally equivalent to
mc_rate_control. Implements the PID rate loop with non-linear I factor,
integrator clamp, control-allocator saturation handling, feed-forward,
yaw-torque low-pass filter, battery scaling, and rate_ctrl_status telemetry.

Input  : vehicle_angular_velocity, vehicle_rates_setpoint, vehicle_control_mode,
         control_allocator_status, battery_status
Output : vehicle_torque_setpoint, vehicle_thrust_setpoint, rate_ctrl_status,
         actuator_controls_status_0

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("custom_mc_rate_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int custom_mc_rate_control_main(int argc, char *argv[])
{
	return ModuleBase::main(CustomMCRateControl::desc, argc, argv);
}
