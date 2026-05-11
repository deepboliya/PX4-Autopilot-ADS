/****************************************************************************
 *
 *   Custom Multicopter Attitude Controller - flat single-file replica of
 *   mc_att_control with the same math/control flow.
 *
 *   Publication documenting the implemented Quaternion Attitude Control:
 *   Nonlinear Quadrocopter Attitude Control (2013)
 *   by Dario Brescianini, Markus Hehn and Raffaello D'Andrea
 *   Institute for Dynamic Systems and Control (IDSC), ETH Zurich
 *
 *   https://www.research-collection.ethz.ch/bitstream/handle/20.500.11850/154099/eth-7387-01.pdf
 *
 ****************************************************************************/

#include "custom_mc_att_control.hpp"

#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <mathlib/math/Limits.hpp>
#include <mathlib/math/Functions.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/tasks.h>

using namespace matrix;
using namespace time_literals;

ModuleBase::Descriptor CustomAttitudeControl::desc{task_spawn, custom_command, print_usage};

CustomAttitudeControl::CustomAttitudeControl() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")),
	_loop_interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME ": interval"))
{
	_attitude_setpoint_q = Quatf(1.f, 0.f, 0.f, 0.f);

	// Slew rates copied from mc_att_control: gentle ramp on the manual throttle
	// floor (5%/s -> 1.6s to default 8% MPC_MANTHR_MIN), faster ramp on the
	// maximum (50%/s -> 2s to 100%), and 5%/s on hover thrust used by the curve.
	_manual_throttle_minimum.setSlewRate(0.05f);
	_manual_throttle_maximum.setSlewRate(0.5f);
	_hover_thrust_slew_rate.setSlewRate(0.05f);

	parameters_updated();
}

CustomAttitudeControl::~CustomAttitudeControl()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool CustomAttitudeControl::init()
{
	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("vehicle_attitude callback registration failed");
		return false;
	}

	return true;
}

void CustomAttitudeControl::parameters_updated()
{
	// Roll/Pitch/Yaw P gains. The yaw component is pre-divided by the yaw weight
	// so that scaling the delta-yaw quaternion by _yaw_w cancels out and the
	// effective yaw gain stays equal to MC_YAW_P (see AttitudeControl::setProportionalGain).
	_proportional_gain = Vector3f(_param_mc_roll_p.get(), _param_mc_pitch_p.get(), _param_mc_yaw_p.get());
	_yaw_w = math::constrain(_param_mc_yaw_weight.get(), 0.f, 1.f);

	if (_yaw_w > 1e-4f) {
		_proportional_gain(2) /= _yaw_w;
	}

	_rate_limit = Vector3f(math::radians(_param_mc_rollrate_max.get()),
			       math::radians(_param_mc_pitchrate_max.get()),
			       math::radians(_param_mc_yawrate_max.get()));

	_man_tilt_max = math::radians(_param_mpc_man_tilt_max.get());

	// If no HTE estimate has been received yet, anchor the hover-thrust slew to the parameter
	// so the throttle curve has a sensible center until the estimator comes up.
	if (!PX4_ISFINITE(_hover_thrust_estimate)) {
		_hover_thrust_slew_rate.setForcedValue(_param_mpc_thr_hover.get());
	}
}

float CustomAttitudeControl::throttle_curve(float throttle_stick_input)
{
	float thrust = 0.f;

	switch (_param_mpc_thr_curve.get()) {
	case 1: // No re-scaling: linear from manual-throttle min to MPC_THR_MAX.
		thrust = math::interpolate(throttle_stick_input, -1.f, 1.f,
					   _manual_throttle_minimum.getState(), _param_mpc_thr_max.get());
		break;

	case 2: // Rescale around MPC_THR_HOVER at zero stick.
		thrust = math::interpolateNXY(throttle_stick_input,
		{-1.f, 0.f, 1.f},
		{_manual_throttle_minimum.getState(), _param_mpc_thr_hover.get(), _param_mpc_thr_max.get()});
		break;

	default: // 0 / fallback: rescale around the slewed hover-thrust-estimate value.
		thrust = math::interpolateNXY(throttle_stick_input,
		{-1.f, 0.f, 1.f},
		{_manual_throttle_minimum.getState(), _hover_thrust_slew_rate.getState(), _param_mpc_thr_max.get()});
		break;
	}

	return math::min(thrust, _manual_throttle_maximum.getState());
}

void CustomAttitudeControl::generate_attitude_setpoint(const Quatf &q, float dt)
{
	vehicle_attitude_setpoint_s attitude_setpoint{};

	// During the arming gesture (full down throttle stick) keep yaw unlocked so we don't
	// accumulate absolute yaw error while the user holds the stick. AIRMODE==2 keeps the
	// yaw locked because that mode wants yaw authority all the way down.
	const bool arming_gesture = (_manual_control_setpoint.throttle < -.9f) && (_param_mc_airmode.get() != 2);

	if (arming_gesture) {
		_yaw_setpoint_stabilized = NAN;
	}

	const float yaw = Eulerf(q).psi();
	const float yaw_stick_input = math::expo_deadzone(_manual_control_setpoint.yaw, .6f, _param_man_deadzone.get());

	// =================================================================
	// Inlined StickYaw::generateYawSetpoint
	// =================================================================
	const float yaw_correction_prev = _yaw_correction;
	bool reset_yaw_setpoint = false;

	if (PX4_ISFINITE(_unaided_heading)) {
		// Detect the convergence phase of the yaw estimate by comparing the aided
		// heading with an unaided source. While converging the locked setpoint
		// would drift; mark it for reset when convergence ends.
		const float yaw_error = wrap_pi(yaw - _unaided_heading);
		const float yaw_error_hpf = wrap_pi(yaw_error - _yaw_error_lpf.getState());
		_yaw_error_lpf.update(yaw_error, dt);

		const bool was_converging = _yaw_estimate_converging;
		_yaw_estimate_converging = fabsf(yaw_error_hpf) > math::radians(1.f);

		if (!_yaw_estimate_converging) {
			_yaw_error_ref = yaw_error;

			if (was_converging) {
				reset_yaw_setpoint = true;
			}
		}

		_yaw_correction = wrap_pi(yaw_error - _yaw_error_ref);
	}

	if (reset_yaw_setpoint) {
		_yaw_setpoint_stabilized = NAN;
	}

	_yawspeed_filter.setParameters(dt, _param_mpc_man_y_tau.get());
	const float yawspeed_scale = math::radians(_param_mpc_man_y_max.get());
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_filter.update(yaw_stick_input * yawspeed_scale);

	// Yaw lock: if user is actively turning, unlock; if released, hold current yaw; if held,
	// advance the previously-locked yaw by the change in EKF-vs-unaided correction.
	if (fabsf(attitude_setpoint.yaw_sp_move_rate) > FLT_EPSILON) {
		_yaw_setpoint_stabilized = NAN;

	} else if (!PX4_ISFINITE(_yaw_setpoint_stabilized)) {
		_yaw_setpoint_stabilized = yaw;

	} else {
		_yaw_setpoint_stabilized = wrap_pi(_yaw_setpoint_stabilized - yaw_correction_prev + _yaw_correction);
	}

	// =================================================================
	// Stick -> roll/pitch tilt quaternion
	//
	// We control: tilt angle = |(roll, pitch)| and the planar direction.
	// This makes the configured maximum tilt a hard cap and keeps the
	// vehicle moving toward where the stick points.
	// =================================================================
	_man_roll_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());
	_man_pitch_input_filter.setParameters(dt, _param_mc_man_tilt_tau.get());

	Vector2f v = Vector2f(_man_roll_input_filter.update(_manual_control_setpoint.roll * _man_tilt_max),
			      -_man_pitch_input_filter.update(_manual_control_setpoint.pitch * _man_tilt_max));
	const float v_norm = v.norm();

	if (v_norm > _man_tilt_max) {
		v *= _man_tilt_max / v_norm;
	}

	const Quatf q_sp_rp = AxisAnglef(v(0), v(1), 0.f);

	// Build the yaw rotation from the (possibly unlocked) yaw setpoint.
	const float yaw_setpoint = PX4_ISFINITE(_yaw_setpoint_stabilized) ? _yaw_setpoint_stabilized : yaw;
	const Quatf q_sp_yaw(cosf(yaw_setpoint / 2.f), 0.f, 0.f, sinf(yaw_setpoint / 2.f));

	// Compose: align desired tilt with desired yaw.
	const Quatf q_sp = q_sp_yaw * q_sp_rp;
	q_sp.copyTo(attitude_setpoint.q_d);

	// Stick-derived collective thrust into body frame.
	attitude_setpoint.thrust_body[2] = -throttle_curve(_manual_control_setpoint.throttle);

	attitude_setpoint.timestamp = hrt_absolute_time();
	_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);
}

void CustomAttitudeControl::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
		parameters_updated();
	}

	// Refresh hover thrust estimate ahead of the attitude callback so manual mode
	// throttle scaling has the most recent value when generate_attitude_setpoint runs.
	if (_hover_thrust_estimate_sub.updated()) {
		hover_thrust_estimate_s hte;

		if (_hover_thrust_estimate_sub.update(&hte)) {
			if (hte.valid) {
				_hover_thrust_estimate = math::constrain(hte.hover_thrust, .05f, .9f);

			} else {
				// Estimate was invalidated - drift back to the hover-thrust parameter.
				_hover_thrust_estimate = _param_mpc_thr_hover.get();
			}
		}
	}

	vehicle_attitude_s v_att;

	if (_vehicle_attitude_sub.update(&v_att)) {

		// Guard dt against gaps / first-cycle (matches mc_att_control)
		const hrt_abstime now = v_att.timestamp_sample;
		const float dt = math::constrain(((now - _last_run) * 1e-6f), 0.0002f, 0.02f);
		_last_run = now;

		// Current attitude quaternion - constructed directly from the message buffer (zero copy).
		const Quatf q{v_att.q};

		// Update cached topics used for mode gating & manual control.
		_manual_control_setpoint_sub.update(&_manual_control_setpoint);
		_vehicle_control_mode_sub.update(&_vehicle_control_mode);

		if (_vehicle_status_sub.updated()) {
			vehicle_status_s vehicle_status;

			if (_vehicle_status_sub.copy(&vehicle_status)) {
				const bool armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
				_spooled_up = armed
					      && hrt_elapsed_time(&vehicle_status.armed_time) > _param_com_spoolup_time.get() * 1_s;
			}
		}

		if (_vehicle_land_detected_sub.updated()) {
			vehicle_land_detected_s vehicle_land_detected;

			if (_vehicle_land_detected_sub.copy(&vehicle_land_detected)) {
				_landed = vehicle_land_detected.landed;
			}
		}

		if (_vehicle_local_position_sub.updated()) {
			vehicle_local_position_s vehicle_local_position;

			if (_vehicle_local_position_sub.copy(&vehicle_local_position)) {
				_unaided_heading = vehicle_local_position.unaided_heading;
			}
		}

		// In Manual / Stabilized (attitude enabled, no altitude/velocity/position), generate
		// the attitude setpoint locally from sticks. In any other mode an upstream module
		// (mc_pos_control / flight_mode_manager / offboard / etc.) publishes it.
		const bool manual_attitude_only = _vehicle_control_mode.flag_control_attitude_enabled
						  && _vehicle_control_mode.flag_control_manual_enabled
						  && !_vehicle_control_mode.flag_control_altitude_enabled
						  && !_vehicle_control_mode.flag_control_velocity_enabled
						  && !_vehicle_control_mode.flag_control_position_enabled;

		if (manual_attitude_only) {
			generate_attitude_setpoint(q, dt);

		} else {
			// Reset stick-related state so re-entering manual doesn't carry stale input.
			_man_roll_input_filter.reset(0.f);
			_man_pitch_input_filter.reset(0.f);
			_yaw_setpoint_stabilized = NAN;
		}

		// Latch a new attitude setpoint if available; otherwise keep using the previous one.
		if (_vehicle_attitude_setpoint_sub.updated()) {
			vehicle_attitude_setpoint_s att_sp;

			if (_vehicle_attitude_setpoint_sub.copy(&att_sp)
			    && (att_sp.timestamp > _last_attitude_setpoint)) {

				_attitude_setpoint_q = Quatf(att_sp.q_d);
				_attitude_setpoint_q.normalize();
				_yawspeed_setpoint = att_sp.yaw_sp_move_rate;
				_thrust_setpoint_body = Vector3f(att_sp.thrust_body);
				_last_attitude_setpoint = att_sp.timestamp;
			}
		}

		// Apply EKF yaw/quaternion reset to the stored setpoint to avoid setpoint glitches.
		if (_quat_reset_counter != v_att.quat_reset_counter) {
			const Quatf delta_q_reset(v_att.delta_q_reset);

			// Propagate the yaw reset through the locked yaw setpoint and the StickYaw state.
			const float delta_psi = Eulerf(delta_q_reset).psi();

			if (PX4_ISFINITE(_yaw_setpoint_stabilized)) {
				_yaw_setpoint_stabilized = wrap_pi(_yaw_setpoint_stabilized + delta_psi);
			}

			_yaw_error_lpf.reset(wrap_pi(_yaw_error_lpf.getState() + delta_psi));
			_yaw_error_ref = wrap_pi(_yaw_error_ref + delta_psi);

			if (v_att.timestamp > _last_attitude_setpoint) {
				_attitude_setpoint_q = delta_q_reset * _attitude_setpoint_q;
				_attitude_setpoint_q.normalize();
			}

			_quat_reset_counter = v_att.quat_reset_counter;
		}

		if (_vehicle_control_mode.flag_control_attitude_enabled) {

			// ==========================================================================
			// Quaternion attitude control law (inline replica of AttitudeControl::update)
			//
			// 1. Build a reduced desired attitude that aligns thrust direction first
			//    (roll/pitch priority).
			// 2. Extract the residual yaw rotation between reduced and full desired
			//    attitude.
			// 3. Scale that residual yaw by the yaw weight to deprioritize yaw vs
			//    roll/pitch and recombine.
			// 4. Compute body-frame rate setpoint from the error quaternion.
			// 5. Add yaw-rate feed-forward expressed in body frame.
			// 6. Apply per-axis rate limits.
			// ==========================================================================
			Quatf qd = _attitude_setpoint_q;

			// Step 1 - reduced desired attitude neglecting vehicle yaw.
			const Vector3f e_z   = q.dcm_z();
			const Vector3f e_z_d = qd.dcm_z();
			Quatf qd_red(e_z, e_z_d);

			if (fabsf(qd_red(1)) > (1.f - 1e-5f) || fabsf(qd_red(2)) > (1.f - 1e-5f)) {
				// Antipodal / degenerate case: thrust vector is (nearly) opposite.
				// Fall back to full desired attitude. This is safe & stable.
				qd_red = qd;

			} else {
				// Express the tilt error in the world frame and combine with current attitude.
				qd_red *= q;
			}

			// Step 2 - delta yaw quaternion is what remains after removing the tilt rotation.
			// By construction it has the form (cos(angle/2), 0, 0, sin(angle/2)).
			Quatf qd_dyaw = qd_red.inversed() * qd;
			qd_dyaw.canonicalize();
			qd_dyaw(0) = math::constrain(qd_dyaw(0), -1.f, 1.f);
			qd_dyaw(3) = math::constrain(qd_dyaw(3), -1.f, 1.f);

			// Step 3 - scale the residual yaw rotation by the yaw weight and recombine.
			qd = qd_red * Quatf(cosf(_yaw_w * acosf(qd_dyaw(0))),
					    0.f, 0.f,
					    sinf(_yaw_w * asinf(qd_dyaw(3))));

			// Step 4 - body-frame rate setpoint from the error quaternion (axis-angle scaled).
			const Quatf    qe = q.inversed() * qd;
			const Vector3f eq = 2.f * qe.canonical().imag();

			Vector3f rate_setpoint = eq.emult(_proportional_gain);

			// Step 5 - yaw-rate feed-forward: the world Z-axis expressed in the body frame
			// is the last column of R.transposed (== q.inversed().dcm_z()).
			if (PX4_ISFINITE(_yawspeed_setpoint)) {
				rate_setpoint += q.inversed().dcm_z() * _yawspeed_setpoint;
			}

			// Step 6 - per-axis rate limit.
			for (int i = 0; i < 3; i++) {
				rate_setpoint(i) = math::constrain(rate_setpoint(i), -_rate_limit(i), _rate_limit(i));
			}

			// Inject autotune rate setpoint contribution if currently running.
			const hrt_abstime t = hrt_absolute_time();
			autotune_attitude_control_status_s pid_autotune;

			if (_autotune_attitude_control_status_sub.copy(&pid_autotune)) {
				if ((pid_autotune.state == autotune_attitude_control_status_s::STATE_ROLL
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_PITCH
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_YAW
				     || pid_autotune.state == autotune_attitude_control_status_s::STATE_TEST)
				    && ((t - pid_autotune.timestamp) < 1_s)) {
					rate_setpoint += Vector3f(pid_autotune.rate_sp);
				}
			}

			// Build & publish the rate setpoint atomically. Use copyTo for zero-copy of
			// the thrust body vector (no intermediate Vector3f allocations).
			vehicle_rates_setpoint_s rates_sp{};
			rates_sp.roll  = rate_setpoint(0);
			rates_sp.pitch = rate_setpoint(1);
			rates_sp.yaw   = rate_setpoint(2);
			_thrust_setpoint_body.copyTo(rates_sp.thrust_body);
			rates_sp.timestamp = hrt_absolute_time();

			_vehicle_rates_setpoint_pub.publish(rates_sp);
		}

		// Manual throttle envelope slews so the stick scale ramps smoothly across landed/
		// armed transitions. Done outside the attitude_enabled block so the floor / ceiling
		// keep tracking even when we're not actively running the controller.
		if (_landed) {
			_manual_throttle_minimum.update(0.f, dt);

		} else {
			_manual_throttle_minimum.update(_param_mpc_manthr_min.get(), dt);
		}

		if (_spooled_up) {
			_manual_throttle_maximum.update(1.f, dt);

		} else {
			_manual_throttle_maximum.setForcedValue(0.f);
		}

		// Slew the hover thrust used by the throttle curve toward the latest valid HTE.
		if (PX4_ISFINITE(_hover_thrust_estimate)) {
			_hover_thrust_slew_rate.update(_hover_thrust_estimate, dt);
		}
	}

	perf_end(_loop_perf);
}

int CustomAttitudeControl::task_spawn(int argc, char *argv[])
{
	CustomAttitudeControl *instance = new CustomAttitudeControl();

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

int CustomAttitudeControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CustomAttitudeControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Flat single-file multicopter attitude controller, functionally equivalent to
mc_att_control (without VTOL handling). Implements:

  - Quaternion-based reduced attitude control (Brescianini 2013):
    tilt-priority reduced setpoint, yaw deprioritization via the yaw weight,
    body-frame yaw-rate feed-forward and per-axis rate limit.
  - Manual / Stabilized stick -> attitude_setpoint generation:
    tilt-vector stick mapping, throttle curve (3 modes via MPC_THR_CURVE),
    inline StickYaw with yaw-lock + EKF reset propagation, manual throttle
    min/max slewing for landed/spoolup and hover-thrust slewing for the curve.

Input  : vehicle_attitude, vehicle_attitude_setpoint, vehicle_control_mode,
         manual_control_setpoint, vehicle_status, vehicle_land_detected,
         vehicle_local_position, hover_thrust_estimate,
         autotune_attitude_control_status
Output : vehicle_rates_setpoint, vehicle_attitude_setpoint (in manual modes)

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("custom_mc_att_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int custom_mc_att_control_main(int argc, char *argv[])
{
	return ModuleBase::main(CustomAttitudeControl::desc, argc, argv);
}
