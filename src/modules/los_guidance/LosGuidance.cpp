/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "LosGuidance.hpp"

#include <inttypes.h>
#include <math.h>

#include <mathlib/mathlib.h>

using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

ModuleBase::Descriptor LosGuidance::desc{task_spawn, custom_command, print_usage};

LosGuidance::LosGuidance() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	parameters_updated();
	// Seat the camera-pitch latch at the fixed gimbal pitch param so the
	// LOS-to-body transform stays consistent before the first tracking
	// tick runs (and so the servo idles at a sane neutral).
	_camera_pitch_cmd = _param_los_gd_gimb_pit.get();
}

bool LosGuidance::init()
{
	if (!_los_measurements_sub.registerCallback()) {
		PX4_ERR("los_measurements callback registration failed");
		return false;
	}

	// Drive the publisher at the configured rate so the OFFBOARD heartbeat
	// keeps ticking even between bearing samples (commander treats
	// offboard_control_mode silence as a loss-of-link).
	ScheduleOnInterval(_schedule_interval_us);
	return true;
}

void LosGuidance::parameters_updated()
{
	updateParams();

	const int requested_hz = _param_los_gd_pub_hz.get();
	const int clamped_hz = (requested_hz < 1) ? 1 : ((requested_hz > 1000) ? 1000 : requested_hz);
	const hrt_abstime new_interval_us = 1'000'000 / static_cast<hrt_abstime>(clamped_hz);

	if (new_interval_us != _schedule_interval_us) {
		_schedule_interval_us = new_interval_us;
		// ScheduleClear()+ScheduleOnInterval() is idempotent and cheap;
		// only call when the rate actually changed.
		ScheduleClear();
		ScheduleOnInterval(_schedule_interval_us);
	}
}

bool LosGuidance::compute_acceleration_command_ned(Vector3f &acceleration_ned) const
{
	if (!_has_sample || !_has_attitude) {
		return false;
	}

	const float alpha = _latest_sample.alpha;   // yaw bearing  [rad], +right
	const float beta  = _latest_sample.beta;    // pitch bearing [rad], +up

	const float cos_a = cosf(alpha);
	const float sin_a = sinf(alpha);
	const float cos_b = cosf(beta);
	const float sin_b = sinf(beta);

	// Target unit vector in the camera/gimbal frame (FRD-aligned with the
	// gimbal):
	//   X_g = forward, Y_g = right, Z_g = down.
	// Yaw alpha rotates the forward axis toward +Y_g (right); pitch beta
	// rotates the forward axis toward -Z_g (up).
	const Vector3f los_gimbal{cos_a * cos_b, sin_a * cos_b, -sin_b};

	// Gimbal → body: Eulerf(phi, theta, psi) = (roll, pitch, yaw); roll fixed
	// at 0. Pitch uses the live tracking-controller output `_camera_pitch_cmd`
	// (the servo is assumed to follow the command faithfully within one tick);
	// yaw offset is the static gimbal mount offset LOS_GD_GIMB_YAW.
	const Quatf q_body_from_gimbal(
		Eulerf(0.f, _camera_pitch_cmd, _param_los_gd_gimb_yaw.get()));
	const Vector3f los_body = q_body_from_gimbal.rotateVector(los_gimbal);

	// vehicle_attitude.q rotates body-frame vectors to NED, exactly what we
	// need to lift the bearing into the inertial frame for the position
	// controller.
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

void LosGuidance::publish_offboard_setpoint(const Vector3f &acceleration_ned)
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
	// Feedforward yaw rate from the alpha PI controller. mc_pos_control
	// passes this through to the attitude/rate stack as a yawspeed setpoint
	// while sp.yaw=NAN (free heading). NOTE for the operator: make sure
	// MPC_MAN_Y_MAX / MC_YAWRATE_MAX are at least as large as LOS_GD_YR_MAX,
	// otherwise the rate setpoint will be clipped by the lower stack before
	// it reaches the motors.
	sp.yawspeed = _yaw_rate_cmd;
	_trajectory_setpoint_pub.publish(sp);

	_last_publish_timestamp = now;
	++_setpoints_published;
}

void LosGuidance::update_tracking_controllers(float dt)
{
	if (!_has_sample || dt <= 0.f || !PX4_ISFINITE(dt)) {
		return;
	}

	const float alpha = _latest_sample.alpha;   // +right
	const float beta  = _latest_sample.beta;    // +up

	// ---- PI on alpha → vehicle yaw rate ----------------------------------
	// Error convention: setpoint=0 (target on optical axis), error = alpha.
	// alpha>0 means target to the right; we want positive yawspeed in FRD
	// (yaw right) so both Kp and Ki signs are positive here.
	_alpha_integral += alpha * dt;
	const float yaw_im = fabsf(_param_los_gd_yaw_im.get());
	_alpha_integral = math::constrain(_alpha_integral, -yaw_im, yaw_im);

	const float yawrate_raw = _param_los_gd_yaw_kp.get() * alpha
				+ _param_los_gd_yaw_ki.get() * _alpha_integral;
	const float yr_max = fabsf(_param_los_gd_yr_max.get());
	_yaw_rate_cmd = math::constrain(yawrate_raw, -yr_max, yr_max);

	// Conditional integration: if the output is saturated AND the current
	// error would push it further into saturation, peel back the most recent
	// integral step to avoid wind-up.
	if (fabsf(yawrate_raw) > yr_max && (alpha * yawrate_raw) > 0.f) {
		_alpha_integral -= alpha * dt;
	}

	// ---- I-only on beta → camera-servo pitch -----------------------------
	// beta>0 means target above optical axis; tilting the camera UP in FRD
	// requires DECREASING pitch (LOS_GD_GIMB_PIT is +down). Hence the minus
	// sign on the integral term — with a positive LOS_GD_PIT_KI gain.
	_beta_integral += beta * dt;
	const float pit_im = fabsf(_param_los_gd_pit_im.get());
	_beta_integral = math::constrain(_beta_integral, -pit_im, pit_im);

	const float pitch_min = _param_los_gd_pit_min.get();
	const float pitch_max = _param_los_gd_pit_max.get();
	const float pitch_raw = _param_los_gd_gimb_pit.get()
			      - _param_los_gd_pit_ki.get() * _beta_integral;
	const float pitch_cmd = math::constrain(pitch_raw, pitch_min, pitch_max);

	// Anti-windup mirror of the yaw branch: if we just clipped the pitch
	// command, undo the last integral step so it can't keep growing.
	if (pitch_raw < pitch_min || pitch_raw > pitch_max) {
		_beta_integral -= beta * dt;
	}

	_camera_pitch_cmd = pitch_cmd;
}

void LosGuidance::publish_camera_pitch()
{
	// We publish on `gimbal_controls` and let mixer_module's FunctionGimbal
	// route it to whatever PWM channel is mapped to Gimbal_Pitch (output
	// function 421).
	//
	// REQUIRED PX4 configuration on this airframe (Pixhawk 6C, FMU MAIN 1..4
	// = motors, MAIN 5 = camera servo signal):
	//
	//   PWM_MAIN_FUNC1..4 = Motor 1..4         (function 101..104, default for quad)
	//   PWM_MAIN_FUNC5    = Gimbal_Pitch       (function 421)
	//   PWM_MAIN_MIN5  / PWM_MAIN_MAX5 / PWM_MAIN_DIS5
	//                     = servo's mechanical PWM range / disarmed value
	//   MNT_MODE_OUT      = Disabled           (so the gimbal module does not
	//                                           also publish gimbal_controls and
	//                                           fight this module on the topic)
	//
	// We send NaN on roll/yaw so any channel mistakenly mapped to those
	// gimbal axes is held in the "disarmed" state by FunctionGimbal.
	gimbal_controls_s msg{};
	msg.timestamp = hrt_absolute_time();

	const float half_range = fabsf(_param_los_gd_pit_hrg.get());
	const float center = _param_los_gd_gimb_pit.get();
	float normalized = NAN;

	if (half_range > 1e-4f && PX4_ISFINITE(_camera_pitch_cmd)) {
		normalized = (_camera_pitch_cmd - center) / half_range;
		normalized = math::constrain(normalized, -1.f, 1.f);
	}

	msg.control[gimbal_controls_s::INDEX_ROLL]  = NAN;
	msg.control[gimbal_controls_s::INDEX_PITCH] = normalized;
	msg.control[gimbal_controls_s::INDEX_YAW]   = NAN;
	_gimbal_controls_pub.publish(msg);
}

void LosGuidance::reset_tracking_controllers()
{
	_alpha_integral = 0.f;
	_beta_integral = 0.f;
	_yaw_rate_cmd = 0.f;
	_camera_pitch_cmd = _param_los_gd_gimb_pit.get();
	_last_control_timestamp = 0;
}

void LosGuidance::Run()
{
	if (should_exit()) {
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

	// Drain bearing samples (zero-copy via copy() into the latched
	// _latest_sample), keeping only the freshest one for this cycle.
	los_measurements_s los;

	while (_los_measurements_sub.updated()) {
		if (_los_measurements_sub.copy(&los)) {
			_latest_sample = los;
			_has_sample = true;
			_last_sample_timestamp = los.timestamp;
			++_samples_received;
		}
	}

	// Pull the latest attitude (always cheap, single sample).
	vehicle_attitude_s att;

	if (_vehicle_attitude_sub.update(&att)) {
		_q_attitude = Quatf(att.q);
		_has_attitude = true;
	}

	if (!_param_los_gd_en.get()) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime timeout_us =
		static_cast<hrt_abstime>(_param_los_gd_timeout_ms.get()) * 1000ULL;

	if (!_has_sample || (now - _last_sample_timestamp) > timeout_us) {
		// Bearings stale: stop publishing OFFBOARD so commander can run
		// its timeout failsafe, AND zero the tracking controllers so the
		// integrators don't accumulate while we're blind.
		reset_tracking_controllers();
		return;
	}

	// dt for the integrators is the wall time since the last control tick,
	// not since the last sample — this way the schedule rate (LOS_GD_PUB_HZ)
	// determines integration cadence even if samples arrive faster/slower.
	float dt = 0.f;

	if (_last_control_timestamp != 0) {
		dt = static_cast<float>(now - _last_control_timestamp) * 1e-6f;
	}

	_last_control_timestamp = now;

	update_tracking_controllers(dt);

	Vector3f acceleration_ned;

	if (!compute_acceleration_command_ned(acceleration_ned)) {
		return;
	}

	publish_offboard_setpoint(acceleration_ned);
	publish_camera_pitch();
}

int LosGuidance::task_spawn(int argc, char *argv[])
{
	LosGuidance *instance = new LosGuidance();

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

int LosGuidance::print_status()
{
	PX4_INFO("enabled=%d acc_max=%.2f gimbal_pitch=%.3f rad gimbal_yaw=%.3f rad rate=%d Hz",
		 (int)_param_los_gd_en.get(),
		 (double)_param_los_gd_acc_max.get(),
		 (double)_param_los_gd_gimb_pit.get(),
		 (double)_param_los_gd_gimb_yaw.get(),
		 (int)_param_los_gd_pub_hz.get());

	PX4_INFO("rx=%" PRIu64 " tx=%" PRIu64 " has_sample=%d has_attitude=%d",
		 _samples_received, _setpoints_published,
		 (int)_has_sample, (int)_has_attitude);

	if (_has_sample) {
		PX4_INFO("last bearing a=%.3f b=%.3f (ar=%.3f br=%.3f) age=%" PRIu64 " us",
			 (double)_latest_sample.alpha,
			 (double)_latest_sample.beta,
			 (double)_latest_sample.alpha_rate,
			 (double)_latest_sample.beta_rate,
			 (uint64_t)(hrt_absolute_time() - _last_sample_timestamp));
	}

	PX4_INFO("track: yawrate_cmd=%.3f rad/s pitch_cmd=%.3f rad alpha_i=%.3f beta_i=%.3f",
		 (double)_yaw_rate_cmd,
		 (double)_camera_pitch_cmd,
		 (double)_alpha_integral,
		 (double)_beta_integral);

	return 0;
}

int LosGuidance::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int LosGuidance::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
`los_guidance` converts body-frame Line-of-Sight bearings from a
gimbal-mounted camera into a NED-frame acceleration setpoint and feeds
the result into PX4's existing OFFBOARD pipeline.

For now the gimbal orientation relative to the body is fixed by
`LOS_GD_GIMB_PIT` (pitch about body Y) and `LOS_GD_GIMB_YAW` (yaw about
body Z). A future gimbal controller can drive these dynamically.

Bearing frame chain (FRD throughout):

  camera/gimbal frame --Euler(0, LOS_GD_GIMB_PIT, LOS_GD_GIMB_YAW)--> body
                                                         --q_attitude--> NED

The published acceleration vector has constant magnitude `LOS_GD_ACC_MAX`
and points along the NED-frame LOS unit vector. mc_pos_control consumes
the `trajectory_setpoint` exactly as it would any external OFFBOARD
acceleration command; the operator still has to switch the vehicle to
OFFBOARD via RC/QGC/MAVLink.

When the bearing stream goes stale (older than `LOS_GD_TIMEOUT`), the
module stops publishing so commander can engage its OFFBOARD timeout
failsafe.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("los_guidance", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int los_guidance_main(int argc, char *argv[])
{
	return ModuleBase::main(LosGuidance::desc, argc, argv);
}
