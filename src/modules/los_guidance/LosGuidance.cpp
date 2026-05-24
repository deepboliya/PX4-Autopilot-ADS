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

using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

ModuleBase::Descriptor LosGuidance::desc{task_spawn, custom_command, print_usage};

LosGuidance::LosGuidance() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	parameters_updated();
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


	const float tan_a = tanf(alpha);
	const float tan_b = tanf(beta);

	// Target unit vector in the camera/gimbal frame (FRD-aligned with the
	// gimbal):
	//   X_g = forward, Y_g = right, Z_g = down.
	// Yaw alpha rotates the forward axis toward +Y_g (right); pitch beta
	// rotates the forward axis toward -Z_g (up).
	const float denom = sqrt(1 + tan_a*tan_a + tan_b*tan_b);
	const Vector3f los_gimbal{1/denom, tan_a/denom, tan_b/denom};

	// Gimbal → body: Eulerf(phi, theta, psi) = (roll, pitch, yaw); roll fixed
	// at 0. Pitch is LOS_GD_GIMB_PIT; yaw offset is LOS_GD_GIMB_YAW.
	const Quatf q_body_from_gimbal(
		Eulerf(0.f, _param_los_gd_gimb_pit.get(), _param_los_gd_gimb_yaw.get()));
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
	sp.yawspeed = NAN;
	_trajectory_setpoint_pub.publish(sp);

	_last_publish_timestamp = now;
	++_setpoints_published;
}

void LosGuidance::publish_hold_setpoint()
{
	const hrt_abstime now = hrt_absolute_time();

	// Position-only mode: position is authoritative, velocity travels as
	// feedforward through trajectory_setpoint. Do NOT flip ocm.velocity
	// here or commander will treat this as a velocity command.
	offboard_control_mode_s ocm{};
	ocm.timestamp = now;
	ocm.position = true;
	ocm.velocity = false;
	ocm.acceleration = false;
	ocm.attitude = false;
	ocm.body_rate = false;
	ocm.thrust_and_torque = false;
	ocm.direct_actuator = false;
	_offboard_control_mode_pub.publish(ocm);

	trajectory_setpoint_s sp{};
	sp.timestamp = now;
	sp.position[0] = _latched_position_ned(0);
	sp.position[1] = _latched_position_ned(1);
	sp.position[2] = _latched_position_ned(2);
	sp.velocity[0] = 0.f;
	sp.velocity[1] = 0.f;
	sp.velocity[2] = 0.f;
	sp.acceleration[0] = NAN;
	sp.acceleration[1] = NAN;
	sp.acceleration[2] = NAN;
	sp.jerk[0] = NAN;
	sp.jerk[1] = NAN;
	sp.jerk[2] = NAN;
	sp.yaw = NAN;
	sp.yawspeed = NAN;
	_trajectory_setpoint_pub.publish(sp);

	_last_publish_timestamp = now;
	++_hold_setpoints_published;
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
	bool got_fresh_sample = false;

	while (_los_measurements_sub.updated()) {
		if (_los_measurements_sub.copy(&los)) {
			_latest_sample = los;
			_has_sample = true;
			_last_sample_timestamp = los.timestamp;
			++_samples_received;
			got_fresh_sample = true;
		}
	}

	// Pull the latest attitude (always cheap, single sample).
	vehicle_attitude_s att;

	if (_vehicle_attitude_sub.update(&att)) {
		_q_attitude = Quatf(att.q);
		_has_attitude = true;
	}

	// Latch local position at the moment of the most recent confirmed-good
	// bearing — this is what the fallback hold setpoint anchors to. Only
	// updating on fresh samples keeps the latch from drifting during a
	// stale-bearing window.
	if (got_fresh_sample) {
		vehicle_local_position_s lpos;

		if (_vehicle_local_position_sub.copy(&lpos) && lpos.xy_valid && lpos.z_valid) {
			_latched_position_ned(0) = lpos.x;
			_latched_position_ned(1) = lpos.y;
			_latched_position_ned(2) = lpos.z;
			_has_latched_position = true;
		}
	}

	if (!_param_los_gd_en.get()) {
		++_early_return_disabled;
		return;
	}

	const hrt_abstime now = hrt_absolute_time();
	const hrt_abstime timeout_us =
		static_cast<hrt_abstime>(_param_los_gd_timeout_ms.get()) * 1000ULL;

	if (!_has_sample || (now - _last_sample_timestamp) > timeout_us) {
		++_early_return_no_fresh_los;

		// Soft fallback: hold the last known position instead of
		// dropping to commander's OFFBOARD failsafe on transient
		// CV dropouts. If we never latched a position (cold start,
		// no LOS yet), publish nothing and let commander handle it.
		if (_has_latched_position) {
			publish_hold_setpoint();
		}

		return;
	}

	Vector3f acceleration_ned;

	if (!compute_acceleration_command_ned(acceleration_ned)) {
		++_accel_compute_failures;
		return;
	}

	publish_offboard_setpoint(acceleration_ned);
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

	PX4_INFO("rx=%" PRIu64 " tx=%" PRIu64 " hold_tx=%" PRIu64
		 " accel_compute_fail=%" PRIu64
		 " early_disabled=%" PRIu64 " early_no_fresh_los=%" PRIu64
		 " has_sample=%d has_attitude=%d has_latched_pos=%d",
		 _samples_received, _setpoints_published, _hold_setpoints_published,
		 _accel_compute_failures,
		 _early_return_disabled, _early_return_no_fresh_los,
		 (int)_has_sample, (int)_has_attitude, (int)_has_latched_position);

	if (_has_latched_position) {
		PX4_INFO("latched NED x=%.2f y=%.2f z=%.2f",
			 (double)_latched_position_ned(0),
			 (double)_latched_position_ned(1),
			 (double)_latched_position_ned(2));
	}

	if (_has_sample) {
		PX4_INFO("last bearing a=%.3f b=%.3f (ar=%.3f br=%.3f) age=%" PRIu64 " us",
			 (double)_latest_sample.alpha,
			 (double)_latest_sample.beta,
			 (double)_latest_sample.alpha_rate,
			 (double)_latest_sample.beta_rate,
			 (uint64_t)(hrt_absolute_time() - _last_sample_timestamp));
	}

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
module switches to a position-hold setpoint anchored at the vehicle's
local position at the moment of the most recent fresh bearing. Hold
continues indefinitely until fresh bearings resume or the operator
intervenes. If no bearing has ever been received, the module publishes
nothing so commander handles the situation through its usual
OFFBOARD-timeout path.

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
