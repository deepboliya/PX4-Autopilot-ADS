/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "HoldRigAtt.hpp"

#include <inttypes.h>
#include <math.h>

#include <mathlib/math/Limits.hpp>
#include <parameters/param.h>

using matrix::AxisAnglef;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

using hold_rig_att::angle_between;
using hold_rig_att::decompose;
using hold_rig_att::kArrivedRad;
using hold_rig_att::ramp_step;
using hold_rig_att::TargetMode;
using hold_rig_att::tilt_of;

ModuleBase::Descriptor HoldRigAtt::desc{task_spawn, custom_command, print_usage};

HoldRigAtt::HoldRigAtt() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	parameters_updated();
}

bool HoldRigAtt::init()
{
	ScheduleOnInterval(_schedule_interval_us);
	return true;
}

Quatf HoldRigAtt::target_relative() const
{
	return hold_rig_att::target_relative(static_cast<TargetMode>(_param_hratt_mode.get()),
					     _param_hratt_tilt.get(),
					     _param_hratt_dir.get(),
					     _param_hratt_roll.get(),
					     _param_hratt_pitch.get());
}

void HoldRigAtt::build_target()
{
	// Exact for intrinsic Z-Y-X in both modes: the latched yaw commutes out
	// to the left, and target_relative() is a pure body-frame rotation
	// applied on the right, so the tilt direction stays relative to the nose
	// rather than to north.
	_q_target = Quatf(Eulerf(0.f, 0.f, _hold_yaw)) * target_relative();
	_q_target.normalize();
}

bool HoldRigAtt::past_arm_lockdown(hrt_abstime now) const
{
	if (!_param_hratt_arm_gate.get()) {
		return true;
	}

	if (_armed_time == 0) {
		// Not armed (or no vehicle_status yet). Nothing to wait for: the
		// undeferrable-disarm window is measured from arming, and if we are
		// not armed there is no disarm to dodge.
		return true;
	}

	float lockdown_s = 3.f;
	float spoolup_s = 0.f;
	param_t p = param_find("COM_LKDOWN_TKO");

	if (p != PARAM_INVALID) {
		param_get(p, &lockdown_s);
	}

	p = param_find("COM_SPOOLUP_TIME");

	if (p != PARAM_INVALID) {
		param_get(p, &spoolup_s);
	}

	// Small margin on top: the failure detector's own hysteresis
	// (FD_FAIL_R_TTRI, 0.3 s by default) means the trip lands slightly after
	// the tilt does, and the ramp needs to have not yet reached 60 degrees
	// when the window closes rather than merely to have started.
	const float window_s = lockdown_s + spoolup_s + 0.5f;

	return (static_cast<float>(now - _armed_time) * 1e-6f) > window_s;
}

void HoldRigAtt::parameters_updated()
{
	updateParams();

	const int requested_hz = _param_hratt_pub_hz.get();
	const int clamped_hz = (requested_hz < 1) ? 1 : ((requested_hz > 1000) ? 1000 : requested_hz);
	const hrt_abstime new_interval_us = 1'000'000 / static_cast<hrt_abstime>(clamped_hz);

	if (new_interval_us != _schedule_interval_us) {
		_schedule_interval_us = new_interval_us;
		ScheduleClear();
		ScheduleOnInterval(_schedule_interval_us);
	}

	check_rate_against_limits();
	check_ramp_time();
	check_euler_degeneracy();
	check_failure_detector();
	check_rig_disarm_params();
	check_thrust_against_tilt();
	check_sibling_modules();
}

void HoldRigAtt::check_rate_against_limits() const
{
	// HRATT_RATE only drives the leader in ramp_toward_target(). What the
	// vehicle actually gets commanded is AttitudeControl::update()'s output,
	// and that is bounded twice over, per axis. Unlike flip_rig_axis, the
	// consequence of exceeding a bound here is not a stalled manoeuvre - the
	// target is static, so the vehicle always eventually arrives. It is that
	// the ramp stops being a ramp: the leader outruns the vehicle, the
	// tracking error grows until the rate demand saturates, and the approach
	// degenerates into exactly the full-authority slam the ramp exists to
	// avoid.
	//
	// The tilt direction splits the commanded rate between the two axes, so
	// each is checked against its own limits: leaning away from pure pitch
	// relieves the pitch ceilings and loads the roll ceilings in exchange.
	const float rate = _param_hratt_rate.get();
	const Quatf q_rel = target_relative();

	float tilt_deg = 0.f;
	float dir_deg = 0.f;
	float dyaw_deg = 0.f;
	decompose(q_rel, tilt_deg, dir_deg, dyaw_deg);

	const float dir = math::radians(dir_deg);

	check_axis_share("roll", rate * fabsf(cosf(dir)), "MC_ROLLRATE_MAX", "MC_ROLL_P");
	check_axis_share("pitch", rate * fabsf(sinf(dir)), "MC_PITCHRATE_MAX", "MC_PITCH_P");
}

void HoldRigAtt::check_axis_share(const char *axis_label, float axis_rate_deg,
				  const char *rate_max_param, const char *att_p_param) const
{
	if (axis_rate_deg < 1e-3f) {
		// The tilt direction puts nothing on this axis, so its limits are
		// irrelevant.
		return;
	}

	// Ceiling 1: the hard clamp on the attitude controller's output
	// (AttitudeControl::update(), reference Eq. 54). Warn with 20% margin,
	// because merely matching the limit leaves the vehicle no headroom to
	// close the tracking lag - only to hold station behind the leader.
	param_t p = param_find(rate_max_param);
	float rate_max_deg = 0.f;

	// Keep every message inside the 127-char cap of log_message_s.text, or
	// the tail - which is where the actionable part lives - is silently
	// chopped off. Detail belongs in the comments and the parameter docs.
	if ((p != PARAM_INVALID) && (param_get(p, &rate_max_deg) == PX4_OK) && (rate_max_deg > 0.f) &&
	    (axis_rate_deg > 0.8f * rate_max_deg)) {
		PX4_WARN("HRATT_RATE puts %.0f deg/s on %s, >80%% of %s (%.0f) - ramp degrades to a step.",
			 (double)axis_rate_deg, axis_label, rate_max_param, (double)rate_max_deg);
	}

	// Ceiling 2: the attitude P-law itself. The error vector is
	// eq = 2*sin(phi/2)*n_hat, saturating at magnitude 2, so the loop can
	// never command more than 2*MC_*_P rad/s however far behind the leader
	// gets - independently of the clamp above. A low attitude gain can
	// therefore defeat the ramp on its own.
	p = param_find(att_p_param);
	float att_p = 0.f;

	if ((p != PARAM_INVALID) && (param_get(p, &att_p) == PX4_OK) && (att_p > 0.f)) {
		const float p_ceiling_deg = math::degrees(2.f * att_p);

		if (axis_rate_deg > 0.8f * p_ceiling_deg) {
			PX4_WARN("HRATT_RATE puts %.0f deg/s on %s, >80%% of 2*%s ceiling (%.0f) - cannot keep up.",
				 (double)axis_rate_deg, axis_label, att_p_param, (double)p_ceiling_deg);
		}

		// The steady-state tracking lag while the ramp runs, from
		// rate = 2*sin(lag/2)*P inverted. This is the number that decides
		// whether the ramp is doing its job: a small lag means the
		// controller is nowhere near saturation the whole way up.
		const float sin_half = math::radians(axis_rate_deg) / (2.f * att_p);

		if (sin_half < 1.f) {
			const float lag_deg = math::degrees(2.f * asinf(sin_half));

			if (lag_deg > 30.f) {
				PX4_WARN("Ramp lag on %s is %.0f deg at %.0f deg/s - lower HRATT_RATE or raise %s.",
					 axis_label, (double)lag_deg, (double)axis_rate_deg, att_p_param);
			}

		} else {
			PX4_WARN("HRATT_RATE exceeds the %s P-law ceiling outright - the leader will run away.",
				 axis_label);
		}
	}
}

void HoldRigAtt::check_ramp_time() const
{
	// The ramp ends on whichever fires first: the leader arriving, or the
	// HRATT_RAMP_T cutoff. Worst case the leader has to travel 180 degrees
	// (the shortest arc can never be longer), so that is what the cutoff has
	// to cover. Using the worst case rather than the actual distance is
	// deliberate: the distance depends on where the vehicle happens to be at
	// OFFBOARD entry, which is not knowable at parameter-check time.
	const float rate = _param_hratt_rate.get();

	if (rate < 1e-3f) {
		return;
	}

	const float travel_deg = math::degrees(tilt_of(target_relative()));
	const float needed_s = travel_deg / rate;
	const float ramp_t = _param_hratt_ramp_t.get();

	if (ramp_t <= needed_s) {
		// Benign, unlike the equivalent flip_rig_axis case: an early
		// cutoff holds wherever the leader reached instead of snapping
		// anywhere. Still worth saying, because the vehicle then parks
		// somewhere other than where it was told to.
		PX4_WARN("HRATT_RAMP_T %.1f s < %.1f s needed for %.0f deg at %.0f deg/s - will park short.",
			 (double)ramp_t, (double)needed_s, (double)travel_deg, (double)rate);
	}
}

void HoldRigAtt::check_euler_degeneracy() const
{
	const Quatf q_rel = target_relative();

	float tilt_deg = 0.f;
	float dir_deg = 0.f;
	float dyaw_deg = 0.f;
	decompose(q_rel, tilt_deg, dir_deg, dyaw_deg);

	if (_param_hratt_mode.get() != 1) {
		return;
	}

	// Always log the translation, degenerate or not: it is the one line that
	// lets a mode-1 command be checked against what the controller will
	// really do, and it is how the operator moves over to mode 0.
	PX4_INFO("HRATT_MODE=1 roll %.0f pitch %.0f == tilt %.0f deg, dir %.0f deg, plus %.0f deg delta-yaw.",
		 (double)_param_hratt_roll.get(), (double)_param_hratt_pitch.get(),
		 (double)tilt_deg, (double)dir_deg, (double)dyaw_deg);

	const float roll_abs = fabsf(_param_hratt_roll.get());
	const float pitch_abs = fabsf(_param_hratt_pitch.get());

	if ((roll_abs > 80.f) || (pitch_abs > 80.f)) {
		// Past 80 degrees on either axis the Euler pair has effectively
		// stopped controlling tilt in one direction. At exactly 90 it is
		// total: body z is [0, -1, 0] for every pitch value.
		PX4_WARN("HRATT_ROLL/PITCH near 90 deg is degenerate: pitch stops changing tilt. Use HRATT_MODE 0.");
	}

	if (fabsf(dyaw_deg) > 15.f) {
		// The delta-yaw share is not lost, but it is executed on the
		// weakest axis at MC_YAW_WEIGHT (0.4 default) of the error - so
		// this is the part of the command that will visibly fail to
		// arrive on the rig.
		PX4_WARN("%.0f deg of this command is delta-yaw, scaled by MC_YAW_WEIGHT on the weakest axis.",
			 (double)fabsf(dyaw_deg));
	}
}

void HoldRigAtt::check_failure_detector() const
{
	// FailureDetector::updateAttitudeStatus() compares EULER roll and pitch
	// against FD_FAIL_R/FD_FAIL_P, so those are the quantities to check -
	// not the tilt magnitude, which is a different number (a target of
	// tilt 90 / dir 45 is Euler roll 90, pitch 45). The check is gated on
	// flag_control_attitude_enabled, which is exactly what attitude-mode
	// OFFBOARD sets, so it is armed for this module's entire run.
	const Eulerf euler(target_relative());
	const float roll_deg = fabsf(math::degrees(euler.phi()));
	const float pitch_deg = fabsf(math::degrees(euler.theta()));

	float fail_r = 0.f;
	float fail_p = 0.f;
	int32_t fail_r_i = 0;
	int32_t fail_p_i = 0;
	param_t p = param_find("FD_FAIL_R");

	if ((p != PARAM_INVALID) && (param_get(p, &fail_r_i) == PX4_OK)) {
		fail_r = (float)fail_r_i;
	}

	p = param_find("FD_FAIL_P");

	if ((p != PARAM_INVALID) && (param_get(p, &fail_p_i) == PX4_OK)) {
		fail_p = (float)fail_p_i;
	}

	const bool trips_roll = (fail_r > 0.f) && (roll_deg > fail_r);
	const bool trips_pitch = (fail_p > 0.f) && (pitch_deg > fail_p);

	if (!trips_roll && !trips_pitch) {
		return;
	}

	PX4_ERR("Target Euler roll %.0f/pitch %.0f exceeds FD_FAIL_R %.0f/FD_FAIL_P %.0f - set both to 0.",
		(double)roll_deg, (double)pitch_deg, (double)fail_r, (double)fail_p);

	// Split rather than appended: the combined text would exceed the
	// 127-char log_message_s cap and lose exactly the actionable part.
	int32_t cbrk = 0;
	p = param_find("CBRK_FLIGHTTERM");

	if ((p != PARAM_INVALID) && (param_get(p, &cbrk) == PX4_OK) && (cbrk != 121212)) {
		PX4_ERR("CBRK_FLIGHTTERM is %" PRId32 ", not 121212: the trip will TERMINATE flight, not warn.",
			cbrk);
	}

	if (!_param_hratt_arm_gate.get()) {
		// The one regime no circuit breaker or deferral can soften. See
		// past_arm_lockdown() and the HRATT_ARM_GATE docs.
		PX4_ERR("HRATT_ARM_GATE is off: a trip within COM_LKDOWN_TKO of arming DISARMS undeferrably.");
	}
}

void HoldRigAtt::check_rig_disarm_params() const
{
	// A fixture that cannot translate never reports a takeoff, and with a
	// fixed thrust setpoint and no altitude loop the land detector's
	// low-thrust branch can latch "landed" outright. Both of those end in an
	// auto-disarm that has nothing to do with attitude.
	float disarm_prflt = 0.f;
	param_t p = param_find("COM_DISARM_PRFLT");

	if ((p != PARAM_INVALID) && (param_get(p, &disarm_prflt) == PX4_OK) && (disarm_prflt > 0.f)) {
		PX4_WARN("COM_DISARM_PRFLT is %.0f s: on a rig no takeoff is detected, so it disarms. Set 0.",
			 (double)disarm_prflt);
	}

	float disarm_land = 0.f;
	p = param_find("COM_DISARM_LAND");

	if ((p != PARAM_INVALID) && (param_get(p, &disarm_land) == PX4_OK) && (disarm_land > 0.f)) {
		PX4_WARN("COM_DISARM_LAND is %.0f s: the land detector can latch on a rig and disarm. Set 0.",
			 (double)disarm_land);
	}

	int32_t airmode = 0;
	p = param_find("MC_AIRMODE");

	if ((p != PARAM_INVALID) && (param_get(p, &airmode) == PX4_OK) && (airmode == 0)) {
		PX4_INFO("MC_AIRMODE is 0: saturating roll/pitch torque is paid for out of collective thrust.");
	}
}

void HoldRigAtt::check_thrust_against_tilt() const
{
	const float tilt_deg = math::degrees(tilt_of(target_relative()));

	if (tilt_deg <= 90.f) {
		return;
	}

	// Past 90 degrees the body z axis points upward, so a negative
	// thrust_body[2] - which is the "up" direction in body FRD - now
	// accelerates the airframe downward. There is no 1/cos(tilt)
	// compensation on the attitude path; the position controller normally
	// does that and is not in the loop here.
	const float thrust = _param_hratt_thrust.get();

	if (fabsf(thrust) > 0.3f) {
		PX4_WARN("Tilt %.0f deg is inverted and HRATT_THRUST is %.2f - thrust now pushes DOWN. Reduce it.",
			 (double)tilt_deg, (double)thrust);
	}
}

void HoldRigAtt::check_sibling_modules() const
{
	// mc_att_control performs no publisher-identity check: it takes whichever
	// vehicle_attitude_setpoint carries the newest timestamp. Two modules
	// publishing it therefore produce no error anywhere - they interleave at
	// loop rate and the vehicle chases the average. This check is the only
	// warning the operator will get.
	static const char *const kSiblingEnables[] = {"FRIGAX_EN", "FRIGPA_EN", "FRIG_EN", "FRIGP_EN"};

	for (const char *name : kSiblingEnables) {
		param_t p = param_find(name);

		if (p == PARAM_INVALID) {
			continue;
		}

		int32_t enabled = 0;

		if ((param_get(p, &enabled) == PX4_OK) && (enabled != 0)) {
			PX4_ERR("%s is enabled and publishes the same topics - setpoints will interleave. Disable it.",
				name);
		}
	}
}

void HoldRigAtt::publish_setpoint(hrt_abstime now, const Quatf &q_sp)
{
	offboard_control_mode_s ocm{};
	ocm.timestamp = now;
	ocm.position = false;
	ocm.velocity = false;
	ocm.acceleration = false;
	ocm.attitude = true;
	ocm.body_rate = false;
	ocm.thrust_and_torque = false;
	ocm.direct_actuator = false;
	_offboard_control_mode_pub.publish(ocm);

	vehicle_attitude_setpoint_s sp{};
	// The timestamp is load-bearing, not decoration: mc_att_control rejects
	// any setpoint that is not strictly newer than the last one it accepted.
	sp.timestamp = now;
	q_sp.copyTo(sp.q_d);
	sp.thrust_body[0] = 0.f;
	sp.thrust_body[1] = 0.f;
	sp.thrust_body[2] = _param_hratt_thrust.get();
	_vehicle_attitude_setpoint_pub.publish(sp);

	_last_publish_timestamp = now;
	++_setpoints_published;
}

void HoldRigAtt::publish_track_current_attitude()
{
	// Pre-OFFBOARD (and while the arm gate is holding): mirror the vehicle's
	// live attitude every cycle, so commander's pre-switch check ("is a
	// signal already being published?") passes and there is no jump the
	// instant OFFBOARD is engaged. Only vehicle_attitude is used, so this
	// admits the OFFBOARD switch with no GPS/position estimate at all.
	publish_setpoint(hrt_absolute_time(), _current_att);
}

void HoldRigAtt::publish_hold()
{
	// Republish the frozen leader. On a completed ramp that is _q_target
	// exactly, because the final step is clamped to land on it; on a
	// HRATT_RAMP_T cutoff it is wherever the leader reached, which is the
	// right thing to hold - parking short is a small step, whereas snapping
	// to the unreached target would be the large one.
	//
	// Nothing here ever ends the hold. The vehicle stays parked for as long
	// as OFFBOARD is engaged; leaving OFFBOARD is the only exit.
	publish_setpoint(hrt_absolute_time(), _qd_leader);
}

void HoldRigAtt::ramp_toward_target(hrt_abstime now)
{
	// Travel the shortest arc from wherever the leader is to wherever the
	// target is, at no more than HRATT_RATE - see hold_rig_att::ramp_step().
	// dt is clamped because a stalled or restarted work item would otherwise
	// hand the ramp one enormous step, which is precisely the instantaneous
	// jump the ramp exists to prevent.
	const float dt = math::constrain(static_cast<float>(now - _last_leader_update_time) * 1e-6f, 0.f, 0.1f);
	_last_leader_update_time = now;

	_qd_leader = ramp_step(_qd_leader, _q_target, math::radians(_param_hratt_rate.get()) * dt, _remaining);

	publish_setpoint(now, _qd_leader);
}

void HoldRigAtt::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup(desc);
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pu;
		_parameter_update_sub.copy(&pu);
		parameters_updated();
	}

	vehicle_status_s status;

	if (_vehicle_status_sub.update(&status)) {
		_nav_state = status.nav_state;
		_armed_time = status.armed_time;
	}

	vehicle_attitude_s att;

	if (_vehicle_attitude_sub.update(&att)) {
		_current_att = Quatf(att.q);
		_has_attitude = true;

		// An EKF yaw reset rotates the whole world frame under us. Anything
		// we are holding in that frame - the latched yaw, the target, the
		// leader - has to come with it, or the held attitude quietly drifts
		// by the reset amount. mc_att_control has its own repair for this
		// (adaptAttitudeSetpoint) but it is skipped whenever our setpoint is
		// newer than the attitude estimate, which for a module publishing
		// every cycle is always.
		if (!_quat_reset_counter_valid) {
			_quat_reset_counter = att.quat_reset_counter;
			_quat_reset_counter_valid = true;

		} else if (_quat_reset_counter != att.quat_reset_counter) {
			const Quatf delta_q_reset(att.delta_q_reset);

			// Left multiplication: the reset is a world-frame correction.
			_q_target = delta_q_reset * _q_target;
			_q_target.normalize();
			_qd_leader = delta_q_reset * _qd_leader;
			_qd_leader.normalize();
			_hold_yaw = matrix::wrap_pi(_hold_yaw + Eulerf(delta_q_reset).psi());

			_quat_reset_counter = att.quat_reset_counter;
		}
	}

	if (!_param_hratt_en.get()) {
		_was_offboard = false;
		++_early_return_disabled;
		return;
	}

	if (!_has_attitude) {
		// Nothing sane to publish yet - wait for the first attitude sample.
		// This is IMU/mag convergence only, not GPS - it clears in well
		// under a second, indoors or out.
		++_early_return_no_attitude;
		return;
	}

	const bool is_offboard = (_nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD);

	if (!is_offboard) {
		// Leaving OFFBOARD resets the latch, so the next entry re-latches
		// the yaw and runs a fresh ramp instead of staying "used up".
		_ramp_active = false;
		_hold_active = false;
		_was_offboard = false;
		publish_track_current_attitude();
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (!_was_offboard) {
		// Rising edge into OFFBOARD. Latch the yaw and build the target,
		// but do not start the ramp yet - the arm gate below decides that.
		_hold_yaw = Eulerf(_current_att).psi();
		build_target();
		_ramp_active = false;
		_hold_active = false;
		_was_offboard = true;
	}

	if (!_ramp_active && !_hold_active) {
		if (!past_arm_lockdown(now)) {
			// Hold station at the live attitude until the undeferrable
			// post-arm disarm window has passed. Publishing the tracking
			// setpoint keeps OFFBOARD alive meanwhile.
			++_arm_gate_waits;
			publish_track_current_attitude();
			return;
		}

		_ramp_active = true;
		_ramp_start_time = now;
		_last_leader_update_time = now;
		_qd_leader = _current_att;   // leader starts exactly at the live attitude: zero initial error

		_initial_distance = angle_between(_qd_leader, _q_target);
		_remaining = _initial_distance;

		if (_initial_distance > (M_PI_F - 1e-3f)) {
			// Almost exactly antipodal: the arc is a full half-turn and its
			// DIRECTION is decided by the sign of a float that is nominally
			// zero. The ramp is still stable and still arrives - the axis
			// comes from q_err.imag(), which is a unit vector here - but
			// which way round it goes is not predictable from the
			// parameters, and on a rig that is worth saying out loud.
			PX4_WARN("Target is %.0f deg away, near the 180 deg half-turn: travel direction is arbitrary.",
				 (double)math::degrees(_initial_distance));
		}
	}

	if (_ramp_active) {
		ramp_toward_target(now);

		const float elapsed_s = static_cast<float>(now - _ramp_start_time) * 1e-6f;
		const bool arrived = (_remaining < kArrivedRad);
		const bool timed_out = (elapsed_s >= _param_hratt_ramp_t.get());

		if (arrived || timed_out) {
			if (timed_out && !arrived) {
				PX4_WARN("HRATT_RAMP_T cutoff at %.1f s with %.0f deg left - holding here instead.",
					 (double)elapsed_s, (double)math::degrees(_remaining));
			}

			_ramp_active = false;
			_hold_active = true;
		}

		return;
	}

	// Holding. With HRATT_LIVE the target is re-evaluated every cycle, and a
	// parameter change re-engages the ramp toward the new attitude - safe
	// precisely because the ramp is convergent, so it picks up from wherever
	// the leader currently is and is rate-limited exactly as the first
	// approach was.
	if (_param_hratt_live.get()) {
		const Quatf previous_target = _q_target;
		build_target();

		// 0.5 deg of deadband: parameter values arrive quantized and the
		// build involves trigonometry, so an exact-equality test would
		// re-arm the ramp on float noise alone.
		if (angle_between(previous_target, _q_target) > math::radians(0.5f)) {
			_ramp_active = true;
			_hold_active = false;
			_ramp_start_time = now;
			_last_leader_update_time = now;
			_initial_distance = angle_between(_qd_leader, _q_target);
			_remaining = _initial_distance;

			ramp_toward_target(now);
			return;
		}
	}

	publish_hold();
}

int HoldRigAtt::task_spawn(int argc, char *argv[])
{
	HoldRigAtt *instance = new HoldRigAtt();

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

int HoldRigAtt::print_status()
{
	const Quatf q_rel = target_relative();

	float tilt_deg = 0.f;
	float dir_deg = 0.f;
	float dyaw_deg = 0.f;
	decompose(q_rel, tilt_deg, dir_deg, dyaw_deg);

	PX4_INFO("enabled=%d mode=%s rate=%.0f deg/s thrust=%.2f ramp_t=%.1f s live=%d arm_gate=%d",
		 (int)_param_hratt_en.get(),
		 (_param_hratt_mode.get() == 1) ? "euler" : "tilt+dir",
		 (double)_param_hratt_rate.get(),
		 (double)_param_hratt_thrust.get(),
		 (double)_param_hratt_ramp_t.get(),
		 (int)_param_hratt_live.get(),
		 (int)_param_hratt_arm_gate.get());

	// Report the tilt/direction/delta-yaw decomposition rather than an Euler
	// readback. Euler extraction is not injective near pitch = +-90, so a
	// readback there prints a different triple for the same orientation:
	// commanded (45, 90) comes back as (26.6, 90), and (180, 180) as
	// (0, 0, yaw 180).
	PX4_INFO("target: tilt=%.1f deg dir=%.0f deg delta_yaw=%.1f deg", (double)tilt_deg, (double)dir_deg,
		 (double)dyaw_deg);

	PX4_INFO("q_target=[%.4f %.4f %.4f %.4f]",
		 (double)_q_target(0), (double)_q_target(1), (double)_q_target(2), (double)_q_target(3));

	PX4_INFO("state=%s remaining=%.1f of %.1f deg",
		 _ramp_active ? "RAMP" : (_hold_active ? "HOLD" : (_was_offboard ? "ARM-GATE" : "TRACK")),
		 (double)math::degrees(_remaining), (double)math::degrees(_initial_distance));

	PX4_INFO("tx=%" PRIu64 " early_disabled=%" PRIu64 " early_no_attitude=%" PRIu64
		 " arm_gate_waits=%" PRIu64 " offboard=%d",
		 _setpoints_published, _early_return_disabled, _early_return_no_attitude,
		 _arm_gate_waits, (int)_was_offboard);

	if (_was_offboard) {
		PX4_INFO("hold_yaw=%.3f rad", (double)_hold_yaw);
	}

	return 0;
}

int HoldRigAtt::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int HoldRigAtt::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
`hold_rig_att` is a standalone, manually-started controller that ramps the
vehicle to an arbitrary commanded 3D attitude and holds it there, for a fixture
that physically prevents translation (e.g. a 3-axis gimbal/gyroscope test rig).
It is the standing-still counterpart to `flip_rig_axis`: that module rotates
through a full revolution about a fixed axis, this one travels to a single
orientation and parks on it.

Like `flip_rig_axis` it drives everything through the ATTITUDE setpoint path
(mc_att_control) rather than raw body rates, and never subscribes to or
publishes a position setpoint of any kind - so it needs nothing more than a
valid attitude estimate (no GPS, no optical flow, no motion capture). Start it
with `hold_rig_att start` and do not run it at the same time as another
OFFBOARD-publishing module; `HRATT_EN` defaults to 0 for that reason and the
startup check warns if a sibling is enabled.

The instant OFFBOARD is engaged it latches the vehicle's current yaw, ramps the
attitude setpoint from the live attitude to the commanded target at HRATT_RATE
along the shortest arc, and then republishes that attitude for as long as
OFFBOARD stays engaged. HRATT_THRUST is held throughout. It does NOT return to,
or hold, any position. Leaving OFFBOARD ends the hold and arms a fresh run.

#### Why this is not just a static setpoint

It is a reasonable expectation that holding an attitude means publishing the
target quaternion once and letting the controller converge. Two things get in
the way, and between them they account for most of this module.

First, a static setpoint arrives but arrives violently. The usual worry is that
the shortest-arc resolution in `AttitudeControl::update()`
(`eq = 2 * qe.canonical().imag()`, bounded to +-180 degrees) makes a large
target unreachable. It does not: the rotation from level to any (roll, pitch)
is at most 180 degrees, so the target is always on the near side of the wrap and
is always reached. What actually bites is the MAGNITUDE of the initial error.
The rate demand is `2*sin(Phi/2)*MC_ROLL_P`, hard-clamped at MC_ROLLRATE_MAX,
and with stock gains (4.0, 220 deg/s) that clamp is reached at Phi = 57 degrees.
A static roll-90/pitch-30 command presents Phi = 94 degrees at once and demands
335 deg/s - about 1.5x the clamp - so the vehicle slams into the target at full
authority, and with MC_AIRMODE=0 the allocator pays for the saturated torque by
cutting collective thrust. Ramping replaces that single large error with a
steady tracking lag of roughly HRATT_RATE/MC_ROLL_P (about 11 degrees at the
default), which saturates nothing.

Note this is a different problem from the one `flip_rig_axis` solves. There the
moving setpoint exists to get PAST the 180 degree wrap at all, since a static
target can never travel further than half a revolution. Here the wrap is never
reached and saturation is the whole issue.

Second, Euler roll/pitch is the wrong parameterization at large angles, which is
why HRATT_MODE exists:

- HRATT_MODE 0, TILT + DIRECTION (default): HRATT_TILT is the total angle the
  body z axis leans away from vertical and HRATT_DIR is which way, measured in
  the body xy-plane from body x toward body y using the same convention as
  FRIGAX_AXIS. Degenerate nowhere in 0-180 degrees.
- HRATT_MODE 1, EULER ROLL + PITCH: HRATT_ROLL and HRATT_PITCH in PX4's
  intrinsic Z-Y-X convention.

In Z-Y-X the pitch is applied about the already-rolled axis, so once roll
reaches 90 degrees the body z axis sits at [0, -1, 0] for EVERY pitch value and
the pitch command changes the tilt not at all. It is not discarded - it becomes
a rotation about the (now horizontal) body z axis, which
`AttitudeControl::update()` separates out as delta-yaw and scales by
MC_YAW_WEIGHT (0.4) because yaw is the weakest axis a multicopter has. So a
commanded (roll 90, pitch 30) executes as 90 degrees of tilt at full authority
plus 30 degrees on the weakest axis at 40% weight, which on the rig reads as
"the roll arrived and the pitch never did". Startup diagnostics always print the
mode-0 equivalent of a mode-1 command together with its delta-yaw share, so the
translation can be read straight out of the log.

#### Failsafes that will fire

This is rig-only for reasons beyond translation. Thrust is a fixed passthrough
with no 1/cos(tilt) compensation - the position controller normally provides
that and is not in the loop - so lift authority falls to zero at 90 degrees of
tilt and reverses beyond it, driving the airframe into its mount.

More importantly, the attitude failure detector compares Euler roll and pitch
against FD_FAIL_R/FD_FAIL_P (both 60 degrees by default, 0.3 s trigger) and is
armed whenever attitude control is enabled - which is exactly what this module
requires. Any hold past 60 degrees trips it. With CBRK_FLIGHTTERM at its 121212
default the steady-state consequence is only a warning, but within
COM_LKDOWN_TKO + COM_SPOOLUP_TIME of arming the action is an unconditional
DISARM marked `cannotBeDeferred()` - no circuit breaker or deferral suppresses
it. HRATT_ARM_GATE (on by default) holds the ramp until that window has passed;
setting FD_FAIL_R and FD_FAIL_P to 0 is the real fix. COM_DISARM_PRFLT and
COM_DISARM_LAND also fire on a fixture that never reports a takeoff. All of
these are checked at startup and on every parameter change, and warned about
rather than silently corrected.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("hold_rig_att", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int hold_rig_att_main(int argc, char *argv[])
{
	return ModuleBase::main(HoldRigAtt::desc, argc, argv);
}
