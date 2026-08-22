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

#include "FlipRigAxis.hpp"

#include <inttypes.h>
#include <math.h>

#include <mathlib/math/Limits.hpp>
#include <parameters/param.h>

using matrix::AxisAnglef;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

ModuleBase::Descriptor FlipRigAxis::desc{task_spawn, custom_command, print_usage};

FlipRigAxis::FlipRigAxis() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	parameters_updated();
}

bool FlipRigAxis::init()
{
	ScheduleOnInterval(_schedule_interval_us);
	return true;
}

Vector3f FlipRigAxis::flip_axis() const
{
	// Unit vector in the body xy-plane at FRIGAX_AXIS degrees from body x
	// toward body y. Unit length is the whole point: it is what makes the
	// per-step rotation vector's magnitude equal dtheta for every axis
	// setting, so FRIGAX_RATE keeps meaning "total deg/s" and the progress
	// accumulator stays exact. See publish_flip_attitude().
	const float alpha = math::radians(_param_frigax_axis.get());
	return Vector3f(cosf(alpha), sinf(alpha), 0.f);
}

void FlipRigAxis::parameters_updated()
{
	updateParams();

	const int requested_hz = _param_frigax_pub_hz.get();
	const int clamped_hz = (requested_hz < 1) ? 1 : ((requested_hz > 1000) ? 1000 : requested_hz);
	const hrt_abstime new_interval_us = 1'000'000 / static_cast<hrt_abstime>(clamped_hz);

	if (new_interval_us != _schedule_interval_us) {
		_schedule_interval_us = new_interval_us;
		ScheduleClear();
		ScheduleOnInterval(_schedule_interval_us);
	}

	check_rate_against_limits();
	check_duration_against_rate();
}

void FlipRigAxis::check_rate_against_limits() const
{
	// FRIGAX_RATE only drives the leader setpoint in publish_flip_attitude().
	// The rate the vehicle actually gets commanded to is whatever
	// AttitudeControl::update() outputs, and that is bounded twice over,
	// per axis. Ask for more than either bound allows and the vehicle can
	// structurally never keep pace: tracking error grows past the
	// controller's +-180 degree shortest-arc limit, qe.canonical() flips
	// sign, and the flip stalls and reverses instead of completing -
	// exactly the failure this module exists to avoid.
	//
	// FRIGAX_AXIS splits the commanded rate between the two axes, so each
	// one only ever sees a fraction of FRIGAX_RATE and has to be checked
	// against its own limits: tilting the axis away from pure pitch relieves
	// the pitch ceilings but loads the roll ceilings in exchange.
	const Vector3f axis = flip_axis();
	const float rate = _param_frigax_rate.get();

	check_axis_share("roll", rate * fabsf(axis(0)), "MC_ROLLRATE_MAX", "MC_ROLL_P");
	check_axis_share("pitch", rate * fabsf(axis(1)), "MC_PITCHRATE_MAX", "MC_PITCH_P");
}

void FlipRigAxis::check_axis_share(const char *axis_label, float axis_rate_deg,
				   const char *rate_max_param, const char *att_p_param) const
{
	if (axis_rate_deg < 1e-3f) {
		// FRIGAX_AXIS puts nothing on this axis, so its limits are irrelevant.
		return;
	}

	// Ceiling 1: the hard clamp applied to the attitude controller's output
	// (AttitudeControl::update(), reference Eq. 54). Warn with 20% margin,
	// because merely matching the limit leaves the vehicle no headroom to
	// ever catch up to the leader - only to hold station behind it.
	param_t p = param_find(rate_max_param);
	float rate_max_deg = 0.f;

	// Keep every message inside the 127-char cap of log_message_s.text,
	// or the tail - which is where the actionable part lives - is silently
	// chopped off. Detail belongs in the comments and the parameter docs.
	if ((p != PARAM_INVALID) && (param_get(p, &rate_max_deg) == PX4_OK) && (rate_max_deg > 0.f) &&
	    (axis_rate_deg > 0.8f * rate_max_deg)) {
		PX4_WARN("FRIGAX_RATE puts %.0f deg/s on %s, >80%% of %s (%.0f) - flip will stall and reverse.",
			 (double)axis_rate_deg, axis_label, rate_max_param, (double)rate_max_deg);
	}

	// Ceiling 2: the attitude P-law itself. The error vector is
	// eq = 2*sin(phi/2)*n_hat, which saturates at magnitude 2, so the loop
	// can never command more than 2*MC_*_P rad/s however far behind the
	// leader gets - independently of the clamp above. A low attitude gain
	// can therefore defeat the flip on its own.
	p = param_find(att_p_param);
	float att_p = 0.f;

	if ((p != PARAM_INVALID) && (param_get(p, &att_p) == PX4_OK) && (att_p > 0.f)) {
		const float p_ceiling_deg = math::degrees(2.f * att_p);

		if (axis_rate_deg > 0.8f * p_ceiling_deg) {
			PX4_WARN("FRIGAX_RATE puts %.0f deg/s on %s, >80%% of 2*%s ceiling (%.0f) - loop cannot keep up.",
				 (double)axis_rate_deg, axis_label, att_p_param, (double)p_ceiling_deg);
		}
	}
}

void FlipRigAxis::check_duration_against_rate() const
{
	// The flip phase ends on whichever fires first: the leader reaching the
	// target angle, or the FRIGAX_DURATION safety cutoff. Both run off the
	// same clock, and the leader's progress is exact, so completion needs
	// simply duration > travel/rate.
	const float rate = _param_frigax_rate.get();

	if (rate < 1e-3f) {
		return;
	}

	const float hold_ang = _param_frigax_hold_ang.get();
	const bool hold_mode = (hold_ang > 1e-3f);
	const float travel_deg = hold_mode ? hold_ang : 360.f;
	const float needed_s = travel_deg / rate;
	const float duration_s = _param_frigax_duration.get();

	if (duration_s > needed_s) {
		return;
	}

	const float reached_deg = rate * duration_s;

	if (hold_mode) {
		// Falling short here is benign - the module simply holds at a
		// smaller angle than asked for (see publish_attitude_hold), with no
		// snap back to level and no reversal. Still worth saying, because
		// the vehicle quietly parks somewhere other than where it was told.
		PX4_WARN("FRIGAX_DURATION %.1f s reaches only %.0f of %.0f deg at %.0f deg/s - raise above %.1f s.",
			 (double)duration_s, (double)reached_deg, (double)travel_deg, (double)rate, (double)needed_s);
		return;
	}

	// Full-revolution mode: falling short is NOT benign. publish_level_hold()
	// takes the SHORTEST arc back to level, so an abort anywhere in roughly
	// [57, 303] degrees commands the full MC_PITCHRATE_MAX clamp in one
	// cycle - a violent direction change that saturates the mixer and, with
	// MC_AIRMODE=0, is paid for by cutting collective thrust.
	PX4_WARN("FRIGAX_DURATION %.1f s < %.1f s needed at %.0f deg/s - flip aborts at %.0f deg. Raise it.",
		 (double)duration_s, (double)needed_s, (double)rate, (double)reached_deg);

	// Split rather than appended: the combined text would exceed the
	// 127-char log_message_s cap and lose exactly this warning.
	if (reached_deg < 180.f) {
		PX4_WARN("Abort is below 180 deg: the level-hold REVERSES the vehicle instead of finishing.");
	}
}

void FlipRigAxis::publish_track_current_attitude()
{
	// Pre-OFFBOARD: mirror the vehicle's live attitude every cycle, so
	// commander's pre-switch check ("is a signal already being published?")
	// passes and there is no jump the instant OFFBOARD is engaged. Only
	// vehicle_attitude is used, so this admits the OFFBOARD switch with no
	// GPS/position estimate at all.
	const hrt_abstime now = hrt_absolute_time();

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
	sp.timestamp = now;
	_current_att.copyTo(sp.q_d);
	sp.thrust_body[0] = 0.f;
	sp.thrust_body[1] = 0.f;
	sp.thrust_body[2] = _param_frigax_thrust.get();
	_vehicle_attitude_setpoint_pub.publish(sp);

	_last_publish_timestamp = now;
	++_setpoints_published;
}

void FlipRigAxis::publish_level_hold()
{
	// After the flip completes (and while waiting for one to start): command
	// a level attitude at the yaw latched on OFFBOARD entry. This is an
	// active attitude command, so the vehicle is actively driven back to
	// upright rather than left wherever the flip happened to end. No
	// position is latched or chased - by design, this never tries to return
	// to a start point.
	//
	// After a clean flip the step into this phase is only the tracking lag
	// (tens of degrees, in the direction of travel) and the vehicle simply
	// finishes the turn. After an aborted flip it is whatever was left of
	// the revolution, taken the short way - see check_duration_against_rate().
	const hrt_abstime now = hrt_absolute_time();

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
	sp.timestamp = now;
	Quatf(Eulerf(0.f, 0.f, _hold_yaw)).copyTo(sp.q_d);
	sp.thrust_body[0] = 0.f;
	sp.thrust_body[1] = 0.f;
	sp.thrust_body[2] = _param_frigax_thrust.get();
	_vehicle_attitude_setpoint_pub.publish(sp);

	_last_publish_timestamp = now;
	++_setpoints_published;
}

void FlipRigAxis::publish_attitude_hold()
{
	// FRIGAX_HOLD_ANG mode, after the ramp has finished: keep republishing
	// the frozen leader, i.e. the attitude the ramp stopped on. The vehicle
	// is at most one tracking lag away from it, so entering this phase is a
	// step of a few tens of degrees at worst - and usually far less, since
	// the lag decays as soon as the leader stops moving. That is the whole
	// reason this exists as a separate phase from publish_level_hold(),
	// which would instead command a jump back to level: from a parked
	// attitude past ~57 degrees that jump saturates the rate clamp outright.
	//
	// Nothing here ever ends the hold. The vehicle stays parked at this
	// attitude for as long as OFFBOARD is engaged; leaving OFFBOARD is the
	// only exit, and it re-arms a fresh single run.
	const hrt_abstime now = hrt_absolute_time();

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
	sp.timestamp = now;
	_qd_leader.copyTo(sp.q_d);
	sp.thrust_body[0] = 0.f;
	sp.thrust_body[1] = 0.f;
	sp.thrust_body[2] = _param_frigax_thrust.get();
	_vehicle_attitude_setpoint_pub.publish(sp);

	_last_publish_timestamp = now;
	++_setpoints_published;
}

void FlipRigAxis::publish_flip_attitude(hrt_abstime now)
{
	// The flip itself: a rotating "leader" attitude setpoint, advanced every
	// cycle by a fixed increment about _flip_axis, expressed in ITS OWN
	// (evolving) body frame - a right multiplication, not left - so this
	// integrates exactly like a constant body-frame angular rate would,
	// tracing a fixed-axis revolution. A left multiplication would apply the
	// increment in the world frame instead, making the manoeuvre depend on
	// the vehicle's initial yaw (a "pitch" flip would become a roll flip at
	// 90 degrees of yaw).
	//
	// The leader never reads the vehicle's actual attitude back once armed:
	// it free-spins on the clock alone, so mc_att_control's shortest-arc
	// error (qe.canonical() in AttitudeControl::update(), bounded to +-180
	// degrees) only ever sees the small tracking lag between leader and
	// vehicle - never the full flip angle - and so never wraps and reverses
	// direction.
	const float dt = math::constrain(static_cast<float>(now - _last_leader_update_time) * 1e-6f, 0.f, 0.1f);
	_last_leader_update_time = now;

	// dtheta is the TOTAL rotation for this step; _flip_axis is a unit
	// vector, so the rotation vector dtheta * _flip_axis has magnitude
	// exactly dtheta whatever the axis. That is what keeps FRIGAX_RATE
	// meaning "total deg/s" regardless of axis, and keeps
	// _accumulated_rotation below an exact measure of progress.
	//
	// Note this is NOT the same as feeding the two components in
	// independently (e.g. AxisAnglef(rate*dt, rate*dt, 0) for a diagonal):
	// that rotation vector has magnitude sqrt(2)*dtheta, so the vehicle
	// would turn 41% faster than commanded while the accumulator still
	// counted only one component, and the flip would not stop until the
	// vehicle had actually rotated sqrt(2) * 360 = 509 degrees.
	float dtheta = math::radians(_param_frigax_rate.get()) * dt;

	// Never step past the target. Without this the leader stops up to one
	// whole step beyond it (1.2 deg at 120 deg/s and 100 Hz), which does not
	// matter for a full revolution but does when FRIGAX_HOLD_ANG asks for a
	// specific parked attitude. Clamping the last step lands the leader
	// exactly on the requested angle.
	dtheta = math::min(dtheta, math::max(_target_rotation - _accumulated_rotation, 0.f));

	const matrix::Vector3f rotvec = _flip_axis * dtheta;
	_qd_leader = _qd_leader * Quatf(AxisAnglef(rotvec(0), rotvec(1), rotvec(2)));
	_qd_leader.normalize();
	_accumulated_rotation += dtheta;

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
	sp.timestamp = now;
	_qd_leader.copyTo(sp.q_d);
	sp.thrust_body[0] = 0.f;
	sp.thrust_body[1] = 0.f;
	sp.thrust_body[2] = _param_frigax_thrust.get();
	_vehicle_attitude_setpoint_pub.publish(sp);

	_last_publish_timestamp = now;
	++_setpoints_published;
}

void FlipRigAxis::Run()
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
	}

	vehicle_attitude_s att;

	if (_vehicle_attitude_sub.update(&att)) {
		_current_att = Quatf(att.q);
		_has_attitude = true;
	}

	if (!_param_frigax_en.get()) {
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

	if (is_offboard && !_was_offboard) {
		// Rising edge into OFFBOARD: latch the yaw to hold after the flip,
		// and arm a fresh single-shot flip for this entry.
		_hold_yaw = Eulerf(_current_att).psi();
		_flip_active = false;
		_flip_done = false;
	}

	if (!is_offboard) {
		// Leaving OFFBOARD resets the one-shot latch, so the next entry
		// flips again exactly once, instead of staying "used up" forever.
		_flip_active = false;
		_flip_done = false;
	}

	_was_offboard = is_offboard;

	if (!is_offboard) {
		publish_track_current_attitude();
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if (!_flip_active && !_flip_done) {
		_flip_active = true;
		_flip_start_time = now;
		_accumulated_rotation = 0.f;
		_qd_leader = _current_att;        // leader starts exactly at the live attitude: zero initial error
		_flip_axis = flip_axis();         // sampled once and held: the axis must not drift mid-flip
		_last_leader_update_time = now;

		// Latched with the axis, for the same reason: a parameter change
		// must not redefine the manoeuvre already under way.
		const float hold_ang_deg = _param_frigax_hold_ang.get();
		_hold_at_angle = (hold_ang_deg > 1e-3f);
		_target_rotation = _hold_at_angle ? math::radians(hold_ang_deg) : (2.f * M_PI_F);
	}

	if (_flip_active) {
		publish_flip_attitude(now);

		const float elapsed_s = static_cast<float>(now - _flip_start_time) * 1e-6f;
		// _accumulated_rotation is the leader's own exact progress, not a
		// noisy measurement of the real vehicle - so unlike the rate-
		// controlled modules there's no need for a 95%-of-target allowance.
		// Wait for the genuine full travel; duration is only a safety
		// cutoff, and check_duration_against_rate() warns at startup if it
		// is set too low to ever be reached.
		const bool done = (_accumulated_rotation >= _target_rotation) || (elapsed_s >= _param_frigax_duration.get());

		if (done) {
			_flip_active = false;
			_flip_done = true;
		}

		return;
	}

	// _flip_done: hold, and stay there - this module never runs again on its
	// own while OFFBOARD stays engaged.
	if (_hold_at_angle) {
		// Park at the attitude the ramp stopped on, wherever that is. This
		// also covers the duration-cutoff case: holding short of the target
		// is a small step, whereas returning to level from a partial angle
		// is the large one.
		publish_attitude_hold();

	} else {
		// Full revolution: come back to level at the latched yaw.
		publish_level_hold();
	}
}

int FlipRigAxis::task_spawn(int argc, char *argv[])
{
	FlipRigAxis *instance = new FlipRigAxis();

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

int FlipRigAxis::print_status()
{
	const Vector3f axis = flip_axis();

	PX4_INFO("enabled=%d axis=%.0f deg (roll %.2f, pitch %.2f) rate=%.0f deg/s thrust=%.2f duration=%.2f s",
		 (int)_param_frigax_en.get(),
		 (double)_param_frigax_axis.get(),
		 (double)axis(0), (double)axis(1),
		 (double)_param_frigax_rate.get(),
		 (double)_param_frigax_thrust.get(),
		 (double)_param_frigax_duration.get());

	const float rate = _param_frigax_rate.get();
	const float hold_ang = _param_frigax_hold_ang.get();
	const bool hold_mode = (hold_ang > 1e-3f);
	const float travel_deg = hold_mode ? hold_ang : 360.f;

	PX4_INFO("mode=%s travel=%.0f deg, needs %.2f s at this rate; cutoff at %.2f s",
		 hold_mode ? "ramp-and-hold" : "full-revolution",
		 (double)travel_deg,
		 (double)((rate > 1e-3f) ? (travel_deg / rate) : 0.f),
		 (double)_param_frigax_duration.get());

	PX4_INFO("tx=%" PRIu64 " early_disabled=%" PRIu64 " early_no_attitude=%" PRIu64
		 " offboard=%d active=%d done=%d leader_rot=%.1f of %.0f deg",
		 _setpoints_published, _early_return_disabled, _early_return_no_attitude,
		 (int)_was_offboard, (int)_flip_active, (int)_flip_done,
		 (double)math::degrees(_accumulated_rotation),
		 (double)math::degrees(_target_rotation));

	if (_was_offboard) {
		PX4_INFO("hold_yaw=%.3f rad", (double)_hold_yaw);
	}

	return 0;
}

int FlipRigAxis::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlipRigAxis::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
`flip_rig_axis` is a standalone, manually-started single-shot 360 degree
flip controller for a fixture that physically prevents translation (e.g. a
3-axis gimbal/gyroscope test rig). It generalises `flip_rig_pitch_att`: the
rotation axis is a free parameter anywhere in the body xy-plane, so one
module performs roll flips, pitch flips, and any diagonal between them.

Like `flip_rig_pitch_att` it drives the flip through the ATTITUDE setpoint
path (mc_att_control) rather than raw body rates, publishing a continuously
rotating "leader" attitude setpoint that always starts at the vehicle's live
attitude and advances on its own clock. That way mc_att_control's
shortest-arc tracking error never has to represent more than the small lag
between leader and vehicle, avoiding the +-180 degree wraparound that stops
a *static* attitude setpoint from ever completing more than half a rotation.

It never subscribes to or publishes a position setpoint of any kind - entry,
the flip, and the post-flip hold are all attitude-only, so it needs nothing
more than a valid attitude estimate (no GPS, no optical flow, no motion
capture). Start it directly with `flip_rig_axis start` from the shell, and do
not run it at the same time as another OFFBOARD-publishing module - including
`flip_rig_pitch_att`, which publishes the same topics.

The instant OFFBOARD is engaged it latches the vehicle's current yaw and
rotates about the FRIGAX_AXIS direction at FRIGAX_RATE deg/s, holding
FRIGAX_THRUST throughout and capped at FRIGAX_DURATION seconds as a safety
cutoff. It does NOT return to, or hold, any position. It does not run again
on its own; leaving and re-entering OFFBOARD re-latches the yaw and arms a
fresh single run.

FRIGAX_HOLD_ANG selects between the two things it can do:

- 0 (default), FULL-REVOLUTION mode: one complete 360 degree rotation, then a
  level attitude at the latched yaw, held.
- greater than 0, RAMP-AND-HOLD mode: the leader ramps only as far as that
  angle, stops exactly on it, and holds THAT attitude until OFFBOARD is left.
  The vehicle does not return to level. Use it to park the airframe at a
  fixed tilt on the rig.

Ramping rather than stepping is what makes the second mode safe, and is why
it is worth doing here rather than by publishing a static setpoint: a static
target presents the whole angle to the attitude controller as instantaneous
error, and past roughly 57 degrees that saturates MC_PITCHRATE_MAX outright.
The ramp keeps the error at the normal small tracking lag the whole way.
Angles beyond 180 degrees work for the same reason the full revolution does -
the +-180 degree shortest-arc limit binds the tracking error, not the total
distance travelled.

FRIGAX_AXIS is measured in the body xy-plane from body x toward body y: 0 is
a pure roll flip, 90 (the default) a pure pitch flip, 270 a pitch flip the
other way round. FRIGAX_RATE is always the total angular speed about whichever
axis is chosen, so changing the axis alone changes only the direction of the
flip. Torque authority is not isotropic: on a quad-X the 45/135/225/315 degree
axes lie along a motor arm, where only two rotors can generate torque and the
mixer clips at about 71% of what a pure roll or pure pitch flip gets.

Two configuration mistakes are checked at startup and on every parameter
change, and warned about rather than silently corrected:

- FRIGAX_RATE too high for the axis it is applied to. Each axis' share must
  stay clear of both that axis' rate clamp (MC_ROLLRATE_MAX/MC_PITCHRATE_MAX)
  and its attitude P-law ceiling (2 * MC_ROLL_P / 2 * MC_PITCH_P). Exceed
  either and the vehicle cannot keep pace, tracking error runs past 180
  degrees, and the flip stalls and reverses instead of completing.
- FRIGAX_DURATION too short for the travel asked of it. In full-revolution
  mode the cutoff then fires before the revolution finishes, and because the
  level-hold takes the shortest arc back to level, an abort anywhere in
  roughly 57 to 303 degrees commands the full rate clamp in a single cycle -
  a violent reversal that saturates the mixer and, with MC_AIRMODE=0, is paid
  for by cutting collective thrust. In ramp-and-hold mode the same shortfall
  is harmless: the module simply parks at a smaller angle.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flip_rig_axis", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int flip_rig_axis_main(int argc, char *argv[])
{
	return ModuleBase::main(FlipRigAxis::desc, argc, argv);
}
