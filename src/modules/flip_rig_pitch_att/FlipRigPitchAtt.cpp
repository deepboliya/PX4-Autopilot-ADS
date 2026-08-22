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

#include "FlipRigPitchAtt.hpp"

#include <inttypes.h>
#include <math.h>

#include <mathlib/math/Limits.hpp>
#include <parameters/param.h>

using matrix::AxisAnglef;
using matrix::Eulerf;
using matrix::Quatf;

ModuleBase::Descriptor FlipRigPitchAtt::desc{task_spawn, custom_command, print_usage};

FlipRigPitchAtt::FlipRigPitchAtt() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	parameters_updated();
}

bool FlipRigPitchAtt::init()
{
	ScheduleOnInterval(_schedule_interval_us);
	return true;
}

void FlipRigPitchAtt::parameters_updated()
{
	updateParams();

	const int requested_hz = _param_frigpa_pub_hz.get();
	const int clamped_hz = (requested_hz < 1) ? 1 : ((requested_hz > 1000) ? 1000 : requested_hz);
	const hrt_abstime new_interval_us = 1'000'000 / static_cast<hrt_abstime>(clamped_hz);

	if (new_interval_us != _schedule_interval_us) {
		_schedule_interval_us = new_interval_us;
		ScheduleClear();
		ScheduleOnInterval(_schedule_interval_us);
	}

	// FRIGPA_RATE only drives the leader setpoint in publish_flip_attitude() -
	// the rate the vehicle actually gets commanded to is whatever
	// AttitudeControl::update() outputs, which is hard-clamped to
	// MC_PITCHRATE_MAX regardless of how far ahead the leader is. Ask for a
	// leader rate at or above that limit and the vehicle can structurally
	// never keep pace: tracking error grows past the controller's +-180
	// degree shortest-arc limit and the flip stalls and reverses instead of
	// completing - exactly the failure this module exists to avoid. Warn
	// rather than silently clamp, since silently altering a commanded rate
	// on a physical rig is worse than a noisy log.
	param_t p = param_find("MC_PITCHRATE_MAX");
	float pitchrate_max_deg = 0.f;

	if ((p != PARAM_INVALID) && (param_get(p, &pitchrate_max_deg) == PX4_OK) && (pitchrate_max_deg > 0.f) &&
	    (_param_frigpa_rate.get() > 0.8f * pitchrate_max_deg)) {
		PX4_WARN("FRIGPA_RATE (%.0f deg/s) is within 20%% of MC_PITCHRATE_MAX (%.0f deg/s) - "
			 "the flip will stall and reverse instead of completing. Lower FRIGPA_RATE or raise MC_PITCHRATE_MAX.",
			 (double)_param_frigpa_rate.get(), (double)pitchrate_max_deg);
	}
}

void FlipRigPitchAtt::publish_track_current_attitude()
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
	sp.thrust_body[2] = _param_frigpa_thrust.get();
	_vehicle_attitude_setpoint_pub.publish(sp);

	_last_publish_timestamp = now;
	++_setpoints_published;
}

void FlipRigPitchAtt::publish_level_hold()
{
	// After the flip completes (and while waiting for one to start): command
	// a level attitude at the yaw latched on OFFBOARD entry. This is an
	// active attitude command, so the vehicle is actively driven back to
	// upright rather than left wherever the flip happened to end. No
	// position is latched or chased - by design, this never tries to return
	// to a start point.
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
	sp.thrust_body[2] = _param_frigpa_thrust.get();
	_vehicle_attitude_setpoint_pub.publish(sp);

	_last_publish_timestamp = now;
	++_setpoints_published;
}

void FlipRigPitchAtt::publish_attitude_hold()
{
	// FRIGPA_HOLD_ANG mode, once the ramp has finished: keep republishing the
	// frozen leader, i.e. the attitude the ramp stopped on. The vehicle is at
	// most one tracking lag away from it, and that lag decays to zero as soon
	// as the leader stops advancing, so entering this phase is a step of a
	// few tens of degrees at worst.
	//
	// That is exactly why this is a separate phase rather than reusing
	// publish_level_hold(): from a parked attitude past roughly 57 degrees a
	// jump back to level saturates the MC_PITCHRATE_MAX clamp outright, which
	// spikes the torque demand and (with MC_AIRMODE=0) makes the mixer pay
	// for it out of collective thrust.
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
	sp.thrust_body[2] = _param_frigpa_thrust.get();
	_vehicle_attitude_setpoint_pub.publish(sp);

	_last_publish_timestamp = now;
	++_setpoints_published;
}

void FlipRigPitchAtt::publish_flip_attitude(hrt_abstime now)
{
	// The flip itself: a rotating "leader" attitude setpoint, advanced every
	// cycle by a fixed pitch increment expressed in ITS OWN (evolving) body
	// frame - a right multiplication, not left - so this integrates exactly
	// like a constant body-frame pitch rate would, tracing a fixed-axis
	// revolution. The leader never reads the vehicle's actual attitude back
	// once armed: it free-spins on the clock alone, so mc_att_control's
	// shortest-arc error (qe.canonical() in AttitudeControl::update(),
	// bounded to +-180 degrees) only ever sees the small tracking lag
	// between leader and vehicle - never the full flip angle - and so never
	// wraps and reverses direction.
	const float dt = math::constrain(static_cast<float>(now - _last_leader_update_time) * 1e-6f, 0.f, 0.1f);
	_last_leader_update_time = now;

	float dtheta = math::radians(_param_frigpa_rate.get()) * dt;

	// Never step past the target. Without this the leader stops up to one
	// whole step beyond it (1.2 deg at 120 deg/s and 100 Hz), which does not
	// matter for a full revolution but does when FRIGPA_HOLD_ANG asks to park
	// at a specific pitch. Clamping the last step lands the leader exactly on
	// the requested angle.
	dtheta = math::min(dtheta, math::max(_target_rotation - _accumulated_rotation, 0.f));

	_qd_leader = _qd_leader * Quatf(AxisAnglef(0.f, dtheta, 0.f));
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
	sp.thrust_body[2] = _param_frigpa_thrust.get();
	_vehicle_attitude_setpoint_pub.publish(sp);

	_last_publish_timestamp = now;
	++_setpoints_published;
}

void FlipRigPitchAtt::Run()
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

	if (!_param_frigpa_en.get()) {
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
		_last_leader_update_time = now;

		// Latched here, not read per-cycle: a parameter change must not
		// redefine the manoeuvre already under way. Same reasoning as
		// _hold_yaw above.
		const float hold_ang_deg = _param_frigpa_hold_ang.get();
		_hold_at_angle = (hold_ang_deg > 1e-3f);
		_target_rotation = _hold_at_angle ? math::radians(hold_ang_deg) : (2.f * M_PI_F);
	}

	if (_flip_active) {
		publish_flip_attitude(now);

		const float elapsed_s = static_cast<float>(now - _flip_start_time) * 1e-6f;
		// _accumulated_rotation is the leader's own exact progress, not a
		// noisy measurement of the real vehicle - so unlike the rate-
		// controlled modules there's no need for a 95%-of-target allowance.
		// Wait for the genuine full travel.
		const bool reached = (_accumulated_rotation >= _target_rotation);

		// FRIGPA_DURATION guards the 360 flip only. In hold mode the ramp is
		// bounded by construction - it always terminates after
		// FRIGPA_HOLD_ANG / FRIGPA_RATE seconds, at worst 360/30 = 12 s - so
		// there is no runaway for a cutoff to catch, and applying one could
		// only strand the vehicle short of the angle it was told to hold.
		const bool timed_out = !_hold_at_angle && (elapsed_s >= _param_frigpa_duration.get());
		const bool done = reached || timed_out;

		if (done) {
			_flip_active = false;
			_flip_done = true;
		}

		return;
	}

	// _flip_done: hold, and stay there - this module never runs again on its
	// own while OFFBOARD stays engaged.
	if (_hold_at_angle) {
		// Park at the attitude the ramp stopped on.
		publish_attitude_hold();

	} else {
		// Full flip: come back to level at the latched yaw.
		publish_level_hold();
	}
}

int FlipRigPitchAtt::task_spawn(int argc, char *argv[])
{
	FlipRigPitchAtt *instance = new FlipRigPitchAtt();

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

int FlipRigPitchAtt::print_status()
{
	const float rate = _param_frigpa_rate.get();
	const float hold_ang = _param_frigpa_hold_ang.get();
	const bool hold_mode = (hold_ang > 1e-3f);
	const float travel_deg = hold_mode ? hold_ang : 360.f;

	PX4_INFO("enabled=%d rate=%.0f deg/s thrust=%.2f duration=%.2f s",
		 (int)_param_frigpa_en.get(),
		 (double)rate,
		 (double)_param_frigpa_thrust.get(),
		 (double)_param_frigpa_duration.get());

	PX4_INFO("mode=%s travel=%.0f deg, ramp takes %.2f s at this rate (cutoff %s)",
		 hold_mode ? "ramp-and-hold" : "full-flip",
		 (double)travel_deg,
		 (double)((rate > 1e-3f) ? (travel_deg / rate) : 0.f),
		 hold_mode ? "not applied" : "armed");

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

int FlipRigPitchAtt::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlipRigPitchAtt::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
`flip_rig_pitch_att` is a standalone, manually-started single-shot 360
degree pitch flip controller for a fixture that physically prevents
translation (e.g. a 3-axis gimbal/gyroscope test rig). Identical intent to
`flip_rig_pitch`, but drives the flip through the ATTITUDE setpoint path
(mc_att_control) instead of raw body rates: it publishes a continuously
rotating "leader" attitude setpoint that always starts at the vehicle's live
attitude and advances on its own clock, so mc_att_control's shortest-arc
tracking error never has to represent more than the small lag between
leader and vehicle - avoiding the +-180 degree wraparound that stops a
*static* attitude setpoint from ever completing more than half a rotation.

It never subscribes to or publishes a position setpoint of any kind - entry,
the flip, and the post-flip hold are all attitude-only, so it needs nothing
more than a valid attitude estimate (no GPS, no optical flow, no motion
capture). Start it directly with `flip_rig_pitch_att start` from the shell,
and do not run it at the same time as another OFFBOARD-publishing module;
avoiding that conflict is the operator's responsibility.

Once it has a valid attitude estimate, it continuously streams a live
attitude-tracking setpoint (required for commander to admit the OFFBOARD
switch at all). The instant OFFBOARD is engaged it latches the vehicle's
current yaw and pitches by chasing the leader setpoint (FRIGPA_RATE deg/s,
FRIGPA_THRUST held thrust). It does NOT return to, or hold, any position. It
does not run again on its own; leaving and re-entering OFFBOARD re-latches
the yaw and arms a fresh single run.

FRIGPA_HOLD_ANG selects between the two things it can do:

- 0 (the default), FULL-FLIP mode: one complete 360 degree pitch rotation,
  then a level attitude at the latched yaw, held. FRIGPA_DURATION caps the
  rotation as a safety cutoff.
- greater than 0, RAMP-AND-HOLD mode: the leader ramps only as far as that
  pitch angle, stops exactly on it, and holds THAT attitude until OFFBOARD is
  left. The vehicle does not return to level. Use it to park the airframe at
  a fixed pitch on the rig. FRIGPA_DURATION is NOT applied in this mode - the
  ramp is bounded by construction at FRIGPA_HOLD_ANG/FRIGPA_RATE seconds, so
  a cutoff could only strand the vehicle short of the angle it was told to
  hold.

Ramping rather than stepping is why the second mode is done here at all
instead of by publishing a static setpoint. A static target presents the
whole angle to mc_att_control as instantaneous error, and past roughly 57
degrees that saturates MC_PITCHRATE_MAX outright, spiking the torque demand
and (with MC_AIRMODE=0) making the mixer pay for it out of collective thrust.
The ramp keeps the error at the normal small tracking lag the whole way.
Angles beyond 180 degrees work for the same reason the full flip does: the
+-180 degree shortest-arc limit binds the tracking ERROR, not the total
distance travelled.

Note that in ramp-and-hold mode the airframe stays tilted until OFFBOARD is
left, and that at large pitch angles very little of the thrust is vertical
(only cos(79 deg) = 19% at 79 degrees) - this mode is for a fixture that
cannot translate, not for free flight.

FRIGPA_RATE must stay well below MC_PITCHRATE_MAX: the leader is only useful
as long as the real vehicle can keep pace with it, and the attitude
controller's own output is hard-clamped to MC_PITCHRATE_MAX regardless of
how far behind the leader it falls. A rate too close to that limit
reproduces the exact stall-and-reverse failure this module exists to avoid.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flip_rig_pitch_att", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int flip_rig_pitch_att_main(int argc, char *argv[])
{
	return ModuleBase::main(FlipRigPitchAtt::desc, argc, argv);
}
