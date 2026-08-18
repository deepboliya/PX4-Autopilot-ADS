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

#include "FlipRig.hpp"

#include <inttypes.h>
#include <math.h>

using matrix::Eulerf;
using matrix::Quatf;

ModuleBase::Descriptor FlipRig::desc{task_spawn, custom_command, print_usage};

FlipRig::FlipRig() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	parameters_updated();
}

bool FlipRig::init()
{
	ScheduleOnInterval(_schedule_interval_us);
	return true;
}

void FlipRig::parameters_updated()
{
	updateParams();

	const int requested_hz = _param_frig_pub_hz.get();
	const int clamped_hz = (requested_hz < 1) ? 1 : ((requested_hz > 1000) ? 1000 : requested_hz);
	const hrt_abstime new_interval_us = 1'000'000 / static_cast<hrt_abstime>(clamped_hz);

	if (new_interval_us != _schedule_interval_us) {
		_schedule_interval_us = new_interval_us;
		ScheduleClear();
		ScheduleOnInterval(_schedule_interval_us);
	}
}

void FlipRig::publish_track_current_attitude()
{
	// Pre-OFFBOARD: mirror the vehicle's live attitude every cycle, so
	// commander's pre-switch check ("is a signal already being published?")
	// passes and there is no jump the instant OFFBOARD is engaged. Unlike
	// flip_360 this never touches position - only vehicle_attitude is used,
	// so this admits the OFFBOARD switch with no GPS/position estimate at
	// all.
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
	sp.thrust_body[2] = _param_frig_thrust.get();
	_vehicle_attitude_setpoint_pub.publish(sp);

	_last_publish_timestamp = now;
	++_setpoints_published;
}

void FlipRig::publish_level_hold()
{
	// After the flip completes (and while waiting for one to start): command
	// a level attitude at the yaw latched on OFFBOARD entry. This is an
	// active attitude command, not just "stop rotating" - raw body-rate
	// control has no attitude feedback, so if we only zeroed the rate here
	// the vehicle would sit wherever the flip happened to end (e.g. a few
	// degrees short of a full revolution), not necessarily upright. No
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
	sp.thrust_body[2] = _param_frig_thrust.get();
	_vehicle_attitude_setpoint_pub.publish(sp);

	_last_publish_timestamp = now;
	++_setpoints_published;
}

void FlipRig::publish_flip_rates()
{
	// The flip itself: raw body-rate command, bypassing mc_att_control's
	// tilt limiting entirely - identical approach to flip_360.
	const hrt_abstime now = hrt_absolute_time();

	offboard_control_mode_s ocm{};
	ocm.timestamp = now;
	ocm.position = false;
	ocm.velocity = false;
	ocm.acceleration = false;
	ocm.attitude = false;
	ocm.body_rate = true;
	ocm.thrust_and_torque = false;
	ocm.direct_actuator = false;
	_offboard_control_mode_pub.publish(ocm);

	vehicle_rates_setpoint_s sp{};
	sp.timestamp = now;
	sp.roll = math::radians(_param_frig_rate.get());
	sp.pitch = 0.f;
	sp.yaw = 0.f;
	sp.thrust_body[0] = 0.f;
	sp.thrust_body[1] = 0.f;
	sp.thrust_body[2] = _param_frig_thrust.get();
	_vehicle_rates_setpoint_pub.publish(sp);

	_last_publish_timestamp = now;
	++_setpoints_published;
}

void FlipRig::Run()
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
		_current_roll = Eulerf(_current_att).phi();
		_has_attitude = true;
	}

	if (!_param_frig_en.get()) {
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

	if (!_flip_active && !_flip_done) {
		_flip_active = true;
		_flip_start_time = hrt_absolute_time();
		_accumulated_rotation = 0.f;
		_last_roll = _current_roll;
	}

	if (_flip_active) {
		float delta = _current_roll - _last_roll;

		if (delta > M_PI_F) {
			delta -= 2.f * M_PI_F;

		} else if (delta < -M_PI_F) {
			delta += 2.f * M_PI_F;
		}

		_accumulated_rotation += fabsf(delta);
		_last_roll = _current_roll;

		publish_flip_rates();

		const float elapsed_s = static_cast<float>(hrt_absolute_time() - _flip_start_time) * 1e-6f;
		const bool done = (_accumulated_rotation >= 2.f * M_PI_F * 0.95f) || (elapsed_s >= _param_frig_duration.get());

		if (done) {
			_flip_active = false;
			_flip_done = true;
		}

		return;
	}

	// _flip_done: hold level at the latched yaw, and stay there - this mode
	// never flips again on its own while OFFBOARD stays engaged.
	publish_level_hold();
}

int FlipRig::task_spawn(int argc, char *argv[])
{
	FlipRig *instance = new FlipRig();

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

int FlipRig::print_status()
{
	PX4_INFO("enabled=%d rate=%.0f deg/s thrust=%.2f duration=%.2f s",
		 (int)_param_frig_en.get(),
		 (double)_param_frig_rate.get(),
		 (double)_param_frig_thrust.get(),
		 (double)_param_frig_duration.get());

	PX4_INFO("tx=%" PRIu64 " early_disabled=%" PRIu64 " early_no_attitude=%" PRIu64
		 " offboard=%d flip_active=%d flip_done=%d rot=%.0f deg",
		 _setpoints_published, _early_return_disabled, _early_return_no_attitude,
		 (int)_was_offboard, (int)_flip_active, (int)_flip_done,
		 (double)math::degrees(_accumulated_rotation));

	if (_was_offboard) {
		PX4_INFO("hold_yaw=%.3f rad", (double)_hold_yaw);
	}

	return 0;
}

int FlipRig::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int FlipRig::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
`flip_rig` is a standalone, manually-started single-shot 360 degree flip
controller for a fixture that physically prevents translation (e.g. a
3-axis gimbal/gyroscope test rig). Unlike `flip_360`, it never subscribes to
or publishes a position setpoint of any kind - entry, the flip, and the
post-flip hold are all attitude/body-rate only, so it needs nothing more
than a valid attitude estimate (no GPS, no optical flow, no motion capture).
Start it directly with `flip_rig start` from the shell, and do not run it at
the same time as another OFFBOARD-publishing module; avoiding that conflict
is the operator's responsibility.

Once it has a valid attitude estimate, it continuously streams a live
attitude-tracking setpoint (required for commander to admit the OFFBOARD
switch at all). The instant OFFBOARD is engaged, it latches the vehicle's
current yaw and executes exactly one 360 degree roll via raw body-rate
setpoints (FRIG_RATE deg/s, FRIG_THRUST held thrust, capped at
FRIG_DURATION seconds as a safety cutoff) - then commands a level attitude
at the latched yaw and stays there. It does NOT return to, or hold, any
position. It does not flip again on its own; leaving and re-entering
OFFBOARD re-latches the yaw and arms a fresh single flip.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("flip_rig", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int flip_rig_main(int argc, char *argv[])
{
	return ModuleBase::main(FlipRig::desc, argc, argv);
}
