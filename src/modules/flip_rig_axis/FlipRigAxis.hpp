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

#pragma once

#include <drivers/drv_hrt.h>
#include <stdint.h>

#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

// Rig-only single-shot 360 degree flip controller about an ARBITRARY axis in
// the body xy-plane, driven through the ATTITUDE setpoint path
// (mc_att_control) rather than raw body rates.
//
// This generalises flip_rig_pitch_att, which is hard-wired to the pitch axis.
// The manoeuvre and every safety property are identical; the only difference
// is that the rotation axis is the free parameter FRIGAX_AXIS, so one module
// covers roll flips, pitch flips and any diagonal between them. Both modules
// publish the same topics and must not be run at the same time.
//
// See publish_flip_attitude() for why a *moving* setpoint is required: a
// static attitude target cannot complete a 360 degree rotation, because
// AttitudeControl::update() always resolves to the shortest-arc error
// (qe.canonical(), bounded to +-180 degrees), so it would only ever chase the
// target halfway around before reversing.
//
// Never touches position/GPS in any way - entry, the flip itself, and the
// post-flip hold are all attitude setpoints, so it works with nothing more
// than a valid attitude estimate (no GPS, no optical flow, no motion capture
// needed). Built for a mount that physically cannot translate (e.g. a 3-axis
// gimbal/gyroscope rig): with thrust welded to the airframe and no altitude
// loop, the net vertical impulse over a revolution is zero and gravity is
// uncompensated throughout. It does NOT attempt to hold or return to any
// position. It is NOT gated by offboard_selector - start it directly with
// `flip_rig_axis start` from the shell, and do not run it at the same time as
// another OFFBOARD-publishing module; avoiding that conflict is the
// operator's responsibility.
class FlipRigAxis : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static ModuleBase::Descriptor desc;

	FlipRigAxis();
	~FlipRigAxis() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	void parameters_updated();

	// Startup/parameter-change sanity checks. Each warns rather than
	// silently clamping, because quietly altering a commanded rate or
	// duration on a physical rig is worse than a noisy log.
	void check_rate_against_limits() const;
	void check_duration_against_rate() const;
	// One axis' share of FRIGAX_RATE against that axis' own ceilings:
	// the rate clamp (MC_*RATE_MAX) and the attitude P-law ceiling
	// (2 * MC_*_P, since the error vector saturates at 2*sin(phi/2) <= 2).
	void check_axis_share(const char *axis_label, float axis_rate_deg,
			      const char *rate_max_param, const char *att_p_param) const;

	void publish_track_current_attitude();
	void publish_level_hold();
	void publish_attitude_hold();
	void publish_flip_attitude(hrt_abstime now);

	// Unit vector of the flip axis in the body xy-plane, from FRIGAX_AXIS.
	matrix::Vector3f flip_axis() const;

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};

	uint8_t _nav_state{0};   // NAVIGATION_STATE_MANUAL until the first vehicle_status arrives, i.e. "not OFFBOARD"
	matrix::Quatf _current_att{1.f, 0.f, 0.f, 0.f};
	bool _has_attitude{false};

	// Latched the moment OFFBOARD is entered; re-latched every time OFFBOARD
	// is re-entered after having left it (leaving OFFBOARD also resets
	// _flip_done, so the next entry flips again exactly once). Only yaw is
	// latched - there is no position to latch, by design.
	bool _was_offboard{false};
	float _hold_yaw{0.f};

	// The moving attitude setpoint chased during the flip. Starts exactly at
	// the vehicle's live attitude (zero initial error) and is advanced every
	// cycle by a fixed increment about the flip axis, expressed in ITS OWN
	// body frame (right multiplication), regardless of where the real vehicle
	// actually is - it never reads _current_att back once the flip starts.
	// This is what keeps mc_att_control's tracking error small and
	// one-directional for the whole revolution instead of wrapping at +-180
	// degrees.
	matrix::Quatf _qd_leader{1.f, 0.f, 0.f, 0.f};
	hrt_abstime _last_leader_update_time{0};

	// The axis is sampled once, at flip start, and held for the whole
	// revolution. Re-reading FRIGAX_AXIS every cycle would let a mid-flip
	// parameter change tilt the axis under the leader, which breaks the
	// fixed-axis property the whole scheme relies on (increments about a
	// common axis commute; increments about a drifting one do not).
	matrix::Vector3f _flip_axis{0.f, 1.f, 0.f};

	bool _flip_active{false};
	bool _flip_done{false};
	hrt_abstime _flip_start_time{0};
	float _accumulated_rotation{0.f};

	// How far this run is meant to travel, and what to do on arrival. Both
	// are latched at flip start from FRIGAX_HOLD_ANG, alongside _flip_axis
	// and for the same reason: switching mode or moving the target from
	// under a manoeuvre already in progress is never what the operator
	// meant. 0 in the parameter means "full revolution, then level hold";
	// anything else means "ramp to that angle, stop exactly on it, and hold
	// that attitude until OFFBOARD is left".
	float _target_rotation{2.f * M_PI_F};
	bool _hold_at_angle{false};

	hrt_abstime _last_publish_timestamp{0};
	uint64_t _setpoints_published{0};
	uint64_t _early_return_disabled{0};
	uint64_t _early_return_no_attitude{0};

	hrt_abstime _schedule_interval_us{1'000'000 / 50};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FRIGAX_EN>) _param_frigax_en,
		(ParamFloat<px4::params::FRIGAX_AXIS>) _param_frigax_axis,
		(ParamFloat<px4::params::FRIGAX_RATE>) _param_frigax_rate,
		(ParamFloat<px4::params::FRIGAX_THRUST>) _param_frigax_thrust,
		(ParamFloat<px4::params::FRIGAX_DURATION>) _param_frigax_duration,
		(ParamInt<px4::params::FRIGAX_PUB_HZ>) _param_frigax_pub_hz,
		(ParamFloat<px4::params::FRIGAX_HOLD_ANG>) _param_frigax_hold_ang
	)
};
