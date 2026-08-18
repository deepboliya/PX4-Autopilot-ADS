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

// Rig-only single-shot pitch-flip controller, driven through the ATTITUDE
// setpoint path (mc_att_control) instead of raw body rates. Identical intent
// to flip_rig_pitch, but the flip itself publishes a continuously-advancing
// "leader" attitude setpoint instead of a vehicle_rates_setpoint - see
// publish_flip_attitude() for why a *moving* setpoint is required: a static
// attitude target cannot complete a 360 degree rotation, because
// AttitudeControl::update() always resolves to the shortest-arc error
// (qe.canonical(), bounded to +-180 degrees), so it would only ever chase the
// target halfway around before reversing. Never touches position/GPS in any
// way - entry, the flip itself, and the post-flip hold are all attitude
// setpoints, so it works with nothing more than a valid attitude estimate
// (no GPS, no optical flow, no motion capture needed). Built for a mount
// that physically cannot translate (e.g. a 3-axis gimbal/gyroscope rig) - it
// does NOT attempt to hold or return to any position. It is NOT gated by
// offboard_selector - start it directly with `flip_rig_pitch_att start` from
// the shell, and do not run it at the same time as another
// OFFBOARD-publishing module; avoiding that conflict is the operator's
// responsibility.
class FlipRigPitchAtt : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static ModuleBase::Descriptor desc;

	FlipRigPitchAtt();
	~FlipRigPitchAtt() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	void parameters_updated();
	void publish_track_current_attitude();
	void publish_level_hold();
	void publish_flip_attitude(hrt_abstime now);

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
	// cycle by a fixed pitch increment in ITS OWN body frame (right
	// multiplication), regardless of where the real vehicle actually is - it
	// never reads _current_att back once the flip starts. This is what keeps
	// mc_att_control's tracking error small and one-directional for the
	// whole revolution instead of wrapping at +-180 degrees.
	matrix::Quatf _qd_leader{1.f, 0.f, 0.f, 0.f};
	hrt_abstime _last_leader_update_time{0};

	bool _flip_active{false};
	bool _flip_done{false};
	hrt_abstime _flip_start_time{0};
	float _accumulated_rotation{0.f};

	hrt_abstime _last_publish_timestamp{0};
	uint64_t _setpoints_published{0};
	uint64_t _early_return_disabled{0};
	uint64_t _early_return_no_attitude{0};

	hrt_abstime _schedule_interval_us{1'000'000 / 50};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::FRIGPA_EN>) _param_frigpa_en,
		(ParamFloat<px4::params::FRIGPA_RATE>) _param_frigpa_rate,
		(ParamFloat<px4::params::FRIGPA_THRUST>) _param_frigpa_thrust,
		(ParamFloat<px4::params::FRIGPA_DURATION>) _param_frigpa_duration,
		(ParamInt<px4::params::FRIGPA_PUB_HZ>) _param_frigpa_pub_hz
	)
};
