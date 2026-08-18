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
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

// Rig-only single-shot pitch-flip controller. Identical in structure to
// flip_rig, but rotates about the pitch axis instead of roll - for rigs
// whose free axis is pitch rather than roll. Never touches position/GPS in
// any way - entry, the flip itself, and the post-flip hold are all
// attitude/body-rate setpoints, so it works with nothing more than a valid
// attitude estimate (no GPS, no optical flow, no motion capture needed).
// Built for a mount that physically cannot translate (e.g. a 3-axis
// gimbal/gyroscope rig) - it does NOT attempt to hold or return to any
// position. It is NOT gated by offboard_selector - start it directly with
// `flip_rig_pitch start` from the shell, and do not run it at the same time
// as another OFFBOARD-publishing module; avoiding that conflict is the
// operator's responsibility.
class FlipRigPitch : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static ModuleBase::Descriptor desc;

	FlipRigPitch();
	~FlipRigPitch() override = default;

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
	void publish_flip_rates();

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_rates_setpoint_s> _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};

	uint8_t _nav_state{0};   // NAVIGATION_STATE_MANUAL until the first vehicle_status arrives, i.e. "not OFFBOARD"
	matrix::Quatf _current_att{1.f, 0.f, 0.f, 0.f};
	bool _has_attitude{false};

	// Body-frame pitch rate (rad/s), integrated over time to track accumulated
	// flip rotation. Unlike roll, Euler pitch (theta) is confined to
	// [-pi/2, pi/2] by the ZYX convention and cannot represent a full 360
	// degree rotation by differencing angle samples the way flip_rig's
	// roll-based tracking does - it has to be integrated from the measured
	// body rate instead.
	float _pitch_rate{0.f};
	bool _has_angular_velocity{false};
	hrt_abstime _last_rate_sample_time{0};

	// Latched the moment OFFBOARD is entered; re-latched every time OFFBOARD
	// is re-entered after having left it (leaving OFFBOARD also resets
	// _flip_done, so the next entry flips again exactly once). Only yaw is
	// latched - there is no position to latch, by design.
	bool _was_offboard{false};
	float _hold_yaw{0.f};

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
		(ParamBool<px4::params::FRIGP_EN>) _param_frigp_en,
		(ParamFloat<px4::params::FRIGP_RATE>) _param_frigp_rate,
		(ParamFloat<px4::params::FRIGP_THRUST>) _param_frigp_thrust,
		(ParamFloat<px4::params::FRIGP_DURATION>) _param_frigp_duration,
		(ParamInt<px4::params::FRIGP_PUB_HZ>) _param_frigp_pub_hz
	)
};
