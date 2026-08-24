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

#include "AttitudeRampMath.hpp"

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

// Rig-only controller that ramps the vehicle to an ARBITRARY commanded 3D
// attitude and holds it there, driven through the ATTITUDE setpoint path
// (mc_att_control) rather than raw body rates.
//
// This is the standing-still counterpart to flip_rig_axis. That module rotates
// through a full revolution about a fixed axis; this one travels to one
// orientation and parks on it. Both publish the same two topics and must not be
// run at the same time - see HRATT_EN for why that is not merely untidy.
//
// Two things about "just command the angle and hold it" turn out not to work,
// and between them they account for most of this module:
//
//  1. A STATIC setpoint arrives, but violently. It is a common worry that the
//     shortest-arc resolution in AttitudeControl::update() (eq = 2 *
//     qe.canonical().imag(), bounded to +-180 degrees) makes a large target
//     unreachable. It does not: the rotation from level to any (roll, pitch) is
//     always at most 180 degrees, so the target is always on the near side of
//     the wrap. What bites instead is the MAGNITUDE of the initial error. The
//     rate demand is 2*sin(Phi/2)*MC_ROLL_P, hard-clamped at MC_ROLLRATE_MAX,
//     and with stock gains that clamp is already reached at Phi = 57 degrees -
//     so a static roll-90/pitch-30 command asks for 335 deg/s against a 220
//     deg/s limit and slams in at full authority. Hence ramp_toward_target():
//     the leader travels at HRATT_RATE and the controller only ever sees a
//     small tracking lag. (flip_rig_axis needs a moving setpoint for a
//     different reason - to get PAST the 180 degree wrap at all. Here the
//     wrap is not the problem; saturation is.)
//
//  2. EULER ROLL/PITCH IS THE WRONG PARAMETERIZATION at large angles, which is
//     why HRATT_MODE exists. In intrinsic Z-Y-X, pitch is applied about the
//     already-rolled axis, so at roll = 90 the body z axis is [0, -1, 0] for
//     EVERY pitch value and the pitch command tilts the airframe not at all.
//     It is not discarded - it becomes a rotation about the (now horizontal)
//     body z axis, which AttitudeControl::update() separates out as delta-yaw
//     and scales by MC_YAW_WEIGHT (0.4) because yaw is the weakest axis a
//     multicopter has. Mode 0 (tilt magnitude + direction) names the tilt
//     directly and is degenerate nowhere in 0-180 degrees; mode 1 reproduces
//     the Euler behaviour for anyone who wants it, with startup diagnostics
//     that print the mode-0 equivalent and the delta-yaw share.
//
// Never touches position/GPS in any way - tracking, the ramp and the hold are
// all attitude setpoints, so it works with nothing more than a valid attitude
// estimate (no GPS, no optical flow, no motion capture needed). Built for a
// mount that physically cannot translate (e.g. a 3-axis gimbal/gyroscope rig):
// thrust is a fixed passthrough with no 1/cos(tilt) compensation and no
// altitude loop, so beyond 90 degrees of tilt it drives the airframe into its
// fixture rather than away from it. It does NOT attempt to hold or return to
// any position. It is NOT gated by offboard_selector - start it directly with
// `hold_rig_att start` from the shell.
class HoldRigAtt : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static ModuleBase::Descriptor desc;

	HoldRigAtt();
	~HoldRigAtt() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	void parameters_updated();

	// Startup/parameter-change sanity checks. Each warns rather than
	// silently clamping, because quietly altering a commanded attitude or
	// rate on a physical rig is worse than a noisy log.
	void check_rate_against_limits() const;
	// One axis' share of HRATT_RATE against that axis' own ceilings: the
	// rate clamp (MC_*RATE_MAX) and the attitude P-law ceiling (2 * MC_*_P,
	// since the error vector saturates at 2*sin(phi/2) <= 2). Also reports
	// the steady-state tracking lag the ramp will run at on this axis.
	void check_axis_share(const char *axis_label, float axis_rate_deg,
			      const char *rate_max_param, const char *att_p_param) const;
	void check_ramp_time() const;
	// Warns when HRATT_MODE=1 has been given a command that the Euler
	// parameterization cannot express as tilt, and always logs the
	// equivalent mode-0 (tilt, direction) pair plus the delta-yaw residual.
	void check_euler_degeneracy() const;
	// FD_FAIL_R/FD_FAIL_P vs the Euler roll/pitch of the target - those are
	// the exact quantities the failure detector measures - plus what the
	// resulting trip would actually do given CBRK_FLIGHTTERM.
	void check_failure_detector() const;
	// COM_DISARM_PRFLT / COM_DISARM_LAND, both of which fire on a fixture
	// that never reports a takeoff.
	void check_rig_disarm_params() const;
	void check_thrust_against_tilt() const;
	void check_sibling_modules() const;

	void publish_setpoint(hrt_abstime now, const matrix::Quatf &q_sp);
	void publish_track_current_attitude();
	void publish_hold();
	void ramp_toward_target(hrt_abstime now);

	// Bind the current parameter values to the pure maths in
	// AttitudeRampMath.hpp. All attitude maths lives there so it can be
	// unit-tested without uORB, parameters or timers.
	matrix::Quatf target_relative() const;
	void build_target();
	// True once the vehicle has been armed for longer than
	// COM_LKDOWN_TKO + COM_SPOOLUP_TIME, during which window an attitude
	// failure disarms undeferrably. See HRATT_ARM_GATE.
	bool past_arm_lockdown(hrt_abstime now) const;

	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};

	uint8_t _nav_state{0};   // NAVIGATION_STATE_MANUAL until the first vehicle_status arrives, i.e. "not OFFBOARD"
	matrix::Quatf _current_att{1.f, 0.f, 0.f, 0.f};
	bool _has_attitude{false};
	hrt_abstime _armed_time{0};

	// EKF yaw resets arrive as a bumped counter plus the delta. mc_att_control
	// normally repairs a held setpoint itself via adaptAttitudeSetpoint(), but
	// only `if (v_att.timestamp > _last_attitude_setpoint)` - and because this
	// module republishes every cycle with a fresh timestamp, ours is always
	// the newer one and that repair never runs. So the rotation has to be
	// applied here or the held attitude's yaw silently goes stale.
	uint8_t _quat_reset_counter{0};
	bool _quat_reset_counter_valid{false};

	// Latched the moment OFFBOARD is entered; re-latched every time OFFBOARD
	// is re-entered after having left it. Only yaw is latched - there is no
	// position to latch, by design.
	bool _was_offboard{false};
	float _hold_yaw{0.f};

	// Where the ramp is going. Latched at OFFBOARD entry unless HRATT_LIVE,
	// in which case it is re-evaluated every cycle and the ramp re-engages
	// on a change.
	matrix::Quatf _q_target{1.f, 0.f, 0.f, 0.f};

	// The moving attitude setpoint. Starts exactly at the vehicle's live
	// attitude (zero initial error) and is advanced along the shortest arc
	// toward _q_target by at most HRATT_RATE * dt each cycle. Unlike
	// flip_rig_axis' open-loop leader this one is recomputed against the
	// target every cycle, which is what makes it converge exactly and makes
	// HRATT_LIVE retargeting safe.
	matrix::Quatf _qd_leader{1.f, 0.f, 0.f, 0.f};
	hrt_abstime _last_leader_update_time{0};

	bool _ramp_active{false};
	bool _hold_active{false};
	hrt_abstime _ramp_start_time{0};
	float _remaining{0.f};        // radians still to travel, as of the last ramp step
	float _initial_distance{0.f}; // radians the ramp had to cover when it started, for status

	hrt_abstime _last_publish_timestamp{0};
	uint64_t _setpoints_published{0};
	uint64_t _early_return_disabled{0};
	uint64_t _early_return_no_attitude{0};
	uint64_t _arm_gate_waits{0};

	hrt_abstime _schedule_interval_us{1'000'000 / 50};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::HRATT_EN>) _param_hratt_en,
		(ParamInt<px4::params::HRATT_MODE>) _param_hratt_mode,
		(ParamFloat<px4::params::HRATT_TILT>) _param_hratt_tilt,
		(ParamFloat<px4::params::HRATT_DIR>) _param_hratt_dir,
		(ParamFloat<px4::params::HRATT_ROLL>) _param_hratt_roll,
		(ParamFloat<px4::params::HRATT_PITCH>) _param_hratt_pitch,
		(ParamFloat<px4::params::HRATT_RATE>) _param_hratt_rate,
		(ParamFloat<px4::params::HRATT_THRUST>) _param_hratt_thrust,
		(ParamFloat<px4::params::HRATT_RAMP_T>) _param_hratt_ramp_t,
		(ParamInt<px4::params::HRATT_PUB_HZ>) _param_hratt_pub_hz,
		(ParamBool<px4::params::HRATT_LIVE>) _param_hratt_live,
		(ParamBool<px4::params::HRATT_ARM_GATE>) _param_hratt_arm_gate
	)
};
