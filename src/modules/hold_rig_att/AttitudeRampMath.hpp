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

// Pure attitude maths for hold_rig_att, kept free of uORB, parameters and
// timers so it can be unit-tested directly (AttitudeRampMathTest.cpp). The
// module itself holds no attitude maths of its own beyond calling these.

#include <matrix/matrix/math.hpp>
#include <mathlib/math/Limits.hpp>

namespace hold_rig_att
{

// Below this the leader is considered to have arrived. 1e-4 rad is 0.006 deg -
// far finer than the attitude estimate, so it only ever fires on genuine
// arrival, but coarse enough that float error in the final clamped step cannot
// leave the ramp one cycle short forever.
static constexpr float kArrivedRad = 1e-4f;

enum class TargetMode : int32_t {
	TiltDirection = 0,
	EulerRollPitch = 1,
};

/**
 * The target attitude expressed relative to level at zero yaw - i.e. everything
 * the mode selection decides, and nothing else.
 *
 * Both modes factor cleanly as q_target = q_yaw(latched) * target_relative(),
 * because for intrinsic Z-Y-X
 *
 *   Quatf(Eulerf(r, p, y)) == Quatf(Eulerf(0, 0, y)) * Quatf(Eulerf(r, p, 0))
 *
 * so the yaw latch and the target shape never have to be reasoned about
 * together.
 */
inline matrix::Quatf target_relative(TargetMode mode, float tilt_deg, float dir_deg,
				     float roll_deg, float pitch_deg)
{
	if (mode == TargetMode::EulerRollPitch) {
		// Euler mode, exactly as PX4 means it: intrinsic Z-Y-X, so the
		// pitch is applied about the already-yawed axis and the roll
		// about the already-pitched one. That ordering is the whole
		// reason this mode goes degenerate near 90 degrees - see
		// decompose().
		return matrix::Quatf(matrix::Eulerf(math::radians(roll_deg), math::radians(pitch_deg), 0.f));
	}

	// Tilt + direction mode. The axis is a UNIT vector in the body xy-plane,
	// so the rotation vector axis*tilt has magnitude exactly tilt whatever
	// the direction - which is what keeps the tilt parameter meaning "total
	// degrees of lean" rather than something that varies with direction.
	// Feeding the two components in independently instead
	// (AxisAnglef(tilt, tilt, 0) for a diagonal) would give sqrt(2)*tilt.
	const float dir = math::radians(dir_deg);
	const matrix::Vector3f axis(cosf(dir), sinf(dir), 0.f);
	const matrix::Vector3f rotvec = axis * math::radians(tilt_deg);

	return matrix::Quatf(matrix::AxisAnglef(rotvec(0), rotvec(1), rotvec(2)));
}

/**
 * Total tilt of a relative target, in radians: the angle its body z axis makes
 * with vertical.
 *
 * This is the quantity that governs lift authority, and the one worth logging -
 * as opposed to an Euler readback, which is not injective near pitch = +-90 and
 * so prints a different triple for the same orientation.
 */
inline float tilt_of(const matrix::Quatf &q_rel)
{
	// dcm_z() is the body z axis in the reference frame, so its third
	// component is cos(tilt) and the norm of its xy part is sin(tilt).
	//
	// Using atan2(sin, cos) rather than the more obvious acos(cos) is
	// deliberate. acos' derivative is unbounded at +-1, so acosf(cos_tilt)
	// loses precision exactly where this module spends its time: measured
	// error reaches 0.03 deg at a tilt of 180 degrees, against 1e-5 deg for
	// atan2. It only feeds diagnostics, so 0.03 deg would not have mattered -
	// but the two-argument form is no more expensive and is well-conditioned
	// over the whole 0-180 range, so there is nothing to trade off.
	const matrix::Vector3f e_z_d = q_rel.dcm_z();
	const float sin_tilt = sqrtf(e_z_d(0) * e_z_d(0) + e_z_d(1) * e_z_d(1));

	return atan2f(sin_tilt, e_z_d(2));
}

/**
 * Decompose a relative target the way AttitudeControl::update() will: into a
 * tilt magnitude, the body-xy direction of the tilt axis, and the residual
 * delta-yaw that gets scaled by MC_YAW_WEIGHT.
 *
 * The point of mirroring the controller's own split rather than approximating it
 * separately is that the delta-yaw figure is then exactly the part of a command
 * that will be executed at reduced weight on the weakest axis. For a mode-1
 * command of roll 90 / pitch 30 this reports tilt 90, dir 0, delta-yaw -30 -
 * i.e. the pitch has entirely left the tilt channel.
 *
 * All outputs in degrees; dir is wrapped to [0, 360).
 */
inline void decompose(const matrix::Quatf &q_rel, float &tilt_deg, float &dir_deg, float &dyaw_deg)
{
	const matrix::Vector3f e_z(0.f, 0.f, 1.f);      // level reference
	const matrix::Vector3f e_z_d = q_rel.dcm_z();   // where the target points body z

	tilt_deg = math::degrees(tilt_of(q_rel));

	// The reduced (tilt-only) rotation: shortest arc from level's body z onto
	// the target's. Its axis is e_z x e_z_d, which lies in the xy-plane, and
	// that direction is exactly the tilt-direction convention.
	const matrix::Quatf qd_red(e_z, e_z_d);

	if (sqrtf(qd_red(1) * qd_red(1) + qd_red(2) * qd_red(2)) > 1e-6f) {
		dir_deg = math::degrees(atan2f(qd_red(2), qd_red(1)));

		if (dir_deg < 0.f) {
			dir_deg += 360.f;
		}

	} else {
		// Either no tilt at all, or tilt is exactly 180 degrees, where the
		// axis is genuinely undetermined by the two vectors alone.
		dir_deg = 0.f;
	}

	// Whatever is left once the tilt is accounted for can only be a rotation
	// about the shared body z axis - that is the delta-yaw channel, the one
	// scaled by MC_YAW_WEIGHT. This is where a mode-1 pitch command goes at
	// large roll.
	matrix::Quatf qd_dyaw = qd_red.inversed() * q_rel;
	qd_dyaw.canonicalize();
	dyaw_deg = math::degrees(2.f * atan2f(qd_dyaw(3), qd_dyaw(0)));
}

/**
 * Advance the leader setpoint along the shortest arc toward the target by at
 * most max_step_rad, and report how far there was still to go BEFORE the step.
 *
 * Recomputing the arc every cycle rather than latching an axis once (as
 * flip_rig_axis does for its open-loop revolution) costs one quaternion product
 * and buys two things: the leader converges exactly on the target instead of
 * accumulating integration error, and a target that moves under it is simply
 * chased rather than lost - which is what makes live retargeting safe.
 *
 * @param leader        current leader attitude
 * @param target        attitude to converge on
 * @param max_step_rad  ceiling on this step, normally rate * dt
 * @param remaining_out angle still to travel before this step, radians in [0, pi]
 * @return the advanced leader, normalized
 */
inline matrix::Quatf ramp_step(const matrix::Quatf &leader, const matrix::Quatf &target,
			       float max_step_rad, float &remaining_out)
{
	// Body-frame error of the LEADER, not of the vehicle: this is a
	// feed-forward trajectory generator and deliberately never reads the
	// vehicle's attitude back once the ramp has started. That is what keeps
	// the controller's own shortest-arc error down to the small tracking lag.
	matrix::Quatf q_err = leader.inversed() * target;

	// canonicalize() before AxisAnglef is load-bearing. AxisAngle's
	// quaternion constructor computes 2*atan2(|imag|, w), which for w < 0
	// returns an angle in [pi, 2pi) - i.e. the LONG way round. Forcing
	// w >= 0 first bounds it to [0, pi] and guarantees the shortest arc.
	q_err.canonicalize();

	const matrix::AxisAnglef aa(q_err);
	const float remaining = aa.norm();
	remaining_out = remaining;

	if (remaining < kArrivedRad) {
		// Arrived. Snap to the target so the held quaternion is exact
		// rather than a float error away from it.
		remaining_out = 0.f;
		matrix::Quatf arrived = target;
		arrived.normalize();
		return arrived;
	}

	// Never step past the target. Without this the leader overshoots by up to
	// one whole step (0.45 deg at 45 deg/s and 100 Hz) and then walks back,
	// leaving the held attitude jittering by that amount forever.
	const float dtheta = math::min(max_step_rad, remaining);
	const matrix::Vector3f rotvec = (aa / remaining) * dtheta;

	matrix::Quatf advanced = leader * matrix::Quatf(matrix::AxisAnglef(rotvec(0), rotvec(1), rotvec(2)));
	advanced.normalize();

	return advanced;
}

/**
 * Angle in radians between two attitudes, along the shortest arc. Used to
 * decide whether a live retarget has actually moved the target enough to be
 * worth re-engaging the ramp for.
 */
inline float angle_between(const matrix::Quatf &a, const matrix::Quatf &b)
{
	matrix::Quatf delta = a.inversed() * b;
	delta.canonicalize();
	return matrix::AxisAnglef(delta).norm();
}

/**
 * Scale a base thrust magnitude up as tilt grows, to buy back floor headroom
 * on whichever motor a large static roll/pitch command drives down (see
 * HRATT_THRUST_K docs for the allocator argument this is built on).
 *
 * sin(tilt) rather than the classical 1/cos(tilt) lift-compensation curve is
 * deliberate: there is no free-flight weight to support here (this is a rig,
 * not a hover), so there is no first-principles target this "should" hit -
 * it is an empirical headroom knob, and sin(tilt) is bounded in [0, 1] for
 * any tilt with no singularity to guard against, where 1/cos(tilt) diverges
 * approaching 90 degrees and would need an arbitrary clamp angle to stay
 * finite. k = 0 reproduces the old constant-thrust behaviour exactly.
 *
 * @param base_thrust HRATT_THRUST, i.e. the untilted thrust_body[2] value
 * @param tilt_rad    current commanded tilt, e.g. tilt_of(leader or target)
 * @param k           HRATT_THRUST_K, dimensionless compensation gain
 * @return thrust_body[2] to publish this cycle
 */
inline float compensate_thrust(float base_thrust, float tilt_rad, float k)
{
	return base_thrust * (1.f + k * sinf(tilt_rad));
}

} // namespace hold_rig_att
