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

#include <gtest/gtest.h>

#include "AttitudeRampMath.hpp"

using namespace hold_rig_att;
using matrix::Eulerf;
using matrix::Quatf;
using matrix::Vector3f;

// Runs the ramp to convergence the way the module does - fixed timestep, fixed
// rate - and reports what happened on the way. Everything the ramp must
// guarantee is a property of this trace rather than of any single step.
struct RampTrace {
	int steps{0};
	float max_step_rad{0.f};        // largest single advance actually taken
	float final_remaining{0.f};
	float final_error_to_target{0.f};
	bool monotonic{true};           // remaining never grew
	bool converged{false};
};

static RampTrace run_ramp(const Quatf &start, const Quatf &target, float rate_deg_s, float dt,
			  int max_steps = 20000)
{
	RampTrace t;
	Quatf leader = start;
	float previous_remaining = angle_between(start, target) + 1.f;
	const float max_step = math::radians(rate_deg_s) * dt;

	for (int i = 0; i < max_steps; ++i) {
		float remaining = 0.f;
		const Quatf next = ramp_step(leader, target, max_step, remaining);

		if (remaining > previous_remaining + 1e-5f) {
			t.monotonic = false;
		}

		previous_remaining = remaining;

		const float step_taken = angle_between(leader, next);

		if (step_taken > t.max_step_rad) {
			t.max_step_rad = step_taken;
		}

		leader = next;
		++t.steps;

		if (remaining == 0.f) {
			t.converged = true;
			break;
		}
	}

	t.final_remaining = previous_remaining;
	t.final_error_to_target = angle_between(leader, target);
	return t;
}

// ---------------------------------------------------------------------------
// target_relative: the two parameterizations
// ---------------------------------------------------------------------------

TEST(AttitudeRampMath, tiltDirectionGivesExactTilt)
{
	// The headline property of mode 0: HRATT_TILT is the tilt, at every
	// direction. This is what mode 1 fails to do above ~60 degrees.
	for (const float tilt : {0.f, 15.f, 30.f, 60.f, 90.f, 120.f, 179.f, 180.f}) {
		for (const float dir : {0.f, 45.f, 90.f, 137.f, 180.f, 270.f, 315.f}) {
			const Quatf q = target_relative(TargetMode::TiltDirection, tilt, dir, 0.f, 0.f);
			EXPECT_NEAR(math::degrees(tilt_of(q)), tilt, 1e-2f)
					<< "tilt=" << tilt << " dir=" << dir;
		}
	}
}

TEST(AttitudeRampMath, tiltDirectionRoundTripsThroughDecompose)
{
	for (const float tilt : {10.f, 45.f, 90.f, 135.f, 170.f}) {
		for (const float dir : {0.f, 30.f, 90.f, 200.f, 330.f}) {
			const Quatf q = target_relative(TargetMode::TiltDirection, tilt, dir, 0.f, 0.f);

			float tilt_out = 0.f;
			float dir_out = 0.f;
			float dyaw_out = 0.f;
			decompose(q, tilt_out, dir_out, dyaw_out);

			EXPECT_NEAR(tilt_out, tilt, 1e-2f) << "tilt=" << tilt << " dir=" << dir;
			EXPECT_NEAR(dir_out, dir, 1e-1f) << "tilt=" << tilt << " dir=" << dir;
			// A pure tilt has no delta-yaw component at all - that is
			// exactly why mode 0 does not lose anything to the yaw
			// channel.
			EXPECT_NEAR(dyaw_out, 0.f, 1e-1f) << "tilt=" << tilt << " dir=" << dir;
		}
	}
}

TEST(AttitudeRampMath, pureRollAndPitchMatchEulerAtSmallAngles)
{
	// Where the two modes agree, they must agree exactly, or the mode-0
	// equivalent printed by the diagnostics would be misleading.
	const Quatf tilt_roll = target_relative(TargetMode::TiltDirection, 30.f, 0.f, 0.f, 0.f);
	const Quatf euler_roll = target_relative(TargetMode::EulerRollPitch, 0.f, 0.f, 30.f, 0.f);
	EXPECT_LT(math::degrees(angle_between(tilt_roll, euler_roll)), 1e-2f);

	const Quatf tilt_pitch = target_relative(TargetMode::TiltDirection, 30.f, 90.f, 0.f, 0.f);
	const Quatf euler_pitch = target_relative(TargetMode::EulerRollPitch, 0.f, 0.f, 0.f, 30.f);
	EXPECT_LT(math::degrees(angle_between(tilt_pitch, euler_pitch)), 1e-2f);
}

// ---------------------------------------------------------------------------
// The Euler degeneracy. These tests PIN the surprising behaviour so that a
// future reader finds it documented and asserted rather than mistaking it for
// a bug. See HRATT_MODE.
// ---------------------------------------------------------------------------

TEST(AttitudeRampMath, atRoll90PitchNoLongerChangesTilt)
{
	// body_z is [0, -1, 0] for EVERY pitch value once roll is 90 degrees,
	// because Z-Y-X applies the pitch about the already-rolled axis and that
	// axis is now vertical.
	const Quatf reference = target_relative(TargetMode::EulerRollPitch, 0.f, 0.f, 90.f, 0.f);
	const Vector3f reference_bz = reference.dcm_z();

	EXPECT_NEAR(reference_bz(0), 0.f, 1e-5f);
	EXPECT_NEAR(reference_bz(1), -1.f, 1e-5f);
	EXPECT_NEAR(reference_bz(2), 0.f, 1e-5f);

	for (const float pitch : {0.f, 15.f, 30.f, 60.f, 90.f, -45.f}) {
		const Quatf q = target_relative(TargetMode::EulerRollPitch, 0.f, 0.f, 90.f, pitch);
		const Vector3f bz = q.dcm_z();

		EXPECT_NEAR(bz(0), reference_bz(0), 1e-5f) << "pitch=" << pitch;
		EXPECT_NEAR(bz(1), reference_bz(1), 1e-5f) << "pitch=" << pitch;
		EXPECT_NEAR(bz(2), reference_bz(2), 1e-5f) << "pitch=" << pitch;

		// Tilt is pinned at 90 regardless.
		EXPECT_NEAR(math::degrees(tilt_of(q)), 90.f, 1e-2f) << "pitch=" << pitch;
	}
}

TEST(AttitudeRampMath, atRoll90TheWholePitchCommandBecomesDeltaYaw)
{
	// The pitch is not discarded - it lands in the delta-yaw channel, where
	// AttitudeControl::update() scales it by MC_YAW_WEIGHT and executes it on
	// the weakest axis. The magnitude transfers one-for-one.
	for (const float pitch : {15.f, 30.f, 60.f, 90.f}) {
		const Quatf q = target_relative(TargetMode::EulerRollPitch, 0.f, 0.f, 90.f, pitch);

		float tilt_out = 0.f;
		float dir_out = 0.f;
		float dyaw_out = 0.f;
		decompose(q, tilt_out, dir_out, dyaw_out);

		EXPECT_NEAR(tilt_out, 90.f, 1e-2f) << "pitch=" << pitch;
		EXPECT_NEAR(fabsf(dyaw_out), pitch, 1e-1f) << "pitch=" << pitch;
	}
}

TEST(AttitudeRampMath, roll90Pitch30ReportsItsModeZeroEquivalent)
{
	// The exact command that motivated the module. The diagnostics promise
	// this translation; assert it.
	const Quatf q = target_relative(TargetMode::EulerRollPitch, 0.f, 0.f, 90.f, 30.f);

	float tilt_out = 0.f;
	float dir_out = 0.f;
	float dyaw_out = 0.f;
	decompose(q, tilt_out, dir_out, dyaw_out);

	EXPECT_NEAR(tilt_out, 90.f, 1e-2f);
	EXPECT_NEAR(dir_out, 0.f, 1e-1f);
	EXPECT_NEAR(dyaw_out, -30.f, 1e-1f);
}

// ---------------------------------------------------------------------------
// The ramp
// ---------------------------------------------------------------------------

TEST(AttitudeRampMath, rampConvergesMonotonicallyAcrossTheEnvelope)
{
	const Quatf level(1.f, 0.f, 0.f, 0.f);
	const float rate = 45.f;
	const float dt = 0.01f;   // 100 Hz, the HRATT_PUB_HZ default

	for (const float tilt : {0.f, 15.f, 30.f, 60.f, 90.f, 120.f, 150.f, 179.f, 180.f}) {
		for (const float dir : {0.f, 45.f, 90.f, 213.f}) {
			const Quatf target = target_relative(TargetMode::TiltDirection, tilt, dir, 0.f, 0.f);
			const RampTrace t = run_ramp(level, target, rate, dt);

			EXPECT_TRUE(t.monotonic) << "tilt=" << tilt << " dir=" << dir;
			EXPECT_TRUE(t.converged) << "tilt=" << tilt << " dir=" << dir;
			// Arrives ON the target, not near it.
			EXPECT_LT(math::degrees(t.final_error_to_target), 0.05f)
					<< "tilt=" << tilt << " dir=" << dir;
			// And never takes a step larger than the rate allows, which
			// is the property that keeps the controller out of
			// saturation.
			EXPECT_LE(t.max_step_rad, math::radians(rate) * dt + 1e-5f)
					<< "tilt=" << tilt << " dir=" << dir;
		}
	}
}

TEST(AttitudeRampMath, rampNeverExceedsTheCommandedRate)
{
	// The whole safety argument rests on this. A step larger than rate*dt
	// would reintroduce exactly the instantaneous error the ramp exists to
	// remove.
	const Quatf level(1.f, 0.f, 0.f, 0.f);
	const Quatf target = target_relative(TargetMode::TiltDirection, 150.f, 37.f, 0.f, 0.f);

	for (const float rate : {5.f, 20.f, 45.f, 120.f}) {
		for (const float dt : {0.004f, 0.01f, 0.04f}) {
			const RampTrace t = run_ramp(level, target, rate, dt);
			EXPECT_LE(t.max_step_rad, math::radians(rate) * dt + 1e-5f)
					<< "rate=" << rate << " dt=" << dt;
			EXPECT_TRUE(t.converged) << "rate=" << rate << " dt=" << dt;
		}
	}
}

TEST(AttitudeRampMath, rampTakesTheShortWayRound)
{
	// canonicalize() inside ramp_step is what guarantees this. Without it,
	// AxisAngle's 2*atan2(|imag|, w) would return the reflex angle for w < 0
	// and the leader would travel the long way - the failure the module's
	// comments warn about.
	const Quatf level(1.f, 0.f, 0.f, 0.f);

	for (const float tilt : {170.f, 179.f}) {
		const Quatf target = target_relative(TargetMode::TiltDirection, tilt, 0.f, 0.f, 0.f);
		const RampTrace t = run_ramp(level, target, 45.f, 0.01f);

		// Distance actually travelled must match the shortest arc, not
		// 360 minus it.
		const float travelled_deg = static_cast<float>(t.steps) * 45.f * 0.01f;
		EXPECT_NEAR(travelled_deg, tilt, 2.f) << "tilt=" << tilt;
	}
}

TEST(AttitudeRampMath, rampArrivesAtA180DegreeTarget)
{
	// The case the plan flagged as the only genuine singularity risk. It is
	// not singular in this implementation: at w = 0 the axis comes from
	// q_err.imag(), which is a unit vector and is exactly the rotation axis.
	// The travel DIRECTION is float-noise-determined, which the module warns
	// about, but arrival is not in doubt.
	const Quatf level(1.f, 0.f, 0.f, 0.f);
	const Quatf target = target_relative(TargetMode::TiltDirection, 180.f, 0.f, 0.f, 0.f);

	EXPECT_NEAR(math::degrees(angle_between(level, target)), 180.f, 1e-2f);

	const RampTrace t = run_ramp(level, target, 45.f, 0.01f);
	EXPECT_TRUE(t.converged);
	EXPECT_TRUE(t.monotonic);
	EXPECT_LT(math::degrees(t.final_error_to_target), 0.05f);
}

TEST(AttitudeRampMath, alreadyAtTargetIsAnImmediateNoOp)
{
	const Quatf target = target_relative(TargetMode::TiltDirection, 63.f, 21.f, 0.f, 0.f);

	float remaining = 1.f;
	const Quatf next = ramp_step(target, target, math::radians(45.f) * 0.01f, remaining);

	EXPECT_EQ(remaining, 0.f);
	EXPECT_LT(math::degrees(angle_between(next, target)), 1e-3f);
}

TEST(AttitudeRampMath, rampDoesNotOvershootOnTheFinalStep)
{
	// The final step is clamped to the remaining distance. Without that clamp
	// the leader would overshoot by up to one step and then walk back,
	// leaving the held attitude jittering forever.
	const Quatf level(1.f, 0.f, 0.f, 0.f);
	// A target deliberately not a whole number of steps away.
	const Quatf target = target_relative(TargetMode::TiltDirection, 33.33f, 0.f, 0.f, 0.f);

	Quatf leader = level;
	const float max_step = math::radians(45.f) * 0.01f;
	float previous_remaining = 1e9f;

	for (int i = 0; i < 5000; ++i) {
		float remaining = 0.f;
		leader = ramp_step(leader, target, max_step, remaining);
		// remaining must never increase, which it would if a step
		// overshot and the next one had to come back.
		EXPECT_LE(remaining, previous_remaining + 1e-6f) << "step " << i;
		previous_remaining = remaining;

		if (remaining == 0.f) {
			break;
		}
	}

	EXPECT_LT(math::degrees(angle_between(leader, target)), 0.05f);
}

TEST(AttitudeRampMath, rampChasesAMovingTarget)
{
	// What makes HRATT_LIVE safe: the arc is recomputed every step, so a
	// target that moves mid-ramp is followed rather than lost, and still at
	// no more than the commanded rate.
	const Quatf level(1.f, 0.f, 0.f, 0.f);
	Quatf leader = level;
	const float max_step = math::radians(45.f) * 0.01f;

	Quatf target = target_relative(TargetMode::TiltDirection, 30.f, 0.f, 0.f, 0.f);

	for (int i = 0; i < 2000; ++i) {
		if (i == 200) {
			// Retarget mid-flight, a long way away.
			target = target_relative(TargetMode::TiltDirection, 120.f, 180.f, 0.f, 0.f);
		}

		float remaining = 0.f;
		const Quatf next = ramp_step(leader, target, max_step, remaining);
		EXPECT_LE(angle_between(leader, next), max_step + 1e-5f) << "step " << i;
		leader = next;
	}

	EXPECT_LT(math::degrees(angle_between(leader, target)), 0.05f);
}

// ---------------------------------------------------------------------------
// Yaw factorization - the identity build_target() relies on
// ---------------------------------------------------------------------------

TEST(AttitudeRampMath, yawFactorsOutOfBothModes)
{
	// build_target() computes q_yaw * target_relative(). That is only correct
	// if Quatf(Eulerf(r, p, y)) == Quatf(Eulerf(0, 0, y)) * Quatf(Eulerf(r, p, 0)),
	// which is the property that lets the yaw latch and the target shape be
	// reasoned about separately.
	for (const float yaw : {-170.f, -90.f, 0.f, 45.f, 130.f}) {
		for (const float roll : {-60.f, 0.f, 30.f, 90.f}) {
			for (const float pitch : {-45.f, 0.f, 25.f}) {
				const Quatf direct(Eulerf(math::radians(roll), math::radians(pitch), math::radians(yaw)));
				const Quatf factored = Quatf(Eulerf(0.f, 0.f, math::radians(yaw)))
						       * target_relative(TargetMode::EulerRollPitch, 0.f, 0.f, roll, pitch);

				EXPECT_LT(math::degrees(angle_between(direct, factored)), 1e-2f)
						<< "yaw=" << yaw << " roll=" << roll << " pitch=" << pitch;
			}
		}
	}
}

TEST(AttitudeRampMath, tiltIsRelativeToTheNoseNotToNorth)
{
	// Mode 0 applies the tilt on the RIGHT of the yaw quaternion, so a
	// commanded direction of 0 leans about the body x axis whatever the
	// latched yaw is. A left multiplication would make it lean about north
	// instead, which on a rig would silently change the manoeuvre with
	// heading.
	const Quatf q_rel = target_relative(TargetMode::TiltDirection, 40.f, 0.f, 0.f, 0.f);

	for (const float yaw : {0.f, 90.f, 180.f, -75.f}) {
		const Quatf full = Quatf(Eulerf(0.f, 0.f, math::radians(yaw))) * q_rel;

		// Tilt magnitude is yaw-invariant...
		EXPECT_NEAR(math::degrees(tilt_of(full)), 40.f, 1e-2f) << "yaw=" << yaw;

		// ...and in the body frame the lean is still pure roll.
		const Eulerf euler(full);
		EXPECT_NEAR(math::degrees(euler.phi()), 40.f, 1e-2f) << "yaw=" << yaw;
		EXPECT_NEAR(math::degrees(euler.theta()), 0.f, 1e-2f) << "yaw=" << yaw;
	}
}

// ---------------------------------------------------------------------------
// The saturation argument the module is built around
// ---------------------------------------------------------------------------

TEST(AttitudeRampMath, staticStepWouldSaturateButTheRampDoesNot)
{
	// Quantifies the claim in the module docs, using the stock gains:
	// MC_ROLL_P = 4.0, MC_ROLLRATE_MAX = 220 deg/s. The rate demand from
	// AttitudeControl's P-law is 2*sin(Phi/2)*P.
	constexpr float kRollP = 4.f;
	constexpr float kRateMaxDeg = 220.f;

	const Quatf level(1.f, 0.f, 0.f, 0.f);
	const Quatf target = target_relative(TargetMode::EulerRollPitch, 0.f, 0.f, 90.f, 30.f);

	// A static command presents the whole error at once.
	const float phi = angle_between(level, target);
	const float static_demand_deg = math::degrees(2.f * sinf(phi / 2.f) * kRollP);

	EXPECT_NEAR(math::degrees(phi), 93.8f, 0.2f);
	EXPECT_GT(static_demand_deg, kRateMaxDeg);   // saturates - the failure mode

	// 335 deg/s is the demand from the FULL attitude error. The figure after
	// MC_YAW_WEIGHT=0.4 has attenuated the 30 deg delta-yaw component is a
	// slightly lower 326 deg/s. Both are well past the 220 deg/s clamp, so
	// which one is quoted does not change the conclusion - but they are
	// different quantities and worth not conflating.
	EXPECT_NEAR(static_demand_deg, 334.8f, 2.f);

	// The ramp instead presents only one step of error per cycle.
	const float rate = 45.f;
	const float dt = 0.01f;
	const RampTrace t = run_ramp(level, target, rate, dt);
	const float step_demand_deg = math::degrees(2.f * sinf(t.max_step_rad / 2.f) * kRollP);

	EXPECT_LT(step_demand_deg, kRateMaxDeg);
	EXPECT_TRUE(t.converged);
}

// ---------------------------------------------------------------------------
// HRATT_THRUST_K's thrust-vs-tilt boost
// ---------------------------------------------------------------------------

TEST(AttitudeRampMath, zeroThrustGainReproducesTheOldConstantThrust)
{
	// k=0 must be a no-op at every tilt - this is the default, and existing
	// HRATT_THRUST tunings must not change underneath anyone.
	for (const float tilt_deg : {0.f, 30.f, 89.f, 90.f, 150.f}) {
		EXPECT_FLOAT_EQ(compensate_thrust(-0.5f, math::radians(tilt_deg), 0.f), -0.5f)
				<< "tilt=" << tilt_deg;
	}
}

TEST(AttitudeRampMath, thrustGainIsInertAtZeroTilt)
{
	// sin(0) = 0 regardless of k: no boost is ever applied while level,
	// whatever HRATT_THRUST_K is set to.
	for (const float k : {0.1f, 0.5f, 1.f, 2.f}) {
		EXPECT_FLOAT_EQ(compensate_thrust(-0.5f, 0.f, k), -0.5f) << "k=" << k;
	}
}

TEST(AttitudeRampMath, thrustGainScalesSmoothlyWithTilt)
{
	// At 90 degrees sin(tilt) = 1, so k=1 exactly doubles the magnitude - the
	// simplest case to check the formula against by hand.
	EXPECT_NEAR(compensate_thrust(-0.5f, math::radians(90.f), 1.f), -1.f, 1e-5f);

	// Monotonically increasing magnitude from 0 to 90 degrees, for a fixed
	// gain - the whole point of scaling with tilt rather than stepping.
	float previous_mag = 0.f;

	for (const float tilt_deg : {0.f, 15.f, 30.f, 45.f, 60.f, 75.f, 90.f}) {
		const float mag = fabsf(compensate_thrust(-0.5f, math::radians(tilt_deg), 0.5f));
		EXPECT_GE(mag, previous_mag) << "tilt=" << tilt_deg;
		previous_mag = mag;
	}
}

TEST(AttitudeRampMath, thrustGainNeverFlipsSignOrBlowsUp)
{
	// Bounded for any tilt, including past 90 degrees where 1/cos(tilt) would
	// have already diverged - this is the reason sin(tilt) was chosen over
	// the classical lift-compensation curve. Sign always matches the base
	// thrust: this only ever scales magnitude, never redirects it.
	for (const float tilt_deg : {0.f, 45.f, 90.f, 135.f, 179.f}) {
		const float thrust = compensate_thrust(-0.5f, math::radians(tilt_deg), 2.f);
		EXPECT_LT(thrust, 0.f) << "tilt=" << tilt_deg;          // sign preserved
		EXPECT_GE(thrust, -0.5f * 3.f) << "tilt=" << tilt_deg;  // bounded: |1+k*sin| <= 1+k
	}
}
