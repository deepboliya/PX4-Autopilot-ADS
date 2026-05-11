/****************************************************************************
 *
 *   Custom Multicopter Position Controller - flat single-file replica of
 *   mc_pos_control's core PositionControl + ControlMath math.
 *
 *   The controller has two loops: a P loop on position error producing a
 *   velocity setpoint, and a PID loop on velocity error producing an
 *   acceleration setpoint. The acceleration setpoint is converted to a
 *   thrust vector and split into thrust direction (attitude) and thrust
 *   magnitude.
 *
 ****************************************************************************/

#include "custom_mc_pos_control.hpp"

#include <float.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <px4_platform_common/defines.h>

using namespace matrix;

ModuleBase::Descriptor CustomPositionControl::desc{task_spawn, custom_command, print_usage};

// ============================================================================
// Inlined helpers (formerly ControlMath:: namespace)
// ============================================================================

namespace
{

// Add `addition` into `setpoint` while preserving NaN semantics:
//   - both finite -> add
//   - setpoint NaN, addition finite -> overwrite
//   - addition NaN -> leave setpoint unchanged
inline void addIfNotNan(float &setpoint, float addition)
{
	if (PX4_ISFINITE(setpoint) && PX4_ISFINITE(addition)) {
		setpoint += addition;

	} else if (!PX4_ISFINITE(setpoint)) {
		setpoint = addition;
	}
}

inline void addIfNotNanVector3f(Vector3f &setpoint, const Vector3f &addition)
{
	for (int i = 0; i < 3; i++) {
		addIfNotNan(setpoint(i), addition(i));
	}
}

inline void setZeroIfNanVector3f(Vector3f &vector)
{
	addIfNotNanVector3f(vector, Vector3f());
}

// Limit the tilt angle between two unit vectors. The body_unit vector is rotated
// in the plane formed by world_unit and itself until the angle between them is
// at most max_angle.
inline void limitTilt(Vector3f &body_unit, const Vector3f &world_unit, float max_angle)
{
	const float dot_product_unit = body_unit.dot(world_unit);
	float angle = acosf(dot_product_unit);
	angle = math::min(angle, max_angle);
	Vector3f rejection = body_unit - (dot_product_unit * world_unit);

	if (rejection.norm_squared() < FLT_EPSILON) {
		// Vectors exactly parallel - pick any orthogonal direction.
		rejection(0) = 1.f;
	}

	body_unit = cosf(angle) * world_unit + sinf(angle) * rejection.unit();
}

// Convert a world-frame body_z vector and yaw setpoint to a quaternion attitude
// setpoint. Mirrors ControlMath::bodyzToAttitude.
inline void bodyzToAttitude(Vector3f body_z, float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
	if (body_z.norm_squared() < FLT_EPSILON) {
		// No direction - default to "up" so the rotation matrix stays valid.
		body_z(2) = 1.f;
	}

	body_z.normalize();

	// Desired yaw direction rotated by pi/2 in the XY plane.
	const Vector3f y_C{-sinf(yaw_sp), cosf(yaw_sp), 0.f};

	// body_x is the axis orthogonal to both body_z and the yaw direction.
	Vector3f body_x = y_C % body_z;

	if (body_z(2) < 0.f) {
		// Keep nose to front when inverted.
		body_x = -body_x;
	}

	if (fabsf(body_z(2)) < 0.000001f) {
		// Desired thrust is in XY plane - put X downward to keep matrix valid;
		// yaw will be ignored by the attitude controller in this regime.
		body_x.zero();
		body_x(2) = 1.f;
	}

	body_x.normalize();

	const Vector3f body_y = body_z % body_x;

	Dcmf R_sp;

	for (int i = 0; i < 3; i++) {
		R_sp(i, 0) = body_x(i);
		R_sp(i, 1) = body_y(i);
		R_sp(i, 2) = body_z(i);
	}

	const Quatf q_sp{R_sp};
	q_sp.copyTo(att_sp.q_d);
}

// Convert thrust vector + yaw to attitude setpoint. Mirrors ControlMath::thrustToAttitude.
inline void thrustToAttitude(const Vector3f &thr_sp, float yaw_sp, vehicle_attitude_setpoint_s &att_sp)
{
	bodyzToAttitude(-thr_sp, yaw_sp, att_sp);
	att_sp.thrust_body[0] = 0.f;
	att_sp.thrust_body[1] = 0.f;
	att_sp.thrust_body[2] = -thr_sp.length();
}

// Constrain v0 + v1 inside a circle of radius `max`, keeping v0 priority over v1.
// Mirrors ControlMath::constrainXY exactly.
inline Vector2f constrainXY(const Vector2f &v0, const Vector2f &v1, float max)
{
	if (Vector2f(v0 + v1).norm() <= max) {
		return v0 + v1;

	} else if (v0.length() >= max) {
		return v0.normalized() * max;

	} else if (fabsf(Vector2f(v1 - v0).norm()) < 0.001f) {
		return v0.normalized() * max;

	} else if (v0.length() < 0.001f) {
		return v1.normalized() * max;

	} else {
		// vf = v0 + s * u1 with ||vf|| <= max, derived as a quadratic in s.
		Vector2f u1 = v1.normalized();
		float m = u1.dot(v0);
		float c = v0.dot(v0) - max * max;
		float s = -m + sqrtf(m * m - c);
		return v0 + u1 * s;
	}
}

} // anonymous namespace

// ============================================================================
// CustomPositionControl
// ============================================================================

CustomPositionControl::CustomPositionControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_vel_int.zero();

	// Default filter passthrough until parameters_updated() configures them.
	_vel_xy_lp_filter.setAlpha(1.f);
	_vel_z_lp_filter.setAlpha(1.f);
	_vel_deriv_xy_lp_filter.setAlpha(1.f);
	_vel_deriv_z_lp_filter.setAlpha(1.f);

	_tilt_limit_slew_rate.setSlewRate(.2f);

	ModuleParams::updateParams();
	parameters_updated();
}

CustomPositionControl::~CustomPositionControl()
{
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

bool CustomPositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("vehicle_local_position callback registration failed");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

void CustomPositionControl::parameters_updated()
{
	_gain_pos_p = Vector3f(_param_mpc_xy_p.get(),         _param_mpc_xy_p.get(),         _param_mpc_z_p.get());
	_gain_vel_p = Vector3f(_param_mpc_xy_vel_p_acc.get(), _param_mpc_xy_vel_p_acc.get(), _param_mpc_z_vel_p_acc.get());
	_gain_vel_i = Vector3f(_param_mpc_xy_vel_i_acc.get(), _param_mpc_xy_vel_i_acc.get(), _param_mpc_z_vel_i_acc.get());
	_gain_vel_d = Vector3f(_param_mpc_xy_vel_d_acc.get(), _param_mpc_xy_vel_d_acc.get(), _param_mpc_z_vel_d_acc.get());

	_lim_vel_horizontal = _param_mpc_xy_vel_max.get();
	_lim_vel_up         = _param_mpc_z_vel_max_up.get();
	_lim_vel_down       = _param_mpc_z_vel_max_dn.get();

	// _lim_thr_min must stay positive to leave a usable thrust direction; the
	// PositionControl reference enforces 1e-3 here, do the same.
	_lim_thr_min       = math::max(_param_mpc_thr_min.get(), 10e-4f);
	_lim_thr_max       = _param_mpc_thr_max.get();
	_lim_thr_xy_margin = _param_mpc_thr_xy_marg.get();
	_lim_tilt          = math::radians(_param_mpc_tiltmax_air.get());

	_hover_thrust = math::constrain(_param_mpc_thr_hover.get(), 0.05f, 0.9f);

	_decouple_horizontal_and_vertical_acceleration = _param_mpc_acc_decouple.get();
}

void CustomPositionControl::update_hover_thrust(float hover_thrust_new)
{
	// T = a_sp * Th/g - Th. To keep thrust continuous when Th changes,
	// add (a_sp' - a_sp) into the vertical integrator with
	// a_sp' = (a_sp - g) * Th / Th' + g.
	const float previous_hover_thrust = _hover_thrust;
	const float new_hover_thrust = math::constrain(hover_thrust_new, 0.05f, 0.9f);
	_hover_thrust = new_hover_thrust;

	if (PX4_ISFINITE(_acc_sp(2))) {
		_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
			       + CONSTANTS_ONE_G - _acc_sp(2);
	}
}

PositionControlStates CustomPositionControl::set_vehicle_states(const vehicle_local_position_s &local_pos, float dt)
{
	PositionControlStates states;

	const Vector2f position_xy(local_pos.x, local_pos.y);

	if (local_pos.xy_valid && position_xy.isAllFinite()) {
		states.position.xy() = position_xy;

	} else {
		states.position(0) = states.position(1) = NAN;
	}

	if (PX4_ISFINITE(local_pos.z) && local_pos.z_valid) {
		states.position(2) = local_pos.z;

	} else {
		states.position(2) = NAN;
	}

	const Vector2f velocity_xy(local_pos.vx, local_pos.vy);

	if (local_pos.v_xy_valid && velocity_xy.isAllFinite()) {
		const Vector2f vel_xy_prev = _vel_xy_lp_filter.getState();
		states.velocity.xy() = _vel_xy_lp_filter.update(velocity_xy);
		states.acceleration.xy() = _vel_deriv_xy_lp_filter.update((_vel_xy_lp_filter.getState() - vel_xy_prev) / dt);

	} else {
		states.velocity(0) = states.velocity(1) = NAN;
		states.acceleration(0) = states.acceleration(1) = NAN;
		_vel_xy_lp_filter.reset({});
		_vel_deriv_xy_lp_filter.reset({});
	}

	if (PX4_ISFINITE(local_pos.vz) && local_pos.v_z_valid) {
		const float vel_z_prev = _vel_z_lp_filter.getState();
		states.velocity(2) = _vel_z_lp_filter.update(local_pos.vz);
		states.acceleration(2) = _vel_deriv_z_lp_filter.update((_vel_z_lp_filter.getState() - vel_z_prev) / dt);

	} else {
		states.velocity(2) = NAN;
		states.acceleration(2) = NAN;
		_vel_z_lp_filter.reset({});
		_vel_deriv_z_lp_filter.reset({});
	}

	states.yaw = local_pos.heading;

	return states;
}

trajectory_setpoint_s CustomPositionControl::generate_failsafe_setpoint(hrt_abstime now,
		const PositionControlStates &states)
{
	trajectory_setpoint_s failsafe_setpoint = custom_empty_trajectory_setpoint;
	failsafe_setpoint.timestamp = now;

	if (Vector2f(states.velocity.xy()).isAllFinite()) {
		failsafe_setpoint.velocity[0] = 0.f;
		failsafe_setpoint.velocity[1] = 0.f;

	} else {
		failsafe_setpoint.acceleration[0] = 0.f;
		failsafe_setpoint.acceleration[1] = 0.f;
		failsafe_setpoint.velocity[2] = _param_mpc_land_speed.get();
	}

	if (PX4_ISFINITE(states.velocity(2))) {
		if (!PX4_ISFINITE(failsafe_setpoint.velocity[2])) {
			failsafe_setpoint.velocity[2] = 0.f;
		}

	} else {
		failsafe_setpoint.velocity[2] = NAN;
		failsafe_setpoint.acceleration[2] = .3f;
	}

	return failsafe_setpoint;
}

void CustomPositionControl::adjust_setpoint_for_ekf_resets(const vehicle_local_position_s &local_pos,
		trajectory_setpoint_s &setpoint)
{
	if ((setpoint.timestamp != 0) && (setpoint.timestamp < local_pos.timestamp)) {
		if (local_pos.vxy_reset_counter != _vxy_reset_counter) {
			setpoint.velocity[0] += local_pos.delta_vxy[0];
			setpoint.velocity[1] += local_pos.delta_vxy[1];
		}

		if (local_pos.vz_reset_counter != _vz_reset_counter) {
			setpoint.velocity[2] += local_pos.delta_vz;
		}

		if (local_pos.xy_reset_counter != _xy_reset_counter) {
			setpoint.position[0] += local_pos.delta_xy[0];
			setpoint.position[1] += local_pos.delta_xy[1];
		}

		if (local_pos.z_reset_counter != _z_reset_counter) {
			setpoint.position[2] += local_pos.delta_z;
		}

		if (local_pos.heading_reset_counter != _heading_reset_counter) {
			setpoint.yaw = wrap_pi(setpoint.yaw + local_pos.delta_heading);
		}
	}

	if (local_pos.vxy_reset_counter != _vxy_reset_counter) {
		_vel_xy_lp_filter.reset(_vel_xy_lp_filter.getState() + Vector2f(local_pos.delta_vxy));
	}

	if (local_pos.vz_reset_counter != _vz_reset_counter) {
		_vel_z_lp_filter.reset(_vel_z_lp_filter.getState() + local_pos.delta_vz);
	}

	_vxy_reset_counter     = local_pos.vxy_reset_counter;
	_vz_reset_counter      = local_pos.vz_reset_counter;
	_xy_reset_counter      = local_pos.xy_reset_counter;
	_z_reset_counter       = local_pos.z_reset_counter;
	_heading_reset_counter = local_pos.heading_reset_counter;
}

void CustomPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup(desc);
		return;
	}

	// Reschedule backup so we still run even if the angular_velocity callback stops firing.
	ScheduleDelayed(100_ms);

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s pu;
		_parameter_update_sub.copy(&pu);
		updateParams();
		parameters_updated();
	}

	vehicle_local_position_s local_pos;

	if (_local_pos_sub.update(&local_pos)) {
		const hrt_abstime now = hrt_absolute_time();
		const float dt = math::constrain(((local_pos.timestamp_sample - _time_stamp_last_loop) * 1e-6f),
						 0.002f, 0.04f);
		_time_stamp_last_loop = local_pos.timestamp_sample;

		// Control mode transitions (mirrors mc_pos_control)
		if (_vehicle_control_mode_sub.updated()) {
			const bool prev_enabled = _vehicle_control_mode.flag_multicopter_position_control_enabled;

			if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
				if (!prev_enabled && _vehicle_control_mode.flag_multicopter_position_control_enabled) {
					_time_position_control_enabled = _vehicle_control_mode.timestamp;
					// Defer initial yaw capture until the first run with a finite measurement.
					_yaw_sp = NAN;

				} else if (prev_enabled && !_vehicle_control_mode.flag_multicopter_position_control_enabled) {
					_setpoint = custom_empty_trajectory_setpoint;
				}
			}
		}

		_vehicle_land_detected_sub.update(&_vehicle_land_detected);

		// Hover thrust estimator update - adjusts integrator so output stays continuous.
		if (_hover_thrust_estimate_sub.updated()) {
			hover_thrust_estimate_s hte;

			if (_hover_thrust_estimate_sub.copy(&hte) && hte.valid) {
				update_hover_thrust(hte.hover_thrust);
			}
		}

		PositionControlStates states = set_vehicle_states(local_pos, dt);

		_trajectory_setpoint_sub.update(&_setpoint);
		adjust_setpoint_for_ekf_resets(local_pos, _setpoint);

		if (_vehicle_control_mode.flag_multicopter_position_control_enabled) {

			// No fresh setpoint since position control was enabled: synthesize one.
			if ((_setpoint.timestamp < _time_position_control_enabled)
			    && (local_pos.timestamp_sample > _time_position_control_enabled)) {
				_setpoint = generate_failsafe_setpoint(local_pos.timestamp_sample, states);
			}

			// Latch states and setpoints used by the inline math below.
			_pos      = states.position;
			_vel      = states.velocity;
			_vel_dot  = states.acceleration;
			_yaw      = states.yaw;

			_pos_sp   = Vector3f(_setpoint.position);
			_vel_sp   = Vector3f(_setpoint.velocity);
			_acc_sp   = Vector3f(_setpoint.acceleration);

			if (PX4_ISFINITE(_setpoint.yaw)) {
				_yaw_sp = _setpoint.yaw;
			}

			if (!PX4_ISFINITE(_yaw_sp)) {
				_yaw_sp = _yaw;
			}

			_yawspeed_sp = _setpoint.yawspeed;

			if (_vehicle_land_detected.landed) {
				_vel_int.zero();
			}

			// Slew the tilt limit to avoid step changes when params change.
			_lim_tilt = _tilt_limit_slew_rate.update(math::radians(_param_mpc_tiltmax_air.get()), dt);

			// =====================================================================
			// _inputValid() inline - require finite states for every controlled axis
			// =====================================================================
			bool valid = true;

			for (int i = 0; i <= 2; i++) {
				valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i))
						  || PX4_ISFINITE(_acc_sp(i)));
			}

			valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
			valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
			valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

			for (int i = 0; i <= 2; i++) {
				if (PX4_ISFINITE(_pos_sp(i))) {
					valid = valid && PX4_ISFINITE(_pos(i));
				}

				if (PX4_ISFINITE(_vel_sp(i))) {
					valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
				}
			}

			if (valid) {

				// =================================================================
				// _positionControl() inline - P loop on position error.
				// =================================================================
				Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
				addIfNotNanVector3f(_vel_sp, vel_sp_position);
				setZeroIfNanVector3f(vel_sp_position);

				// Prioritize the position-derived velocity over the feed-forward component
				// when saturating the horizontal velocity budget.
				_vel_sp.xy() = constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(),
							   _lim_vel_horizontal);

				_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);

				// =================================================================
				// _velocityControl() inline - PID loop on velocity error.
				// =================================================================
				_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

				Vector3f vel_error = _vel_sp - _vel;
				Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int
							   - _vel_dot.emult(_gain_vel_d);
				addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

				// =================================================================
				// _accelerationControl() inline - convert acceleration to thrust.
				// =================================================================
				float z_specific_force = -CONSTANTS_ONE_G;

				if (!_decouple_horizontal_and_vertical_acceleration) {
					z_specific_force += _acc_sp(2);
				}

				Vector3f body_z(-_acc_sp(0), -_acc_sp(1), -z_specific_force);
				body_z.normalize();
				limitTilt(body_z, Vector3f(0.f, 0.f, 1.f), _lim_tilt);

				const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
				const float cos_ned_body = Vector3f(0.f, 0.f, 1.f).dot(body_z);
				const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
				_thr_sp = body_z * collective_thrust;

				// =================================================================
				// Integrator anti-windup in vertical direction.
				// =================================================================
				if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
				    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
					vel_error(2) = 0.f;
				}

				// Vertical-prioritized thrust saturation with horizontal margin.
				const Vector2f thrust_sp_xy(_thr_sp);
				const float thrust_sp_xy_norm = thrust_sp_xy.norm();
				const float thrust_max_squared = math::sq(_lim_thr_max);

				const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
				const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

				_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

				const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
				float thrust_max_xy = 0.f;

				if (thrust_max_xy_squared > 0.f) {
					thrust_max_xy = sqrtf(thrust_max_xy_squared);
				}

				if (thrust_sp_xy_norm > thrust_max_xy) {
					_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
				}

				// Anti-Reset Windup (Rundqwist 1990) for horizontal axis.
				const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);

				if (_acc_sp.xy().norm_squared() > acc_sp_xy_produced.norm_squared()) {
					const float arw_gain = 2.f / _gain_vel_p(0);
					const Vector2f acc_sp_xy = _acc_sp.xy();
					vel_error.xy() = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_sp_xy_produced);
				}

				setZeroIfNanVector3f(vel_error);
				_vel_int += vel_error.emult(_gain_vel_i) * dt;

				_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
				_yaw_sp      = PX4_ISFINITE(_yaw_sp)      ? _yaw_sp      : _yaw;

				// =================================================================
				// Publish local position setpoint (atomic build, zero-copy via copyTo)
				// =================================================================
				vehicle_local_position_setpoint_s local_pos_sp{};
				local_pos_sp.x        = _pos_sp(0);
				local_pos_sp.y        = _pos_sp(1);
				local_pos_sp.z        = _pos_sp(2);
				local_pos_sp.yaw      = _yaw_sp;
				local_pos_sp.yawspeed = _yawspeed_sp;
				local_pos_sp.vx       = _vel_sp(0);
				local_pos_sp.vy       = _vel_sp(1);
				local_pos_sp.vz       = _vel_sp(2);
				_acc_sp.copyTo(local_pos_sp.acceleration);
				_thr_sp.copyTo(local_pos_sp.thrust);
				local_pos_sp.timestamp = hrt_absolute_time();
				_local_pos_sp_pub.publish(local_pos_sp);

				// =================================================================
				// Publish attitude setpoint (thrust + yaw -> quaternion)
				// =================================================================
				vehicle_attitude_setpoint_s att_sp{};
				thrustToAttitude(_thr_sp, _yaw_sp, att_sp);
				att_sp.yaw_sp_move_rate = _yawspeed_sp;
				att_sp.timestamp = hrt_absolute_time();
				_vehicle_attitude_setpoint_pub.publish(att_sp);

			} else {
				// Invalid input: reset integrator so we don't accumulate junk.
				_vel_int.zero();
			}

		} else {
			// Position control disabled: keep integrator zero so we restart cleanly.
			_vel_int.zero();
		}

		(void)now;
	}

	perf_end(_cycle_perf);
}

int CustomPositionControl::task_spawn(int argc, char *argv[])
{
	CustomPositionControl *instance = new CustomPositionControl();

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

int CustomPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CustomPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Flat single-file multicopter position controller, functionally equivalent to
the core PositionControl + ControlMath used by mc_pos_control. Implements:

  - P loop on position error -> velocity setpoint (with prioritized 2D constrain)
  - PID loop on velocity error -> acceleration setpoint (with ARW anti-windup)
  - Acceleration -> thrust vector (with tilt limit and vertical-priority
    saturation keeping a horizontal margin)
  - Thrust + yaw -> attitude quaternion (thrustToAttitude / bodyzToAttitude)

Excludes Takeoff state machine and GotoControl (these are orthogonal state
machines, not core position-control math).

Input  : vehicle_local_position, trajectory_setpoint, vehicle_control_mode,
         vehicle_land_detected, hover_thrust_estimate
Output : vehicle_attitude_setpoint, vehicle_local_position_setpoint

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("custom_mc_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int custom_mc_pos_control_main(int argc, char *argv[])
{
	return ModuleBase::main(CustomPositionControl::desc, argc, argv);
}
