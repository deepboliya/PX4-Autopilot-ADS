/****************************************************************************
 *
 *   Custom Minimal Position Controller for X500 Quadcopter
 *
 *   A simplified position controller optimized for Gazebo X500 simulation.
 *   Stripped of VTOL support, goto control, and complex checks.
 *
 ****************************************************************************/

#include "custom_mc_pos_control.hpp"

#include <float.h>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

using namespace matrix;

CustomPositionControl::CustomPositionControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	// Initialize with X500 defaults
	_hover_thrust = X500_HOVER_THRUST;
	_vel_int.zero();

	// Initialize gains with typical X500 values (will be overwritten by params)
	_gain_pos_p = Vector3f(0.95f, 0.95f, 1.0f);
	_gain_vel_p = Vector3f(1.8f, 1.8f, 4.0f);
	_gain_vel_i = Vector3f(0.4f, 0.4f, 2.0f);
	_gain_vel_d = Vector3f(0.2f, 0.2f, 0.0f);

	// Initialize filters (no filtering by default)
	_vel_xy_lp_filter.setAlpha(1.f);
	_vel_z_lp_filter.setAlpha(1.f);
	_vel_deriv_xy_lp_filter.setAlpha(1.f);
	_vel_deriv_z_lp_filter.setAlpha(1.f);

	// Load parameters once at startup
	ModuleParams::updateParams();

	// Update gains from parameters
	_gain_pos_p = Vector3f(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(), _param_mpc_z_p.get());
	_gain_vel_p = Vector3f(_param_mpc_xy_vel_p_acc.get(), _param_mpc_xy_vel_p_acc.get(), _param_mpc_z_vel_p_acc.get());
	_gain_vel_i = Vector3f(_param_mpc_xy_vel_i_acc.get(), _param_mpc_xy_vel_i_acc.get(), _param_mpc_z_vel_i_acc.get());
	_gain_vel_d = Vector3f(_param_mpc_xy_vel_d_acc.get(), _param_mpc_xy_vel_d_acc.get(), _param_mpc_z_vel_d_acc.get());

	// Update limits from parameters
	_lim_vel_horizontal = _param_mpc_xy_vel_max.get();
	_lim_vel_up = _param_mpc_z_vel_max_up.get();
	_lim_vel_down = _param_mpc_z_vel_max_dn.get();
	_lim_thr_min = math::max(_param_mpc_thr_min.get(), 0.01f);
	_lim_thr_max = _param_mpc_thr_max.get();
	_lim_tilt = math::radians(_param_mpc_tiltmax_air.get());

	// Set hover thrust
	_hover_thrust = math::constrain(_param_mpc_thr_hover.get(), 0.1f, 0.9f);

	PX4_INFO("Custom Position Controller initialized for X500");
}

CustomPositionControl::~CustomPositionControl()
{
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

bool CustomPositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

PositionControlStates CustomPositionControl::set_vehicle_states(const vehicle_local_position_s &local_pos, float dt)
{
	PositionControlStates states;

	// Position
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

	// Velocity with filtering
	const Vector2f velocity_xy(local_pos.vx, local_pos.vy);

	if (local_pos.v_xy_valid && velocity_xy.isAllFinite()) {
		const Vector2f vel_xy_prev = _vel_xy_lp_filter.getState();
		states.velocity.xy() = _vel_xy_lp_filter.update(velocity_xy);
		states.acceleration.xy() = _vel_deriv_xy_lp_filter.update((states.velocity.xy() - vel_xy_prev) / dt);
	} else {
		states.velocity(0) = states.velocity(1) = NAN;
		states.acceleration(0) = states.acceleration(1) = NAN;
		_vel_xy_lp_filter.reset({});
		_vel_deriv_xy_lp_filter.reset({});
	}

	if (PX4_ISFINITE(local_pos.vz) && local_pos.v_z_valid) {
		const float vel_z_prev = _vel_z_lp_filter.getState();
		states.velocity(2) = _vel_z_lp_filter.update(local_pos.vz);
		states.acceleration(2) = _vel_deriv_z_lp_filter.update((states.velocity(2) - vel_z_prev) / dt);
	} else {
		states.velocity(2) = NAN;
		states.acceleration(2) = NAN;
		_vel_z_lp_filter.reset({});
		_vel_deriv_z_lp_filter.reset({});
	}

	states.yaw = local_pos.heading;

	return states;
}

void CustomPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// Reschedule backup
	ScheduleDelayed(100_ms);

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	vehicle_local_position_s local_pos;

	if (_local_pos_sub.update(&local_pos)) {
		const hrt_abstime now = hrt_absolute_time();
		const float dt = math::constrain(((local_pos.timestamp_sample - _time_stamp_last_loop) * 1e-6f),
						 POS_CTRL_MIN_DT, POS_CTRL_MAX_DT);
		_time_stamp_last_loop = local_pos.timestamp_sample;

		// Update control mode
		if (_vehicle_control_mode_sub.updated()) {
			const bool prev_enabled = _vehicle_control_mode.flag_multicopter_position_control_enabled;

			if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
				if (!prev_enabled && _vehicle_control_mode.flag_multicopter_position_control_enabled) {
					_time_position_control_enabled = _vehicle_control_mode.timestamp;
					// Reset yaw setpoint to NAN to capture current yaw on first run
					_yaw_sp = NAN;
				} else if (prev_enabled && !_vehicle_control_mode.flag_multicopter_position_control_enabled) {
					_setpoint = empty_trajectory_setpoint;
				}
			}
		}

		// Update land detection
		_vehicle_land_detected_sub.update(&_vehicle_land_detected);

		// Get vehicle states
		PositionControlStates states = set_vehicle_states(local_pos, dt);

		// Get trajectory setpoint
		_trajectory_setpoint_sub.update(&_setpoint);

		// Run position control if enabled
		if (_vehicle_control_mode.flag_multicopter_position_control_enabled) {

			// Generate failsafe if no valid setpoint
			if (_setpoint.timestamp < _time_position_control_enabled) {
				_setpoint = generate_failsafe_setpoint(now, states);
			}

			// Store states
			_pos = states.position;
			_vel = states.velocity;
			_vel_dot = states.acceleration;
			_yaw = states.yaw;

			// Set setpoints
			_pos_sp = Vector3f(_setpoint.position);
			_vel_sp = Vector3f(_setpoint.velocity);
			_acc_sp = Vector3f(_setpoint.acceleration);
			
			// Yaw Setpoint Logic: Hold last valid yaw if input is NAN
			if (PX4_ISFINITE(_setpoint.yaw)) {
				_yaw_sp = _setpoint.yaw;
			}
			// If _yaw_sp is still NAN (first run and no input), initialize to current yaw
			if (!PX4_ISFINITE(_yaw_sp)) {
				_yaw_sp = _yaw;
			}
			
			_yawspeed_sp = _setpoint.yawspeed;

			// Reset integrator if landed
			if (_vehicle_land_detected.landed) {
				_vel_int.zero();
			}

			// Run control if input is valid
			if (input_valid()) {
				// Position P control
				position_control(states);

				// Velocity PID control (calls acceleration_control internally)
				velocity_control(dt);

				// Ensure valid yaw rate
				_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;

				// Publish local position setpoint
				vehicle_local_position_setpoint_s local_pos_sp{};
				local_pos_sp.timestamp = hrt_absolute_time();
				local_pos_sp.x = _pos_sp(0);
				local_pos_sp.y = _pos_sp(1);
				local_pos_sp.z = _pos_sp(2);
				local_pos_sp.yaw = _yaw_sp;
				local_pos_sp.yawspeed = _yawspeed_sp;
				local_pos_sp.vx = _vel_sp(0);
				local_pos_sp.vy = _vel_sp(1);
				local_pos_sp.vz = _vel_sp(2);
				_acc_sp.copyTo(local_pos_sp.acceleration);
				_thr_sp.copyTo(local_pos_sp.thrust);
				_local_pos_sp_pub.publish(local_pos_sp);

				// Publish attitude setpoint
				vehicle_attitude_setpoint_s att_sp{};
				thrust_to_attitude(_thr_sp, _yaw_sp, att_sp);
				att_sp.yaw_sp_move_rate = _yawspeed_sp;
				att_sp.timestamp = hrt_absolute_time();
				_vehicle_attitude_setpoint_pub.publish(att_sp);

			} else {
				// Invalid input - reset integrator
				_vel_int.zero();
			}

		} else {
			// Position control disabled - reset integrator
			_vel_int.zero();
		}
	}

	perf_end(_cycle_perf);
}

void CustomPositionControl::position_control(const PositionControlStates &states)
{
	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);

	// Add position-based velocity to feedforward velocity
	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(vel_sp_position(i))) {
			if (!PX4_ISFINITE(_vel_sp(i))) {
				_vel_sp(i) = vel_sp_position(i);
			} else {
				_vel_sp(i) += vel_sp_position(i);
			}
		}
		else{
			// PX4_INFO("Position control NAN detected on axis %d", i);
		}
	}

	// Make sure there are no NAN for further reference
	for (int i = 0; i < 3; i++) {
		if (!PX4_ISFINITE(vel_sp_position(i))) {
			vel_sp_position(i) = 0.f;
		}
	}

	// Constrain horizontal velocity (prioritize position control over feedforward)
	const Vector2f vel_sp_xy = Vector2f(_vel_sp.xy());
	const float vel_sp_xy_norm = vel_sp_xy.norm();

	if (vel_sp_xy_norm > _lim_vel_horizontal) {
		_vel_sp.xy() = vel_sp_xy / vel_sp_xy_norm * _lim_vel_horizontal;
	}

	// Constrain vertical velocity
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void CustomPositionControl::velocity_control(float dt)
{
	// Constrain vertical velocity integral
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	// PID velocity control
	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

	// Add velocity-based acceleration to feedforward acceleration
	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(acc_sp_velocity(i))) {
			if (!PX4_ISFINITE(_acc_sp(i))) {
				_acc_sp(i) = acc_sp_velocity(i);
			} else {
				_acc_sp(i) += acc_sp_velocity(i);
			}
		}
	}

	// Convert acceleration to thrust (needed before anti-windup uses _thr_sp)
	acceleration_control();

	// Integrator anti-windup in vertical direction
	if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
	    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
		vel_error(2) = 0.f;
	}

	// Prioritize vertical control while keeping a horizontal margin
	const Vector2f thrust_sp_xy(_thr_sp.xy());
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = _lim_thr_max * _lim_thr_max;

	// Saturate maximal vertical thrust
	const float thrust_z_max_squared = thrust_max_squared;
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left
	const float thrust_max_xy_squared = thrust_max_squared - _thr_sp(2) * _thr_sp(2);
	float thrust_max_xy = 0.f;

	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate horizontal thrust
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	// Make sure integral doesn't get NAN
	for (int i = 0; i < 3; i++) {
		if (!PX4_ISFINITE(vel_error(i))) {
			vel_error(i) = 0.f;
		}
	}

	// Update integral
	_vel_int += vel_error.emult(_gain_vel_i) * dt;
}

void CustomPositionControl::acceleration_control()
{	/*
	// Assume standard gravity for attitude generation
	float z_specific_force = -CONSTANTS_ONE_G;

	// Body z direction
	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();

	// Limit tilt
	// limit_tilt(body_z, Vector3f(0.f, 0.f, 1.f), _lim_tilt);

	// Convert to thrust assuming hover thrust produces standard gravity
	const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;

	// Project thrust to planned body attitude
	const float cos_ned_body = Vector3f(0.f, 0.f, 1.f).dot(body_z);
	const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);

	_thr_sp = body_z * collective_thrust;
	*/
	Vector3f gravity(0.f, 0.f, CONSTANTS_ONE_G);
    	Vector3f total_acceleration = _acc_sp - gravity;
	if (total_acceleration.norm_squared() < 0.001f) {
		_thr_sp.zero();
		return;
    	}
	Vector3f body_z = total_acceleration.normalized();
	float acceleration_magnitude = total_acceleration.norm();
    	float collective_thrust = (acceleration_magnitude / CONSTANTS_ONE_G) * _hover_thrust; // mass = hover_thrust / g
	collective_thrust = math::min(collective_thrust, _lim_thr_max);
	_thr_sp = body_z * collective_thrust;
}

void CustomPositionControl::limit_tilt(Vector3f &body_unit, const Vector3f &world_unit, float max_angle)
{
	// Determine tilt
	const float dot_product_unit = body_unit.dot(world_unit);
	float angle = acosf(dot_product_unit);

	// Limit tilt
	angle = math::min(angle, max_angle);
	Vector3f rejection = body_unit - (dot_product_unit * world_unit);

	// Corner case: exactly parallel vectors
	if (rejection.norm_squared() < FLT_EPSILON) {
		rejection(0) = 1.f;
	}

	body_unit = cosf(angle) * world_unit + sinf(angle) * rejection.unit();
}

void CustomPositionControl::thrust_to_attitude(const Vector3f &thr_sp, float yaw_sp,
		vehicle_attitude_setpoint_s &att_sp)
{
	Vector3f body_z = -thr_sp;

	// Zero vector, no direction, set safe level value
	if (body_z.norm_squared() < FLT_EPSILON) {
		body_z(2) = 1.f;
	}

	body_z.normalize();

	// Vector of desired yaw direction in XY plane, rotated by PI/2
	const Vector3f y_C{-sinf(yaw_sp), cosf(yaw_sp), 0.f};

	// Desired body_x axis, orthogonal to body_z
	Vector3f body_x = y_C % body_z;

	// Keep nose to front while inverted upside down
	if (body_z(2) < 0.f) {
		body_x = -body_x;
	}

	if (fabsf(body_z(2)) < 0.000001f) {
		body_x.zero();
		body_x(2) = 1.f;
	}

	body_x.normalize();

	// Desired body_y axis
	const Vector3f body_y = body_z % body_x;

	// Rotation matrix from body to world
	Dcmf R_sp;
	R_sp.setCol(0, body_x);
	R_sp.setCol(1, body_y);
	R_sp.setCol(2, body_z);

	// Copy quaternion setpoint to attitude setpoint
	const Quatf q_sp(R_sp);
	q_sp.copyTo(att_sp.q_d);

	// Set thrust
	att_sp.thrust_body[0] = 0.f;
	att_sp.thrust_body[1] = 0.f;
	att_sp.thrust_body[2] = -thr_sp.length();
}

bool CustomPositionControl::input_valid()
{
	bool valid = true;

	// Every axis needs some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// X and Y setpoints always come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state, the estimate must be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

trajectory_setpoint_s CustomPositionControl::generate_failsafe_setpoint(hrt_abstime now,
		const PositionControlStates &states)
{
	trajectory_setpoint_s failsafe_setpoint = empty_trajectory_setpoint;
	failsafe_setpoint.timestamp = now;

	if (Vector2f(states.velocity.xy()).isAllFinite()) {
		// Stop horizontally
		failsafe_setpoint.velocity[0] = 0.f;
		failsafe_setpoint.velocity[1] = 0.f;
	} else {
		// Can't stop - set zero acceleration
		failsafe_setpoint.acceleration[0] = 0.f;
		failsafe_setpoint.acceleration[1] = 0.f;
		failsafe_setpoint.velocity[2] = 1.0f; // Land slowly
	}

	if (PX4_ISFINITE(states.velocity(2))) {
		if (!PX4_ISFINITE(failsafe_setpoint.velocity[2])) {
			failsafe_setpoint.velocity[2] = 0.f;
		}
	} else {
		// Emergency descend
		failsafe_setpoint.velocity[2] = NAN;
		failsafe_setpoint.acceleration[2] = 0.3f;
	}

	return failsafe_setpoint;
}

void CustomPositionControl::reset_filters()
{
	_vel_xy_lp_filter.reset({});
	_vel_z_lp_filter.reset({});
	_vel_deriv_xy_lp_filter.reset({});
	_vel_deriv_z_lp_filter.reset({});
}

int CustomPositionControl::task_spawn(int argc, char *argv[])
{
	CustomPositionControl *instance = new CustomPositionControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			PX4_INFO("Custom Position Control started for X500");
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

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
Minimal position controller optimized for Gazebo X500 quadcopter.

This is a simplified version of mc_pos_control designed specifically for the X500 model.
It uses a cascaded P-PID controller for position and velocity control.

### X500 Optimization
- Hover thrust: 0.5 (normalized)
- Max tilt: 35 degrees
- Max horizontal velocity: 12 m/s
- Max vertical velocity: 3 m/s up, 1.5 m/s down
- No VTOL support
- No goto control complexity

### Implementation
- Position P loop generates velocity setpoints
- Velocity PID loop generates acceleration setpoints
- Acceleration is converted to thrust and attitude commands
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("custom_mc_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int custom_mc_pos_control_main(int argc, char *argv[])
{
	return CustomPositionControl::main(argc, argv);
}
