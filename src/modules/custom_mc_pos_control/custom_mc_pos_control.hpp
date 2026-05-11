/****************************************************************************
 *
 *   Custom Multicopter Position Controller (flat single-file replica of
 *   mc_pos_control's core PositionControl + ControlMath behavior).
 *
 *   All rotation math, transformations, NaN-safe operations, tilt limiting,
 *   prioritized 2D constrain, thrust-to-attitude conversion and PID with
 *   ARW anti-windup are kept in the .cpp - no PositionControl / ControlMath
 *   helper class.
 *
 ****************************************************************************/

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/slew_rate/SlewRate.hpp>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

#include <matrix/matrix/math.hpp>
#include <geo/geo.h>

using namespace time_literals;

// All-NAN trajectory setpoint used as "no command" sentinel.
static const trajectory_setpoint_s custom_empty_trajectory_setpoint = {
	0,
	{NAN, NAN, NAN}, // position
	{NAN, NAN, NAN}, // velocity
	{NAN, NAN, NAN}, // acceleration
	{NAN, NAN, NAN}, // jerk
	NAN,             // yaw
	NAN              // yawspeed
};

// Vehicle states consumed by the position-control math.
struct PositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
};

class CustomPositionControl : public ModuleBase, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	static Descriptor desc;

	CustomPositionControl();
	~CustomPositionControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	// Refresh cached gains/limits from parameters.
	void parameters_updated();

	// Build PositionControlStates from a local_position message, applying velocity LP filters.
	PositionControlStates set_vehicle_states(const vehicle_local_position_s &local_pos, float dt);

	// Build the failsafe setpoint when no valid trajectory_setpoint is available.
	trajectory_setpoint_s generate_failsafe_setpoint(hrt_abstime now, const PositionControlStates &states);

	// Adjust the stored / latest trajectory setpoint for EKF state resets (xy/z/vxy/vz/heading).
	void adjust_setpoint_for_ekf_resets(const vehicle_local_position_s &local_pos, trajectory_setpoint_s &setpoint);

	// Update the integrator to keep thrust output continuous across hover-thrust changes.
	void update_hover_thrust(float hover_thrust_new);

	// Subscriptions
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};

	// Publications
	uORB::Publication<vehicle_attitude_setpoint_s>       _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};

	// Timing
	hrt_abstime _time_stamp_last_loop{0};
	hrt_abstime _time_position_control_enabled{0};

	// Current trajectory setpoint (sticky between updates)
	trajectory_setpoint_s _setpoint{custom_empty_trajectory_setpoint};

	// Control mode + land detection cache
	vehicle_control_mode_s _vehicle_control_mode{};
	vehicle_land_detected_s _vehicle_land_detected {
		.timestamp = 0,
		.freefall = false,
		.ground_contact = true,
		.maybe_landed = true,
		.landed = true,
	};

	// EKF reset bookkeeping
	uint8_t _vxy_reset_counter{0};
	uint8_t _vz_reset_counter{0};
	uint8_t _xy_reset_counter{0};
	uint8_t _z_reset_counter{0};
	uint8_t _heading_reset_counter{0};

	// Velocity filters
	AlphaFilter<matrix::Vector2f> _vel_xy_lp_filter{};
	AlphaFilter<float>            _vel_z_lp_filter{};
	AlphaFilter<matrix::Vector2f> _vel_deriv_xy_lp_filter{};
	AlphaFilter<float>            _vel_deriv_z_lp_filter{};

	// Cached controller gains (Vector3f-per-axis form to match mc_pos_control)
	matrix::Vector3f _gain_pos_p;
	matrix::Vector3f _gain_vel_p;
	matrix::Vector3f _gain_vel_i;
	matrix::Vector3f _gain_vel_d;

	// Per-cycle controller state (kept as members so debugging / status are easy)
	matrix::Vector3f _pos;
	matrix::Vector3f _vel;
	matrix::Vector3f _vel_dot;
	matrix::Vector3f _vel_int;
	matrix::Vector3f _pos_sp;
	matrix::Vector3f _vel_sp;
	matrix::Vector3f _acc_sp;
	matrix::Vector3f _thr_sp;
	float _yaw{0.f};
	float _yaw_sp{NAN};
	float _yawspeed_sp{0.f};

	// Limits / configuration
	float _hover_thrust{0.5f};
	float _lim_vel_horizontal{0.f};
	float _lim_vel_up{0.f};
	float _lim_vel_down{0.f};
	float _lim_thr_min{0.f};
	float _lim_thr_max{0.f};
	float _lim_thr_xy_margin{0.f};
	float _lim_tilt{0.f};
	bool  _decouple_horizontal_and_vertical_acceleration{true};

	// Tilt-limit slew rate (replaces mc_pos_control's slew during takeoff ramp).
	SlewRate<float> _tilt_limit_slew_rate;

	// Performance counters
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME ": interval")};

	DEFINE_PARAMETERS(
		// Position P
		(ParamFloat<px4::params::MPC_XY_P>) _param_mpc_xy_p,
		(ParamFloat<px4::params::MPC_Z_P>)  _param_mpc_z_p,

		// Velocity PID
		(ParamFloat<px4::params::MPC_XY_VEL_P_ACC>) _param_mpc_xy_vel_p_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_I_ACC>) _param_mpc_xy_vel_i_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_D_ACC>) _param_mpc_xy_vel_d_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_P_ACC>)  _param_mpc_z_vel_p_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_I_ACC>)  _param_mpc_z_vel_i_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_D_ACC>)  _param_mpc_z_vel_d_acc,

		// Velocity limits
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>)   _param_mpc_xy_vel_max,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,

		// Thrust
		(ParamFloat<px4::params::MPC_THR_HOVER>)   _param_mpc_thr_hover,
		(ParamFloat<px4::params::MPC_THR_MIN>)     _param_mpc_thr_min,
		(ParamFloat<px4::params::MPC_THR_MAX>)     _param_mpc_thr_max,
		(ParamFloat<px4::params::MPC_THR_XY_MARG>) _param_mpc_thr_xy_marg,

		// Tilt
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>) _param_mpc_tiltmax_air,

		// Coupling option
		(ParamBool<px4::params::MPC_ACC_DECOUPLE>) _param_mpc_acc_decouple,

		// Land speed - used by failsafe to descend with a known rate.
		(ParamFloat<px4::params::MPC_LAND_SPEED>)  _param_mpc_land_speed
	)
};
