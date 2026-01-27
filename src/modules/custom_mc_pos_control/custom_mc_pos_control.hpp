/****************************************************************************
 *
 *   Custom Minimal Position Controller for X500 Quadcopter
 *
 *   A simplified position controller optimized for Gazebo X500 simulation.
 *   Stripped of VTOL support and complex checks - pure quadcopter operation.
 *
 ****************************************************************************/

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

#include <matrix/matrix/math.hpp>
#include <geo/geo.h>

using namespace time_literals;

// X500 SPECIFIC CONFIGURATION
static constexpr float X500_HOVER_THRUST = 0.5f;       // X500 hover thrust (normalized)
static constexpr float X500_MAX_TILT_RAD = 0.6f;       // ~35 degrees max tilt
static constexpr float X500_MAX_XY_VEL = 12.0f;        // m/s max horizontal velocity
static constexpr float X500_MAX_Z_VEL_UP = 3.0f;       // m/s max upward velocity
static constexpr float X500_MAX_Z_VEL_DOWN = 1.5f;     // m/s max downward velocity
static constexpr float X500_MIN_THRUST = 0.12f;        // Minimum thrust
static constexpr float X500_MAX_THRUST = 0.9f;         // Maximum thrust

// Control timing
static constexpr float POS_CTRL_EXPECTED_DT = 0.004f;  // 250Hz expected rate
static constexpr float POS_CTRL_MIN_DT = 0.002f;       // 500Hz maximum rate
static constexpr float POS_CTRL_MAX_DT = 0.04f;        // 25Hz minimum rate

/**
 * Vehicle states for position control
 */
struct PositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
};

/**
 * Empty trajectory setpoint (all NAN)
 */
static const trajectory_setpoint_s empty_trajectory_setpoint = {
	0,                      // timestamp
	{NAN, NAN, NAN},       // position
	{NAN, NAN, NAN},       // velocity
	{NAN, NAN, NAN},       // acceleration
	{NAN, NAN, NAN},       // jerk
	NAN,                   // yaw
	NAN                    // yawspeed
};

class CustomPositionControl : public ModuleBase<CustomPositionControl>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	CustomPositionControl();
	~CustomPositionControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * Set vehicle states from local position
	 */
	PositionControlStates set_vehicle_states(const vehicle_local_position_s &local_pos, float dt);

	/**
	 * Core position control - P loop
	 */
	void position_control(const PositionControlStates &states);

	/**
	 * Core velocity control - PID loop
	 */
	void velocity_control(float dt);

	/**
	 * Convert acceleration to thrust and attitude
	 */
	void acceleration_control();

	/**
	 * Get attitude setpoint from thrust vector
	 */
	void thrust_to_attitude(const matrix::Vector3f &thr_sp, float yaw_sp,
				vehicle_attitude_setpoint_s &att_sp);

	/**
	 * Check if input setpoints are valid
	 */
	bool input_valid();

	/**
	 * Generate failsafe setpoint
	 */
	trajectory_setpoint_s generate_failsafe_setpoint(hrt_abstime now, const PositionControlStates &states);

	/**
	 * Limit tilt angle
	 */
	void limit_tilt(matrix::Vector3f &body_unit, const matrix::Vector3f &world_unit, float max_angle);

	/**
	 * Reset velocity filters
	 */
	void reset_filters();

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _local_pos_sub{this, ORB_ID(vehicle_local_position)};
	uORB::Subscription _trajectory_setpoint_sub{ORB_ID(trajectory_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

	// Publications
	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Publication<vehicle_local_position_setpoint_s> _local_pos_sp_pub{ORB_ID(vehicle_local_position_setpoint)};

	// Timing
	hrt_abstime _time_stamp_last_loop{0};
	hrt_abstime _time_position_control_enabled{0};

	// Setpoints
	trajectory_setpoint_s _setpoint{empty_trajectory_setpoint};

	// Control mode
	vehicle_control_mode_s _vehicle_control_mode{};

	// Land detection
	vehicle_land_detected_s _vehicle_land_detected {
		.timestamp = 0,
		.freefall = false,
		.ground_contact = true,
		.maybe_landed = true,
		.landed = true,
	};

	// Velocity filters
	AlphaFilter<matrix::Vector2f> _vel_xy_lp_filter{};
	AlphaFilter<float> _vel_z_lp_filter{};
	AlphaFilter<matrix::Vector2f> _vel_deriv_xy_lp_filter{};
	AlphaFilter<float> _vel_deriv_z_lp_filter{};

	// Controller gains
	matrix::Vector3f _gain_pos_p;
	matrix::Vector3f _gain_vel_p;
	matrix::Vector3f _gain_vel_i;
	matrix::Vector3f _gain_vel_d;

	// Controller state
	matrix::Vector3f _pos;           // Current position
	matrix::Vector3f _vel;           // Current velocity
	matrix::Vector3f _vel_dot;       // Velocity derivative
	matrix::Vector3f _pos_sp;        // Position setpoint
	matrix::Vector3f _vel_sp;        // Velocity setpoint
	matrix::Vector3f _acc_sp;        // Acceleration setpoint
	matrix::Vector3f _thr_sp;        // Thrust setpoint
	matrix::Vector3f _vel_int;       // Velocity integral
	float _yaw{0.f};                 // Current yaw
	float _yaw_sp{0.f};              // Yaw setpoint
	float _yawspeed_sp{0.f};         // Yaw speed setpoint
	float _hover_thrust{X500_HOVER_THRUST};

	// Limits
	float _lim_vel_horizontal{X500_MAX_XY_VEL};
	float _lim_vel_up{X500_MAX_Z_VEL_UP};
	float _lim_vel_down{X500_MAX_Z_VEL_DOWN};
	float _lim_thr_min{X500_MIN_THRUST};
	float _lim_thr_max{X500_MAX_THRUST};
	float _lim_tilt{X500_MAX_TILT_RAD};

	// Performance counters
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// X500-optimized parameters
	DEFINE_PARAMETERS(
		// Position gains
		(ParamFloat<px4::params::MPC_XY_P>) _param_mpc_xy_p,
		(ParamFloat<px4::params::MPC_Z_P>) _param_mpc_z_p,

		// Velocity gains
		(ParamFloat<px4::params::MPC_XY_VEL_P_ACC>) _param_mpc_xy_vel_p_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_I_ACC>) _param_mpc_xy_vel_i_acc,
		(ParamFloat<px4::params::MPC_XY_VEL_D_ACC>) _param_mpc_xy_vel_d_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_P_ACC>) _param_mpc_z_vel_p_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_I_ACC>) _param_mpc_z_vel_i_acc,
		(ParamFloat<px4::params::MPC_Z_VEL_D_ACC>) _param_mpc_z_vel_d_acc,

		// Velocity limits
		(ParamFloat<px4::params::MPC_XY_VEL_MAX>) _param_mpc_xy_vel_max,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn,

		// Thrust
		(ParamFloat<px4::params::MPC_THR_HOVER>) _param_mpc_thr_hover,
		(ParamFloat<px4::params::MPC_THR_MIN>) _param_mpc_thr_min,
		(ParamFloat<px4::params::MPC_THR_MAX>) _param_mpc_thr_max,

		// Tilt
		(ParamFloat<px4::params::MPC_TILTMAX_AIR>) _param_mpc_tiltmax_air
	)
};
