/****************************************************************************
 *
 *   Custom Multicopter Attitude Controller (flat single-file replica of
 *   mc_att_control). All quaternion math (rotation, transformations,
 *   yaw deprioritization, rate limiting, yaw-rate feed-forward) is kept
 *   in the .cpp - no AttitudeControl/AttitudeControlMath helper class.
 *
 ****************************************************************************/

#pragma once

#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <drivers/drv_hrt.h>

#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/slew_rate/SlewRate.hpp>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/autotune_attitude_control_status.h>
#include <uORB/topics/hover_thrust_estimate.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>

using namespace time_literals;

class CustomAttitudeControl : public ModuleBase, public ModuleParams,
	public px4::WorkItem
{
public:
	static Descriptor desc;

	CustomAttitudeControl();
	~CustomAttitudeControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	// Refresh cached gains/limits from parameters.
	void parameters_updated();

	// Manual-throttle stick scaling curve (replicates mc_att_control::throttle_curve).
	float throttle_curve(float throttle_stick_input);

	// Generate an attitude setpoint from sticks and publish vehicle_attitude_setpoint.
	// Flat replica of mc_att_control::generate_attitude_setpoint - includes inlined yaw
	// stick-lock logic that would otherwise live in the StickYaw helper.
	void generate_attitude_setpoint(const matrix::Quatf &q, float dt);

	// Subscriptions
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _vehicle_attitude_sub{this, ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _autotune_attitude_control_status_sub{ORB_ID(autotune_attitude_control_status)};
	uORB::Subscription _hover_thrust_estimate_sub{ORB_ID(hover_thrust_estimate)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	// Publications
	uORB::Publication<vehicle_rates_setpoint_s>    _vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_attitude_setpoint_s> _vehicle_attitude_setpoint_pub{ORB_ID(vehicle_attitude_setpoint)};

	// Cached controller gains/limits
	matrix::Vector3f _proportional_gain;   ///< roll, pitch, yaw P gains (yaw gain is pre-divided by _yaw_w)
	matrix::Vector3f _rate_limit;          ///< per-axis rate limit [rad/s]
	float            _yaw_w{0.f};          ///< yaw deprioritization weight [0, 1]

	// Latest attitude setpoint state
	matrix::Quatf    _attitude_setpoint_q;            ///< latest desired attitude (normalized)
	float            _yawspeed_setpoint{0.f};         ///< latest yaw rate feed-forward [rad/s]
	matrix::Vector3f _thrust_setpoint_body{0.f, 0.f, 0.f}; ///< latest body-frame thrust (passed through)

	// Bookkeeping
	hrt_abstime _last_run{0};
	hrt_abstime _last_attitude_setpoint{0};
	uint8_t     _quat_reset_counter{0};

	vehicle_control_mode_s     _vehicle_control_mode{};
	manual_control_setpoint_s  _manual_control_setpoint{};

	// Manual / Stabilized stick-to-attitude state (flat StickYaw equivalent)
	float _man_tilt_max{0.f};            ///< MPC_MAN_TILT_MAX cached as radians
	float _yaw_setpoint_stabilized{NAN}; ///< heading lock, NaN while yaw stick active
	float _unaided_heading{NAN};         ///< from vehicle_local_position.unaided_heading, NaN if not available
	float _hover_thrust_estimate{NAN};   ///< latest valid HTE; NaN if none seen yet

	// Yaw stick low-pass + yaw-estimate convergence tracking (inline StickYaw state).
	AlphaFilter<float> _yawspeed_filter;
	AlphaFilter<float> _yaw_error_lpf{1.f};
	float _yaw_error_ref{0.f};
	float _yaw_correction{0.f};
	bool  _yaw_estimate_converging{false};

	// Throttle / hover-thrust slew rates and stick low-pass filters.
	SlewRate<float>    _manual_throttle_minimum{0.f}; ///< 0 when landed, ramped to MPC_MANTHR_MIN in air
	SlewRate<float>    _manual_throttle_maximum{0.f}; ///< 0 when disarmed, ramped to 1 when spooled up
	SlewRate<float>    _hover_thrust_slew_rate{.5f};
	AlphaFilter<float> _man_roll_input_filter;
	AlphaFilter<float> _man_pitch_input_filter;

	bool _spooled_up{false};
	bool _landed{true};

	// Performance
	perf_counter_t _loop_perf;
	perf_counter_t _loop_interval_perf;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLL_P>)        _param_mc_roll_p,
		(ParamFloat<px4::params::MC_PITCH_P>)       _param_mc_pitch_p,
		(ParamFloat<px4::params::MC_YAW_P>)         _param_mc_yaw_p,
		(ParamFloat<px4::params::MC_YAW_WEIGHT>)    _param_mc_yaw_weight,
		(ParamFloat<px4::params::MC_ROLLRATE_MAX>)  _param_mc_rollrate_max,
		(ParamFloat<px4::params::MC_PITCHRATE_MAX>) _param_mc_pitchrate_max,
		(ParamFloat<px4::params::MC_YAWRATE_MAX>)   _param_mc_yawrate_max,

		// Manual / Stabilized stick-to-attitude tuning
		(ParamInt<px4::params::MC_AIRMODE>)         _param_mc_airmode,
		(ParamFloat<px4::params::MC_MAN_TILT_TAU>)  _param_mc_man_tilt_tau,
		(ParamFloat<px4::params::MAN_DEADZONE>)     _param_man_deadzone,
		(ParamFloat<px4::params::MPC_MAN_TILT_MAX>) _param_mpc_man_tilt_max,
		(ParamFloat<px4::params::MPC_MAN_Y_MAX>)    _param_mpc_man_y_max,
		(ParamFloat<px4::params::MPC_MAN_Y_TAU>)    _param_mpc_man_y_tau,
		(ParamFloat<px4::params::MPC_MANTHR_MIN>)   _param_mpc_manthr_min,
		(ParamFloat<px4::params::MPC_THR_MAX>)      _param_mpc_thr_max,
		(ParamFloat<px4::params::MPC_THR_HOVER>)    _param_mpc_thr_hover,
		(ParamInt<px4::params::MPC_THR_CURVE>)      _param_mpc_thr_curve,
		(ParamFloat<px4::params::COM_SPOOLUP_TIME>) _param_com_spoolup_time
	)
};
