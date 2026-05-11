/****************************************************************************
 *
 *   Custom Multicopter Rate Controller (flat single-file replica of
 *   mc_rate_control). The full rate-loop math (PID with non-linear I factor,
 *   integrator clamp, control-allocator saturation handling, feed-forward,
 *   battery scaling, yaw low-pass on torque) is kept in the .cpp - no
 *   RateControl helper class.
 *
 ****************************************************************************/

#pragma once

#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

class CustomMCRateControl : public ModuleBase, public ModuleParams,
	public px4::WorkItem
{
public:
	static Descriptor desc;

	CustomMCRateControl();
	~CustomMCRateControl() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	// Refresh cached gains/limits from parameters.
	void parameters_updated();

	// Energy / control-power telemetry (mirrors mc_rate_control).
	void updateActuatorControlsStatus(const vehicle_torque_setpoint_s &torque_sp, float dt);

	// Subscriptions
	uORB::SubscriptionInterval         _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};

	// Publications
	uORB::Publication<actuator_controls_status_s> _actuator_controls_status_pub{ORB_ID(actuator_controls_status_0)};
	uORB::PublicationMulti<rate_ctrl_status_s>    _controller_status_pub{ORB_ID(rate_ctrl_status)};
	uORB::Publication<vehicle_thrust_setpoint_s>  _vehicle_thrust_setpoint_pub{ORB_ID(vehicle_thrust_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>  _vehicle_torque_setpoint_pub{ORB_ID(vehicle_torque_setpoint)};

	// Cached gains/limits (computed from parameters)
	matrix::Vector3f _gain_p;     ///< rate P gain per axis (parallel form = K * P)
	matrix::Vector3f _gain_i;     ///< rate I gain per axis (parallel form = K * I)
	matrix::Vector3f _gain_d;     ///< rate D gain per axis (parallel form = K * D)
	matrix::Vector3f _lim_int;    ///< integrator absolute clamp per axis
	matrix::Vector3f _gain_ff;    ///< direct rate-to-torque feed-forward gain

	// Rate-controller persistent state
	matrix::Vector3f _rate_int{};            ///< integrator term
	matrix::Vector<bool, 3> _sat_pos{};      ///< control-allocator positive saturation
	matrix::Vector<bool, 3> _sat_neg{};      ///< control-allocator negative saturation

	// Latest setpoint state (kept between updates)
	matrix::Vector3f _rates_setpoint{};
	matrix::Vector3f _thrust_setpoint{};

	// Battery scaling
	float _battery_status_scale{0.f};

	// Energy/telemetry integration
	float _energy_integration_time{0.f};
	float _control_energy[4]{};

	// Yaw torque low-pass (reduces high-frequency torque due to rotor accel)
	AlphaFilter<float> _output_lpf_yaw;

	// Cached control mode
	vehicle_control_mode_s _vehicle_control_mode{};

	hrt_abstime    _last_run{0};
	perf_counter_t _loop_perf{nullptr};
	perf_counter_t _loop_interval_perf{nullptr};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLLRATE_P>)    _param_mc_rollrate_p,
		(ParamFloat<px4::params::MC_ROLLRATE_I>)    _param_mc_rollrate_i,
		(ParamFloat<px4::params::MC_RR_INT_LIM>)    _param_mc_rr_int_lim,
		(ParamFloat<px4::params::MC_ROLLRATE_D>)    _param_mc_rollrate_d,
		(ParamFloat<px4::params::MC_ROLLRATE_FF>)   _param_mc_rollrate_ff,
		(ParamFloat<px4::params::MC_ROLLRATE_K>)    _param_mc_rollrate_k,

		(ParamFloat<px4::params::MC_PITCHRATE_P>)   _param_mc_pitchrate_p,
		(ParamFloat<px4::params::MC_PITCHRATE_I>)   _param_mc_pitchrate_i,
		(ParamFloat<px4::params::MC_PR_INT_LIM>)    _param_mc_pr_int_lim,
		(ParamFloat<px4::params::MC_PITCHRATE_D>)   _param_mc_pitchrate_d,
		(ParamFloat<px4::params::MC_PITCHRATE_FF>)  _param_mc_pitchrate_ff,
		(ParamFloat<px4::params::MC_PITCHRATE_K>)   _param_mc_pitchrate_k,

		(ParamFloat<px4::params::MC_YAWRATE_P>)     _param_mc_yawrate_p,
		(ParamFloat<px4::params::MC_YAWRATE_I>)     _param_mc_yawrate_i,
		(ParamFloat<px4::params::MC_YR_INT_LIM>)    _param_mc_yr_int_lim,
		(ParamFloat<px4::params::MC_YAWRATE_D>)     _param_mc_yawrate_d,
		(ParamFloat<px4::params::MC_YAWRATE_FF>)    _param_mc_yawrate_ff,
		(ParamFloat<px4::params::MC_YAWRATE_K>)     _param_mc_yawrate_k,
		(ParamFloat<px4::params::MC_YAW_TQ_CUTOFF>) _param_mc_yaw_tq_cutoff,

		(ParamBool<px4::params::MC_BAT_SCALE_EN>)   _param_mc_bat_scale_en
	)
};
