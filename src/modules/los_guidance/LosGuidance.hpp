/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/los_measurements.h>
#include <uORB/topics/offboard_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>

using namespace time_literals;

class LosGuidance : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	static ModuleBase::Descriptor desc;

	LosGuidance();
	~LosGuidance() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;

	void parameters_updated();
	bool compute_acceleration_command_ned(matrix::Vector3f &acceleration_ned) const;
	void publish_offboard_setpoint(const matrix::Vector3f &acceleration_ned);
	void publish_hold_setpoint();

	uORB::SubscriptionCallbackWorkItem _los_measurements_sub{this, ORB_ID(los_measurements)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	uORB::Publication<offboard_control_mode_s> _offboard_control_mode_pub{ORB_ID(offboard_control_mode)};
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};

	los_measurements_s _latest_sample{};
	bool _has_sample{false};
	hrt_abstime _last_sample_timestamp{0};

	matrix::Quatf _q_attitude{1.f, 0.f, 0.f, 0.f};
	bool _has_attitude{false};

	matrix::Vector3f _latched_position_ned{0.f, 0.f, 0.f};
	bool _has_latched_position{false};

	hrt_abstime _last_publish_timestamp{0};
	uint64_t _samples_received{0};
	uint64_t _setpoints_published{0};
	uint64_t _hold_setpoints_published{0};
	uint64_t _accel_compute_failures{0};
	uint64_t _early_return_disabled{0};
	uint64_t _early_return_no_fresh_los{0};

	hrt_abstime _schedule_interval_us{1'000'000 / 50};

	DEFINE_PARAMETERS(
		(ParamBool<px4::params::LOS_GD_EN>) _param_los_gd_en,
		(ParamFloat<px4::params::LOS_GD_ACC_MAX>) _param_los_gd_acc_max,
		(ParamFloat<px4::params::LOS_GD_GIMB_PIT>) _param_los_gd_gimb_pit,
		(ParamFloat<px4::params::LOS_GD_GIMB_YAW>) _param_los_gd_gimb_yaw,
		(ParamInt<px4::params::LOS_GD_TIMEOUT>) _param_los_gd_timeout_ms,
		(ParamInt<px4::params::LOS_GD_PUB_HZ>) _param_los_gd_pub_hz
	)
};
