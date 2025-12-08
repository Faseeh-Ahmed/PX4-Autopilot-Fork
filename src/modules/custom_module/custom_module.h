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

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
// #include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <systemlib/mavlink_log.h>
#include <string.h>
#include <stdlib.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/parameter_update.h>

#include <lib/systemlib/mavlink_log.h>

#define SAFETY_TIME 35_s

#define SAFETY_DISTANCE 50.0f
#define SAFETY_ALTITUDE 50.0f

using namespace time_literals;

class Mavlink;

class CustomModule : public ModuleBase<CustomModule>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	CustomModule();
	~CustomModule() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	void parameters_update();

	void send_info_to_gcs(const char *message);

	bool conditions_met();

	void safety_status_update();

	void update_home();

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::PWM_MAIN_MIN1>) 			_param_pwm_min,
		(ParamInt<px4::params::PWM_MAIN_MAX1>) 			_param_pwm_max,
		(ParamInt<px4::params::CA_ROTOR_COUNT>)			_param_ca_rotor_count,
		(ParamFloat<px4::params::SAFETY_ENG_DIST>) 		_param_safety_eng_dist,
		(ParamInt<px4::params::SAFETY_RC_CH>) 			_param_safety_rc_ch,
		(ParamInt<px4::params::SAFETY_SOFT_EN>) 		_param_safety_soft_en
	)

	// Subscriptions
	uORB::Subscription _rc_channels_sub{ORB_ID(rc_channels)};
	uORB::Subscription _distance_sensor_sub{ORB_ID(distance_sensor)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	// Publications
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

	orb_advert_t _mavlink_log_pub{nullptr};
	bool _servo_high{false};

	uint8_t _nav_state{0};
	uint8_t _arming_state{0};

	float _local_pos_x{0.0f};
	float _local_pos_y{0.0f};
	float _local_pos_z{0.0f};
	float _local_heading{0.0f};

	bool _armed{false};
	hrt_abstime _arming_timestamp{0};
	float _arming_x{0.0};
	float _arming_y{0.0};
	float _arming_z{0.0f};

	double _global_lat{0.0};
	double _global_lon{0.0};
	double _global_alt{0.0};

	double _arming_lat{0.0};
	double _arming_lon{0.0};
	float _arming_alt{0.0f};

	bool _home_updated{false};
	bool _conditions_warn_once{false};
	bool _safety_check_passed{false};
	bool _rc_high_once{false};
	bool _failsafe_warn_once{false};
};
