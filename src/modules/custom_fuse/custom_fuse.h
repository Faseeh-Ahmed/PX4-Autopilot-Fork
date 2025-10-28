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

/**
 * @file custom_fuse.h
 * @brief Custom MAVLink module for sending servo commands to STM32
 */
#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <drivers/drv_hrt.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/parameter_update.h>

#include <lib/systemlib/mavlink_log.h>

#define PL1_RC_SWITCH_CH 6
#define PL2_RC_SWITCH_CH 6
#define PL3_RC_SWITCH_CH 8
#define PYLON_RC_SWITCH_CH 5
#define RESET_RC_SWITCH_CH 7

#define PL1_SERVO_NUM 2
#define PL2_SERVO_NUM 3
#define PL3_SERVO_NUM 4
#define PYLON_SERVO_NUM 1

#define SAFETY_TIME 35_s

#define SAFETY_DISTANCE 50.0f
#define SAFETY_ALTITUDE 50.0f

#define MC1_MAV_SYSID 100
#define MC1_MAV_COMPID 200
#define MC2_MAV_SYSID 101
#define MC2_MAV_COMPID 201

using namespace time_literals;

class Mavlink;

/**
 * @brief Custom Fuse Module
 *
 * This module sends a MAVLink command (via vehicle_command) to engage/disengage a servo on the connected STM32.
 */
class CustomFuse : public ModuleBase<CustomFuse>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	CustomFuse();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	int print_status() override;

	bool init();

	void Run() override;
	void stop();

private:

	enum class ModuleState {
		ALL_SAFETIES_ENGAGED = 0,
		PL1_SAFETY_DISENGAGED,
		PL2_SAFETY_DISENGAGED,
		PL3_SAFETY_DISENGAGED,
		PYLON_SAFETY_DISENGAGED,
	};

	static constexpr uint16_t MAV_CMD_USER_1 = 31010;

	void operate_servo(uint8_t servo_num, bool disengage);

	void sendInfoToGCS(const char *message);

	void updateStates();

	int reset_state();

	void check_ack();

	bool conditions_met();

	void parameters_update();

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::PWM_MAIN_MIN1>) _param_pwm_min,
		(ParamInt<px4::params::PWM_MAIN_MAX1>) _param_pwm_max
	)

	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_command_ack_sub{ORB_ID(vehicle_command_ack)};
	uORB::Subscription _rc_channels_sub{ORB_ID(rc_channels)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	ModuleState _state{ModuleState::ALL_SAFETIES_ENGAGED};

	uint64_t _last_ack_timestamp{0};
	orb_advert_t _mavlink_log_pub{nullptr};
	uint8_t _servo_commanded{0};
	bool _pending_disengage{false};

	bool _pylon_safety_engaged{true};
	bool _pl1_safety_engaged{true};
	bool _pl2_safety_engaged{true};
	bool _pl3_safety_engaged{true};

	bool _pylon_safety_ack{false};
	bool _pl1_safety_ack{false};
	bool _pl2_safety_ack{false};
	bool _pl3_safety_ack{false};

	bool _pylon_rc_engaged{true};
	bool _pl1_rc_engaged{true};
	bool _pl2_rc_engaged{true};
	bool _pl3_rc_engaged{true};

	bool _reset_rc_pressed{false};

	bool _reset_warn_once{false};
	bool _conditions_warn_once{false};

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
};
