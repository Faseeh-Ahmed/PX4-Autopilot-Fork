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
 * @brief Custom MAVLink module for sending safety engagement/disengagement commands to Pylon.
 */
#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <systemlib/mavlink_log.h>
#include <string.h>
#include <stdlib.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>

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

#define SAFETY_TIME 35_s

#define SAFETY_DISTANCE 50.0f
#define SAFETY_ALTITUDE 50.0f

#define PYLON_MAV_SYSID 100
#define PYLON_MAV_COMPID 200

#define RC_THRESHOLD 0.5f

#define MAX_MAV_CMD_TIMEOUT 5_s
#define RESET_TIMEOUT 15_s

#define MAX_SAFETIES 4  // Safeties 1-3 + Pylon safety (4)

using namespace time_literals;

class Mavlink;

/**
 * @brief Custom Fuse Module
 *
 * This module sends a MAVLink command to engage/disengage a servo on the connected Pylon.
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
	enum class PL_STATE {
		NOT_PRESENT = 0,
		ALL_SAFETIES_ENGAGED,
		SELECTED,
		SFTY1_DISENGAGED,
		SFTY2_DISENGAGED,
		SFTY3_DISENGAGED,
		PYLON_SFTY_DISENGAGED,
		RESETTING
	};

	static constexpr uint8_t MAX_PLS = 10;  // Scalable maximum payloads

	static constexpr uint16_t MAV_CMD_USER_1 = 31010;
	static constexpr uint16_t MAV_CMD_USER_2 = 31011;

	void send_custom_mavlink_message(bool for_pylon, uint8_t pl_num, uint8_t pl_sfty_num, bool disengage);

	void send_info_to_gcs(const char *message);

	void check_pylon_type();

	void get_pl_info();

	void reset_pl(uint8_t pl_num);  // Initiate reset for specific PL (non-blocking)
	void reset_all_pls();  // Initiate reset for all attached PLs

	uint8_t check_ack();

	bool conditions_met();

	void parameters_update();

	void update_topics();

	void update_home();

	bool check_n_exec_rst();

	void process_rc_inputs();  // Unified for both types

	void update_states();  // Handle state updates non-blockingly

	void safety_status_update();

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::PWM_MAIN_MIN1>) 		_param_pwm_min,
		(ParamInt<px4::params::PWM_MAIN_MAX1>) 		_param_pwm_max,
		(ParamInt<px4::params::CA_ROTOR_COUNT>)		_param_ca_rotor_count,
		(ParamInt<px4::params::FUSE_PYLON_TYPE>)	_param_fuse_pylon_type,
		(ParamInt<px4::params::FUSE_RST_RC_SW>)		_param_fuse_rst_rc_sw,
		(ParamInt<px4::params::FUSE_PL_SL_RC_S1>)	_param_fuse_pl_sl_rc_s1,
		(ParamInt<px4::params::FUSE_PL_SL_RC_S2>)	_param_fuse_pl_sl_rc_s2,
		(ParamInt<px4::params::FUSE_PL_SL_RC_S3>)	_param_fuse_pl_sl_rc_s3,
		(ParamInt<px4::params::FUSE_PL_SF_RC_S1>)	_param_fuse_pl_sf_rc_s1,
		(ParamInt<px4::params::FUSE_PL_SF_RC_S2>)	_param_fuse_pl_sf_rc_s2,
		(ParamInt<px4::params::FUSE_SOFT_EN>)		_param_fuse_soft_en
	)

	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};
	uORB::Subscription _vehicle_command_ack_sub{ORB_ID(vehicle_command_ack)};
	uORB::Subscription _rc_channels_sub{ORB_ID(rc_channels)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _actuator_outputs_sub{ORB_ID(actuator_outputs)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	vehicle_status_s status{};
	vehicle_global_position_s gpos{};
	vehicle_local_position_s lpos{};
	rc_channels_s rc{};
	actuator_outputs_s outputs{};

	uint64_t _last_ack_timestamp{0};
	// bool _awaiting_ack{false};
	orb_advert_t _mavlink_log_pub{nullptr};
	// bool _pending_disengage{false};

	uint8_t _rst_ch{7};
	uint8_t _pl_slct_ch1{8};
	uint8_t _pl_slct_ch2{9};
	uint8_t _pl_slct_ch3{10};
	uint8_t _pl_sfty_ch1{11};
	uint8_t _pl_sfty_ch2{12};

	bool _rst_ch_state{false};
	uint8_t _pl_slct_ch1_state{0};
	uint8_t _pl_slct_ch2_state{0};
	uint8_t _pl_slct_ch3_state{0};
	uint8_t _pl_sfty_ch1_state{0};
	uint8_t _pl_sfty_ch2_state{0};

	uint8_t _pylon_type{0};
	uint8_t _prev_pylon_type{0};
	uint8_t _selected_pl{0};
	uint8_t _prev_selected_pl{0};
	uint8_t _safety_num{0};

	uint8_t _pl_count{0};
	uint32_t _pl_mask{0};
	uint8_t _prev_pl_count{0};
	uint32_t _prev_pl_mask{0};

	bool _safety_switch_warn_once{false};
	bool _conditions_warn_once{false};
	bool _switch_rst_once{false};
	bool _mav_msg_once{false};
	bool _pl_count_warn_once{false};
	bool _pl_mask_warn_once{false};
	bool _pylon_chng_once{false};
	bool _pl_info_req_to_once{false};
	bool _bad_ack_once{false};

	hrt_abstime _mav_command_timeout_counter{0};
	hrt_abstime _start_time{0};
	hrt_abstime _pl_info_time{0};

	uint8_t _nav_state{0};
	uint8_t _arming_state{0};
	bool _failsafe{false};

	float _local_pos_x{0.0f};
	float _local_pos_y{0.0f};
	float _local_pos_z{0.0f};
	float _local_heading{0.0f};

	/* Debouncing helpers for RC switches */
	static constexpr uint32_t DEFAULT_RC_DEBOUNCE_MS = 1000; // ms
	uint32_t _rc_debounce_ms{DEFAULT_RC_DEBOUNCE_MS};

	struct DebouncedTernarySwitch {
		uint8_t stable{0};
		uint8_t candidate{0};
		hrt_abstime since{0};

		inline uint8_t raw_from_val(float val) const {
			if (val < -RC_THRESHOLD) return 0;
			if (val > RC_THRESHOLD) return 2;
			return 1;
		}

		inline bool update(float val, hrt_abstime now, uint32_t debounce_ms) {
			uint8_t r = raw_from_val(val);
			if (r != candidate) {
				candidate = r;
				since = now;
			}
			if (candidate != stable && (now - since) >= (hrt_abstime)(debounce_ms * 1000u)) {
				stable = candidate;
				return true;
			}
			return false;
		}
	};

	struct DebouncedBinarySwitch {
		bool stable{false};
		bool candidate{false};
		hrt_abstime since{0};

		inline bool raw_from_val(float val) const {
			return val > RC_THRESHOLD;
		}

		inline bool update(float val, hrt_abstime now, uint32_t debounce_ms) {
			bool r = raw_from_val(val);
			if (r != candidate) {
				candidate = r;
				since = now;
			}
			if (candidate != stable && (now - since) >= (hrt_abstime)(debounce_ms * 1000u)) {
				stable = candidate;
				return true;
			}
			return false;
		}
	};

	// Per-channel debouncers
	DebouncedBinarySwitch _deb_rst_ch;
	DebouncedTernarySwitch _deb_pl_slct_ch1;
	DebouncedTernarySwitch _deb_pl_slct_ch2;
	DebouncedBinarySwitch _deb_pl_slct_ch3;
	DebouncedTernarySwitch _deb_pl_sfty_ch1;
	DebouncedTernarySwitch _deb_pl_sfty_ch2;

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
	bool _safety_check_passed{false};

	// Per-PL state management
	PL_STATE _pl_states[MAX_PLS + 1]{};  // Index 1 to MAX_PLS
	hrt_abstime _pl_timers[MAX_PLS + 1]{};  // For timeouts/retries
	uint8_t _pl_reset_sfty[MAX_PLS + 1]{};  // Current safety being reset (starts at 4)
	bool _pl_mav_sent[MAX_PLS + 1]{};  // Flag if MAV command sent, waiting for ack
	uint8_t _max_pls{0};  // 2 for type 1, 10 for type 2
};
