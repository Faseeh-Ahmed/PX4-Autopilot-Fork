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

#include "custom_fuse.h"

CustomFuse::CustomFuse() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	for (uint8_t i = 1; i <= MAX_PLS; ++i) {
		_pl_states[i] = PL_STATE::NOT_PRESENT;
		_pl_reset_sfty[i] = 0;
		_pl_mav_sent[i] = false;
	}
}

bool CustomFuse::init()
{
	ScheduleOnInterval(100_ms); // 10 Hz
	parameters_update();
	check_pylon_type();
	_max_pls = (_pylon_type == 1) ? 2 : (_pylon_type == 2) ? 10 : 0;

	if (!_pylon_type) return true;

	_prev_pylon_type = _pylon_type;
	_start_time = hrt_absolute_time();
	return true;
}

void CustomFuse::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	parameters_update();

	update_topics();

	check_pylon_type();

	if (_pylon_type != _prev_pylon_type) {
		if (!_pylon_chng_once) {
			send_info_to_gcs("PYLON TYPE CHANGED          RESTART FLIGHT CONTROLLER");
			_pylon_chng_once = true;
		}
		return;
	}
	if (!_pylon_type) return;

	if (hrt_absolute_time() - _pl_info_time > 2_s) {
		get_pl_info();
		_pl_info_time = hrt_absolute_time();
		if (!_pl_count) {
			if (!_pl_count_warn_once)
			{
				send_info_to_gcs("NO PL DETECTED              ATTACH PL TO PROCEED");
				_pl_count_warn_once = true;
			}
			return;
		}
		_pl_count_warn_once = false;
	}

	process_rc_inputs();

	safety_status_update();

	if(check_n_exec_rst()) return;

	update_states();
}

void CustomFuse::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s update;
		_parameter_update_sub.copy(&update);
		updateParams();
	}
}

bool CustomFuse::check_n_exec_rst()
{
	if (_rst_ch_state) {
		if (!_switch_rst_once) {
			send_info_to_gcs("RESET SWITCH ENGAGED.       ENGAGING ALL SAFETIES.");
			_switch_rst_once = true;
			reset_all_pls();
		}
		return true;
	}
	_switch_rst_once = false;
	return false;
}

void CustomFuse::check_pylon_type()
{
	_pylon_type = (uint8_t)_param_fuse_pylon_type.get();
	_max_pls = (_pylon_type == 1) ? 2 : (_pylon_type == 2) ? 10 : 0;
}

void CustomFuse::process_rc_inputs()
{
	const hrt_abstime now = hrt_absolute_time();

	_rst_ch = (uint8_t)_param_fuse_rst_rc_sw.get();
	_pl_slct_ch1 = (uint8_t)_param_fuse_pl_sl_rc_s1.get();
	_pl_sfty_ch1 = (uint8_t)_param_fuse_pl_sf_rc_s1.get();
	_pl_sfty_ch2 = (uint8_t)_param_fuse_pl_sf_rc_s2.get();

	// Process Reset RC channel
	_deb_rst_ch.update(rc.channels[_rst_ch - 1], now, _rc_debounce_ms);
	_rst_ch_state = _deb_rst_ch.stable;

	// Process Safety RC channels
	_deb_pl_sfty_ch1.update(rc.channels[_pl_sfty_ch1 - 1], now, _rc_debounce_ms);
	_pl_sfty_ch1_state = _deb_pl_sfty_ch1.stable;

	_deb_pl_sfty_ch2.update(rc.channels[_pl_sfty_ch2 - 1], now, _rc_debounce_ms);
	_pl_sfty_ch2_state = _deb_pl_sfty_ch2.stable;

	// Determine safety number
	if (_pl_sfty_ch2_state == 0 && _pl_sfty_ch1_state == 0) {
		_safety_num = 0;
	} else if (_pl_sfty_ch2_state == 0 && _pl_sfty_ch1_state == 1) {
		_safety_num = 1;
	} else if (_pl_sfty_ch2_state == 0 && _pl_sfty_ch1_state == 2) {
		_safety_num = 2;
	} else if (_pl_sfty_ch2_state == 1 && _pl_sfty_ch1_state == 2) {
		_safety_num = 3;
	} else if (_pl_sfty_ch2_state == 2 && _pl_sfty_ch1_state == 2) {
		_safety_num = 4;
	}

	// Process Select RC channels based on pylon type
	_deb_pl_slct_ch1.update(rc.channels[_pl_slct_ch1 - 1], now, _rc_debounce_ms);
	_pl_slct_ch1_state = _deb_pl_slct_ch1.stable;

	if (_pylon_type == 1) {
		_selected_pl = _pl_slct_ch1_state;  // 0=none, 1=PL1, 2=PL2
	} else if (_pylon_type == 2) {
		_pl_slct_ch2 = (uint8_t)_param_fuse_pl_sl_rc_s2.get();
		_pl_slct_ch3 = (uint8_t)_param_fuse_pl_sl_rc_s3.get();

		_deb_pl_slct_ch2.update(rc.channels[_pl_slct_ch2 - 1], now, _rc_debounce_ms);
		_pl_slct_ch2_state = _deb_pl_slct_ch2.stable;

		_deb_pl_slct_ch3.update(rc.channels[_pl_slct_ch3 - 1], now, _rc_debounce_ms);
		_pl_slct_ch3_state = _deb_pl_slct_ch3.stable;

		// Decode 3-channel ternary for 0-10
		uint8_t code = _pl_slct_ch3_state * 9 + _pl_slct_ch2_state * 3 + _pl_slct_ch1_state;
		_selected_pl = (code <= 10) ? code : 0;
	}

	if (_selected_pl > _max_pls) _selected_pl = 0;
}

void CustomFuse::update_states()
{
	hrt_abstime now = hrt_absolute_time();

	if (_selected_pl != _prev_selected_pl) {
		if (_prev_selected_pl != 0 && _pl_states[_prev_selected_pl] > PL_STATE::SELECTED) {
			send_info_to_gcs("PL SELECTION CHANGED.       RESETTING PREVIOUS PL.");
			reset_pl(_prev_selected_pl);
		}
		else if (_pl_states[_prev_selected_pl] == PL_STATE::SELECTED) {
			_pl_states[_prev_selected_pl] = PL_STATE::ALL_SAFETIES_ENGAGED;
		}
		if (_selected_pl != 0 && _pl_states[_prev_selected_pl] != PL_STATE::RESETTING) {
			if (_pl_states[_selected_pl] == PL_STATE::ALL_SAFETIES_ENGAGED) {
				_pl_states[_selected_pl] = PL_STATE::SELECTED;
				char msg[50];
				snprintf(msg, sizeof(msg), "PL %d SELECTED.", _selected_pl);
				send_info_to_gcs(msg);
			}
			else if (_pl_states[_selected_pl] == PL_STATE::NOT_PRESENT) {
				char msg[50];
				snprintf(msg, sizeof(msg), "PL %d NOT DETECTED.", _selected_pl);
				send_info_to_gcs(msg);
			}
		}
		else if (_selected_pl == 0) {
			send_info_to_gcs("NO PL SELECTED.");
		}
		_prev_selected_pl = _selected_pl;
	}

	// Handle selected PL progression/regression
	if (_selected_pl != 0 && _pl_states[_selected_pl] >= PL_STATE::SELECTED && _pl_states[_selected_pl] != PL_STATE::RESETTING) {
		int current_sfty = static_cast<int>(_pl_states[_selected_pl]) - static_cast<int>(PL_STATE::SELECTED);
		if (_safety_num == current_sfty + 1) {
			if (!_pl_mav_sent[_selected_pl]) {
				bool for_pylon = (_safety_num == 4);
				send_custom_mavlink_message(for_pylon, _selected_pl, _safety_num, true);  // Disengage
				_pl_mav_sent[_selected_pl] = true;
				_pl_timers[_selected_pl] = now;
			}
			if (check_ack() == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
				_bad_ack_once = false;
				char msg[50];
				snprintf(msg, sizeof(msg), "PL %d SAFETY %d DISENGAGED.", _selected_pl, _safety_num);
				send_info_to_gcs(msg);
				_pl_states[_selected_pl] = static_cast<PL_STATE>(static_cast<int>(_pl_states[_selected_pl]) + 1);
				_pl_mav_sent[_selected_pl] = false;
			} else if (check_ack() == vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED && !_bad_ack_once) {
				send_info_to_gcs("COMMAND DENIED.             RETRYING.");
				_bad_ack_once = true;
				_pl_mav_sent[_selected_pl] = false;  // Retry
			} else if (check_ack() == vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED && !_bad_ack_once) {
				send_info_to_gcs("COMMAND REJECTED TEMP.      RETRYING.");
				_bad_ack_once = true;
				_pl_mav_sent[_selected_pl] = false;  // Retry
			} else if (check_ack() == vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED && !_bad_ack_once) {
				send_info_to_gcs("COMMAND UNSUPPORTED.");
				_bad_ack_once = true;
			} else if (now - _pl_timers[_selected_pl] > MAX_MAV_CMD_TIMEOUT && check_ack() == vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS) {
				send_info_to_gcs("ACKNOWLDGEMENT TIMED OUT.   RETRYING.");
				_pl_mav_sent[_selected_pl] = false;  // Retry
			}
		} else if (_safety_num < current_sfty) {
			if (!_pl_mav_sent[_selected_pl]) {
				bool for_pylon = (current_sfty == 4);
				send_custom_mavlink_message(for_pylon, _selected_pl, current_sfty, false);  // Engage previous
				_pl_mav_sent[_selected_pl] = true;
				_pl_timers[_selected_pl] = now;
			}
			if (check_ack() == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
				_bad_ack_once = false;
				char msg[50];
				snprintf(msg, sizeof(msg), "PL %d SAFETY %d ENGAGED.", _selected_pl, current_sfty);
				send_info_to_gcs(msg);
				_pl_states[_selected_pl] = static_cast<PL_STATE>(static_cast<int>(_pl_states[_selected_pl]) - 1);
				_pl_mav_sent[_selected_pl] = false;
			} else if (check_ack() == vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED && !_bad_ack_once) {
				send_info_to_gcs("COMMAND DENIED.             RETRYING.");
				_bad_ack_once = true;
				_pl_mav_sent[_selected_pl] = false;  // Retry
			} else if (check_ack() == vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED && !_bad_ack_once) {
				send_info_to_gcs("COMMAND REJECTED TEMP.      RETRYING.");
				_bad_ack_once = true;
				_pl_mav_sent[_selected_pl] = false;  // Retry
			} else if (check_ack() == vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED && !_bad_ack_once) {
				send_info_to_gcs("COMMAND UNSUPPORTED.");
				_bad_ack_once = true;
			} else if (now - _pl_timers[_selected_pl] > MAX_MAV_CMD_TIMEOUT && check_ack() == vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS) {
				send_info_to_gcs("ACKNOWLDGEMENT TIMED OUT.   RETRYING.");
				_pl_mav_sent[_selected_pl] = false;  // Retry
			}
		}
		if (_pl_states[_selected_pl] == PL_STATE::PYLON_SFTY_DISENGAGED) {
			// Optional: Trigger release or other action
			// _pl_states[_selected_pl] = PL_STATE::ALL_SAFETIES_ENGAGED;
		}
	}

	// Handle resetting PLs
	for (uint8_t pl = 1; pl <= _max_pls; ++pl) {
		if (_pl_states[pl] == PL_STATE::RESETTING) {
			if (_pl_reset_sfty[pl] == 0) _pl_reset_sfty[pl] = MAX_SAFETIES;  // Init if first time
			uint8_t sfty = _pl_reset_sfty[pl];
			if (!_pl_mav_sent[pl]) {
				bool for_pylon = (sfty == 4);
				send_custom_mavlink_message(for_pylon, pl, sfty, false);  // Engage
				_pl_mav_sent[pl] = true;
				_pl_timers[pl] = now;
			}
			if (check_ack() == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
				_bad_ack_once = false;
				char msg[50];
				snprintf(msg, sizeof(msg), "PL %d SAFETY %d ENGAGED.", pl, sfty);
				send_info_to_gcs(msg);
				_pl_reset_sfty[pl]--;
				_pl_mav_sent[pl] = false;
				if (_pl_reset_sfty[pl] == 0) {
					_pl_states[pl] = PL_STATE::ALL_SAFETIES_ENGAGED;
					send_info_to_gcs("RESET COMPLETE FOR PL.");
				}
			} else if (check_ack() == vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED && !_bad_ack_once) {
				send_info_to_gcs("COMMAND DENIED.             RETRYING.");
				_bad_ack_once = true;
				_pl_mav_sent[_selected_pl] = false;  // Retry
			} else if (check_ack() == vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED && !_bad_ack_once) {
				send_info_to_gcs("COMMAND REJECTED TEMP.      RETRYING.");
				_bad_ack_once = true;
				_pl_mav_sent[_selected_pl] = false;  // Retry
			} else if (check_ack() == vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED && !_bad_ack_once) {
				send_info_to_gcs("COMMAND UNSUPPORTED.");
				_bad_ack_once = true;
			} else if (now - _pl_timers[_selected_pl] > MAX_MAV_CMD_TIMEOUT && check_ack() == vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS) {
				send_info_to_gcs("ACKNOWLDGEMENT TIMED OUT.   RETRYING.");
				_pl_mav_sent[_selected_pl] = false;  // Retry
			}
		}
	}
}

void CustomFuse::safety_status_update()
{
	if (!_param_fuse_soft_en.get()) {
		_safety_check_passed = true;
		return;
	}

	if (_arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		if (!_armed) {
			_armed = true;
			_arming_timestamp = hrt_absolute_time();
			_arming_x = _local_pos_x;
			_arming_y = _local_pos_y;
			_arming_z = _local_pos_z;
			_arming_lat = _global_lat;
			_arming_lon = _global_lon;
			_arming_alt = _global_alt;
			_conditions_warn_once = false;
			_home_updated = false;
		}
	} else if (_arming_state == vehicle_status_s::ARMING_STATE_DISARMED) {
		if (_armed) {
			_armed = false;
		}
	}

	if(conditions_met()) {
		_safety_check_passed = true;
		_conditions_warn_once = false;
	}
	else if (_selected_pl != 0 && _pl_states[_selected_pl] > PL_STATE::SELECTED) {
		_safety_check_passed = false;
		if (!_conditions_warn_once) {
			send_info_to_gcs("SAFETY CONDITIONS NOT MET.  ENGAGING ALL SAFETIES.");
			reset_all_pls();
			_conditions_warn_once = true;
		}
	}
	else {
		_safety_check_passed = false;
		_conditions_warn_once = false;
	}

}

bool CustomFuse::conditions_met()
{
	// This safety needs to be executed before returning the result
	// New Home Position Setting
	if (_param_fuse_soft_en.get() & 0b10000000) {
		update_home();
	}

	// Arming Check
	if (_param_fuse_soft_en.get() & 0b1) {
		if (!_armed) return false;
	}

	// Safety Time Check
	if (_param_fuse_soft_en.get() & 0b10) {
		hrt_abstime now = hrt_absolute_time();
		if (((now - _arming_timestamp < SAFETY_TIME) || _arming_timestamp <= 1) && _armed) return false;
	}

	// Horizontal Distance Check
	if (_param_fuse_soft_en.get() & 0b100) {
		float dist_xy = sqrtf((_local_pos_x - _arming_x) * (_local_pos_x - _arming_x) + (_local_pos_y - _arming_y) * (_local_pos_y - _arming_y));
		if (dist_xy < SAFETY_DISTANCE) return false;
	}

	// Vertical Distance Check
	if (_param_fuse_soft_en.get() & 0b1000) {
		float dist_z = -(_local_pos_z - _arming_z);
		if (dist_z < SAFETY_ALTITUDE) return false;
	}

	// RTH/RTL + Land Mode Check
	if (_param_fuse_soft_en.get() & 0b10000) {
		if (_nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL ||
	    	_nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) return false;
	}

	// PWM Check
	if (_param_fuse_soft_en.get() & 0b100000) {
		float pwm_min_val = (float)_param_pwm_min.get();
		float pwm_max_val = (float)_param_pwm_max.get();
		uint8_t rotor_count = (uint8_t)_param_ca_rotor_count.get();
		float min_throttle_pwm = pwm_min_val + 0.1f * (pwm_max_val - pwm_min_val); // 10% of the range

		for (size_t i = 0; i < rotor_count; ++i) {
			if (outputs.output[i] > 0.0f && outputs.output[i] < min_throttle_pwm) {
				return false;
				break;
			}
		}
	}

	// Failsafe Check
	if (_param_fuse_soft_en.get() & 0b1000000) {
		if (_failsafe) return false;
	}

	return true;
}

void CustomFuse::update_home()
{
	if (!_home_updated && _armed) {
		float delta_x = _local_pos_x - _arming_x;
		float delta_y = _local_pos_y - _arming_y;
		float dist = sqrtf(delta_x * delta_x + delta_y * delta_y);
		if (dist > 20.0f) {
			float yaw = _local_heading;
			float dir_x = cosf(yaw);
			float dir_y = sinf(yaw);
			double new_lat_val;
			double new_lon_val;
			add_vector_to_global_position(_arming_lat, _arming_lon,
				SAFETY_DISTANCE * dir_x, SAFETY_DISTANCE * dir_y,
				&new_lat_val, &new_lon_val);
			vehicle_command_s vcmd{};
			vcmd.timestamp = hrt_absolute_time();
			vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_HOME;
			vcmd.param1 = 0.f;
			vcmd.param2 = 0.f;
			vcmd.param3 = 0.f;
			vcmd.param4 = NAN;
			vcmd.param5 = (float)new_lat_val;
			vcmd.param6 = (float)new_lon_val;
			vcmd.param7 = _arming_alt;
			vcmd.target_system = 1;
			vcmd.target_component = 1;
			vcmd.source_system = 1;
			vcmd.source_component = 1;
			_vehicle_command_pub.publish(vcmd);
			_home_updated = true;
			send_info_to_gcs("NEW HOME POSITION SET 50M   AHEAD IN FLIGHT DIRECTION.");
		}
	}
}

void CustomFuse::update_topics()
{
	if (_vehicle_status_sub.updated()) {
		_vehicle_status_sub.copy(&status);
		_nav_state = status.nav_state;
		_arming_state = status.arming_state;
		_failsafe = status.failsafe;
	}


	if (_vehicle_local_position_sub.updated()) {
		_vehicle_local_position_sub.copy(&lpos);
		_local_pos_x = lpos.x;
		_local_pos_y = lpos.y;
		_local_pos_z = lpos.z;
		_local_heading = lpos.heading;
	}


	if (_vehicle_global_position_sub.updated()) {
		_vehicle_global_position_sub.copy(&gpos);
		_global_lat = gpos.lat;
		_global_lon = gpos.lon;
		_global_alt = gpos.alt;
	}

	if (_actuator_outputs_sub.updated()) {
		_actuator_outputs_sub.copy(&outputs);
	}

	if (_rc_channels_sub.updated()) {
		_rc_channels_sub.copy(&rc);
	}
}

uint8_t CustomFuse::check_ack()
{
	vehicle_command_ack_s ack;
	if (_vehicle_command_ack_sub.updated()) {
		_vehicle_command_ack_sub.copy(&ack);
		if (ack.timestamp > _last_ack_timestamp && ack.command == MAV_CMD_USER_1) {
			_last_ack_timestamp = ack.timestamp;
			return ack.result;
		}
	}
	return vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS;
}

void CustomFuse::send_info_to_gcs(const char *message)
{
	mavlink_log_emergency(&_mavlink_log_pub, "%s", message);
}

void CustomFuse::get_pl_info()
{
	vehicle_command_s vcmd{};
	vcmd.timestamp = hrt_absolute_time();
	vcmd.command = MAV_CMD_USER_2;
	vcmd.param1 = 1.0f;
	vcmd.target_system = PYLON_MAV_SYSID;
	vcmd.target_component = PYLON_MAV_COMPID;
	vcmd.source_system = 1;
	vcmd.source_component = 1;
	if (!_vehicle_command_pub.publish(vcmd)) {
		PX4_ERR("Failed to publish PL info request");
		return;
	}

	hrt_abstime start = hrt_absolute_time();
	while (hrt_absolute_time() - start < MAX_MAV_CMD_TIMEOUT) {
		vehicle_command_ack_s ack{};
		if (_vehicle_command_ack_sub.updated()) {
			_vehicle_command_ack_sub.copy(&ack);
			if (ack.command == vcmd.command && ack.timestamp > _last_ack_timestamp) {
				_last_ack_timestamp = ack.timestamp;
				if (ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
					_pl_count = static_cast<uint8_t>(ack.result_param1);
					_pl_mask = static_cast<uint32_t>(ack.result_param2);
					_pl_info_req_to_once = false;
					if ((_pl_count != _prev_pl_count) || (_pl_mask != _prev_pl_mask)) {
						send_info_to_gcs("PL COUNT UPDATED");
						_prev_pl_count = _pl_count;
						_prev_pl_mask = _pl_mask;
						// Update states based on mask (standardized: bit (pl-1))
						for (uint8_t pl = 1; pl <= _max_pls; ++pl) {
							if (_pl_mask & (1 << (pl - 1))) {
								if (_pl_states[pl] == PL_STATE::NOT_PRESENT) {
									_pl_states[pl] = PL_STATE::ALL_SAFETIES_ENGAGED;
								}
							} else {
								_pl_states[pl] = PL_STATE::NOT_PRESENT;
							}
						}
					}
					return;
				} else {
					_pl_count = 0;
					_pl_mask = 0;
					send_info_to_gcs("PL INFO MISSING.         CHECK PYLON CONNECTION.");
					return;
				}
			}
		}
		px4_usleep(100000);
	}
	_pl_count = 0;
	_pl_mask = 0;
	if (!_pl_info_req_to_once) {
		send_info_to_gcs("PL INFO REQUEST TIMED OUT.  CHECK PYLON CONNECTION.");
		_pl_info_req_to_once = true;
	}
}

void CustomFuse::send_custom_mavlink_message(bool for_pylon, uint8_t pl_num, uint8_t sfty_num, bool disengage)
{
	vehicle_command_s vcmd{};
	vcmd.timestamp = hrt_absolute_time();
	vcmd.command = MAV_CMD_USER_1;
	vcmd.param1 = for_pylon ? 1.0f : 0.0f;
	vcmd.param2 = pl_num;
	vcmd.param3 = sfty_num;
	vcmd.param4 = disengage ? 1.0f : 0.0f;
	vcmd.target_system = PYLON_MAV_SYSID;
	vcmd.target_component = PYLON_MAV_COMPID;
	vcmd.source_system = 1;
	vcmd.source_component = 1;

	if (_vehicle_command_pub.publish(vcmd)) {
		PX4_INFO("Sent message: For Pylon? %d, PL: %d, Safety: %d, Disengage: %d", for_pylon, pl_num, sfty_num, disengage);
	} else {
		PX4_ERR("Failed to publish command");
	}
}

void CustomFuse::reset_pl(uint8_t pl_num)
{
	if (pl_num == 0
		 || pl_num > _max_pls
		 || _pl_states[pl_num] <= PL_STATE::SELECTED) return;

	while (_safety_num > 0)
	{
		if(!_safety_switch_warn_once)
		{
			send_info_to_gcs("RESET SAFETY SWITCHES       TO PROCEED WITH RESET.");
			_safety_switch_warn_once = true;
		}
		update_topics();
		process_rc_inputs();
		px4_usleep(1000);
	}
	_safety_switch_warn_once = false;

	_pl_reset_sfty[pl_num] = static_cast<int>(_pl_states[pl_num]) - static_cast<int>(PL_STATE::SELECTED);
	_pl_states[pl_num] = PL_STATE::RESETTING;
	_pl_mav_sent[pl_num] = false;
	_pl_timers[pl_num] = hrt_absolute_time();
	char msg[50];
	snprintf(msg, sizeof(msg), "INITIATING RESET FOR PL %d.", pl_num);
	send_info_to_gcs(msg);
}

void CustomFuse::reset_all_pls()
{
	for (uint8_t pl = 1; pl <= _max_pls; ++pl) {
		if (_pl_states[pl] > PL_STATE::SELECTED) {
			reset_pl(pl);
		}
	}
}

int CustomFuse::print_status()
{
	PX4_INFO("Custom Fuse Module Status:");
	PX4_INFO("  Running: %s", is_running() ? "YES" : "NO");
	PX4_INFO("  Pylon Type: %s", (_pylon_type == 1) ? "A" : (_pylon_type == 2) ? "B" : "Not Selected");
	PX4_INFO("  Max PLs: %d", _max_pls);
	PX4_INFO("  Selected Payload: %d", _selected_pl);
	PX4_INFO("  Safety Disengaged: %d", _safety_num);
	PX4_INFO("  Total Payloads Detected: %d", _pl_count);
	PX4_INFO("  Payload Mask: 0x%03lX", _pl_mask);
	for (uint8_t pl = 1; pl <= _max_pls; ++pl) {
		if (_pl_states[pl] != PL_STATE::NOT_PRESENT) {
			PX4_INFO("  PL %d State: %d", pl, static_cast<int>(_pl_states[pl]));
		}
	}
	return PX4_OK;
}

void CustomFuse::stop()
{
	if (_object.load()) {
		CustomFuse *instance = _object.load();
		instance->ScheduleClear();
		instance->exit_and_cleanup();
	}
}

int CustomFuse::custom_command(int argc, char *argv[])
{
	if (argc < 1) {
		return print_usage("Missing command");
	}

	CustomFuse *instance = _object.load();
	if (!instance) {
		PX4_ERR("Module not running");
		return PX4_ERROR;
	}

	if (!strcmp(argv[0], "status")) {
		return instance->print_status();
	}

	if (!strcmp(argv[0], "reset")) {
		instance->reset_all_pls();
		return PX4_OK;
	}

	if (!strcmp(argv[0], "stop")) {
		instance->stop();
		return PX4_OK;
	}

	return print_usage("Unrecognized command");
}

int CustomFuse::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Custom module to keep track of safety conditions and send custom MAVLink commands to Pylon.
Sends PL number, safety number and engage/disengage command.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("custom_fuse", "command");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print module status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset all PL states and engage safeties");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int CustomFuse::task_spawn(int argc, char *argv[])
{
	CustomFuse *instance = new CustomFuse();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("Alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

extern "C" __EXPORT int custom_fuse_main(int argc, char *argv[])
{
	return CustomFuse::main(argc, argv);
}
