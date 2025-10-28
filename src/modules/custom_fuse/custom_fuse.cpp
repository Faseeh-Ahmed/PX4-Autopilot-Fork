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

#include <drivers/drv_hrt.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <systemlib/mavlink_log.h>
#include <string.h>
#include <stdlib.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>

CustomFuse::CustomFuse() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool CustomFuse::init()
{
	ScheduleOnInterval(100_ms);
	reset_state();
	return true;
}

void CustomFuse::Run()
{
	parameters_update();
	updateStates();
}

void CustomFuse::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s update;
		_parameter_update_sub.copy(&update);
		updateParams();
	}
}

void CustomFuse::updateStates()
{

	// Handle Acks
	check_ack();

	vehicle_status_s status{};
	bool status_updated = _vehicle_status_sub.updated();
	if (status_updated) {
		// PX4_INFO("STATUS UPDATED 1");
		_vehicle_status_sub.copy(&status);
		_nav_state = status.nav_state;
		_arming_state = status.arming_state;
	}

	vehicle_local_position_s pos{};
	bool pos_updated = _vehicle_local_position_sub.updated();
	if (pos_updated) {
		_vehicle_local_position_sub.copy(&pos);
		_local_pos_x = pos.x;
		_local_pos_y = pos.y;
		_local_pos_z = pos.z;
		_local_heading = pos.heading;
	}

	vehicle_global_position_s global_pos{};
	bool global_pos_updated = _vehicle_global_position_sub.updated();
	if (global_pos_updated) {
		_vehicle_global_position_sub.copy(&global_pos);
		_global_lat = global_pos.lat;
		_global_lon = global_pos.lon;
		_global_alt = global_pos.alt;
	}

	static hrt_abstime _last_disarm_time{0};

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
			PX4_INFO("Armed at lat: %.2f, lon: %.2f, alt: %.2f, time: %0.2f", _arming_lat, _arming_lon, (double)_arming_alt, (double)_arming_timestamp);
			_conditions_warn_once = false;
			_home_updated = false;
		}
	} else if (_arming_state == vehicle_status_s::ARMING_STATE_DISARMED) {
		if (_armed) {
			if (hrt_absolute_time() - _last_disarm_time > 1_s) {  // Debounce 1s
    				sendInfoToGCS("DISARMED.                   ENGAGING ALL SAFETIES.");
    				_last_disarm_time = hrt_absolute_time();
			}
			_armed = false;
			reset_state();
			return;
		}
	}

	if (status.failsafe && _state != ModuleState::ALL_SAFETIES_ENGAGED) {
		sendInfoToGCS("FAILSAFE ACTIVATED.         ENGAGING ALL SAFETIES.");
		reset_state();
		return;
	}

	if (!conditions_met() && _state != ModuleState::ALL_SAFETIES_ENGAGED) {
		sendInfoToGCS("SAFETY CONDITIONS NOT MET.  ENGAGING ALL SAFETIES.");
		reset_state();
		return;
	}

	if (!_home_updated && _armed) {
		// Calculate distance from arming location using local position
		float delta_x = _local_pos_x - _arming_x;
		float delta_y = _local_pos_y - _arming_y;
		float dist = sqrtf(delta_x * delta_x + delta_y * delta_y);
		if (dist > 20.0f) {
			// Calculate new home position SAFETY_DISTANCE ahead in current heading
			float yaw = _local_heading; // radians, -PI..+PI
			float dir_x = cosf(yaw);
			float dir_y = sinf(yaw);
			// Convert local offset to global lat/lon
			double new_lat_val;
			double new_lon_val;
			double *new_lat = &new_lat_val;
			double *new_lon = &new_lon_val;
			add_vector_to_global_position(_arming_lat, _arming_lon,
				SAFETY_DISTANCE * dir_x, SAFETY_DISTANCE * dir_y,
				new_lat, new_lon);
			vehicle_command_s vcmd{};
			vcmd.timestamp = hrt_absolute_time();
			vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_HOME;
			vcmd.param1 = 0.f; // use specified location
			vcmd.param2 = 0.f;
			vcmd.param3 = 0.f;
			vcmd.param4 = NAN; // yaw unchanged
			vcmd.param5 = (float)*new_lat;
			vcmd.param6 = (float)*new_lon;
			vcmd.param7 = _arming_alt;
			vcmd.target_system = 1;
			vcmd.target_component = 1;
			vcmd.source_system = 1;
			vcmd.source_component = 1;
			_vehicle_command_pub.publish(vcmd);
			_home_updated = true;
			sendInfoToGCS("NEW HOME POSITION SET 50M   AHEAD IN FLIGHT DIRECTION.");
		}
	}

	// Handle RC inputs
	rc_channels_s rc{};
	if (_rc_channels_sub.updated()) {
		_rc_channels_sub.copy(&rc);

		_pl2_rc_engaged = (rc.channels[PL2_RC_SWITCH_CH - 1] < 0.5f) ? true : false;
		_pl1_rc_engaged = (rc.channels[PL1_RC_SWITCH_CH - 1] < -0.5f) ? true : false;
		// _pl3_rc_engaged = (rc.channels[PL3_RC_SWITCH_CH - 1] < 0.0f) ? true : false;
		_pylon_rc_engaged = (rc.channels[PYLON_RC_SWITCH_CH - 1] < 0.0f) ? true : false;
		_reset_rc_pressed = (rc.channels[RESET_RC_SWITCH_CH - 1] > 0.0f) ? true : false;

		static hrt_abstime _last_reset_time{0};

		if (_reset_rc_pressed) {
			if (hrt_absolute_time() - _last_reset_time > 3_s) {  // Debounce 3s
				sendInfoToGCS("RESET SWITCH ENGAGED.       ENGAGING ALL SAFETIES.");
				reset_state();
				_last_reset_time = hrt_absolute_time();
			} else {
				return;
			}
		}

		switch (_state)
		{
		case ModuleState::ALL_SAFETIES_ENGAGED:
			if (_pylon_rc_engaged && _pl2_rc_engaged) {
				if (!_pl1_rc_engaged)
				{
					if (conditions_met()) {
						operate_servo(PL1_SERVO_NUM, true); //disengage
						check_ack();
						if (_pl1_safety_ack) {
							_reset_warn_once = false;
							_conditions_warn_once = false;
							sendInfoToGCS("PL 1 SAFETY DISENGAGED.");
							_state = ModuleState::PL1_SAFETY_DISENGAGED;
						}
					} else if (!_conditions_warn_once) {
						sendInfoToGCS("SAFETY CONDITIONS NOT MET   FOR DISENGAGEMENT.");
						_conditions_warn_once = true;
					}
				}
			}
			else if (!_reset_warn_once) {
				sendInfoToGCS("RESET PYLON OR PL 2 SAFETY  SWITCH TO PROCEED.");
				_reset_warn_once = true;
			}
			break;

		case ModuleState::PL1_SAFETY_DISENGAGED:
			if (_pylon_rc_engaged && !_pl1_rc_engaged) {
				if (!_pl2_rc_engaged)
				{
					if (conditions_met()) {
						operate_servo(PL2_SERVO_NUM, true); //disengage
						check_ack();
						if (_pl2_safety_ack) {
							_reset_warn_once = false;
							_conditions_warn_once = false;
							sendInfoToGCS("PL 2 SAFETY DISENGAGED.");
							_state = ModuleState::PL2_SAFETY_DISENGAGED;
							}
					} else if (!_conditions_warn_once) {
						sendInfoToGCS("SAFETY CONDITIONS NOT MET   FOR DISENGAGEMENT.");
						_conditions_warn_once = true;
					}
				}
			}
			else if (!_reset_warn_once) {
				sendInfoToGCS("RESET PYLON SERVO SWITCH    TO PROCEED.");
				_reset_warn_once = true;
			}

			if (_pl1_rc_engaged) {
				operate_servo(PL1_SERVO_NUM, false); //engage
				check_ack();
				if (_pl1_safety_ack) {
					_pl1_safety_ack = false;
					sendInfoToGCS("PL 1 SAFETY ENGAGED.");
					_state = ModuleState::ALL_SAFETIES_ENGAGED;
				}
			}
			break;

		case ModuleState::PL2_SAFETY_DISENGAGED:
			if (!_pl2_rc_engaged && !_pl1_rc_engaged) {
				if (!_pylon_rc_engaged)
				{
					if (conditions_met()) {
						operate_servo(PYLON_SERVO_NUM, true); //disengage
						check_ack();
						if (_pylon_safety_ack) {
							_reset_warn_once = false;
							_conditions_warn_once = false;
							sendInfoToGCS("PYLON SAFETY DISENGAGED.");
							_state = ModuleState::PYLON_SAFETY_DISENGAGED;
						}
					} else if (!_conditions_warn_once) {
						sendInfoToGCS("SAFETY CONDITIONS NOT MET   FOR DISENGAGEMENT.");
						_conditions_warn_once = true;
					}
				}
			}

			if (_pl2_rc_engaged) {
				operate_servo(PL2_SERVO_NUM, false); //engage
				check_ack();
				if (_pl2_safety_ack) {
					_pl2_safety_ack = false;
					sendInfoToGCS("PL 2 SAFETY ENGAGED.");
					_state = ModuleState::PL1_SAFETY_DISENGAGED;
				}
			}
			break;

		case ModuleState::PYLON_SAFETY_DISENGAGED:
			// if (!_pl2_rc_engaged && !_pl1_rc_engaged && !_pylon_rc_engaged && !_reset_warn_once) {
			// 	sendInfoToGCS("ALL SAFETIES DISENGAGED.");
			// 	_reset_warn_once = true;
			// }

			if (_pylon_rc_engaged) {
				operate_servo(PYLON_SERVO_NUM, false); //engage
				check_ack();
				if (_pylon_safety_ack) {
					_pylon_safety_ack = false;
					sendInfoToGCS("PYLON SAFETY ENGAGED.");
					_state = ModuleState::PL2_SAFETY_DISENGAGED;
				}
			}
			break;
		}
	}
}

bool CustomFuse::conditions_met()
{
	// only for debugging
	return true;

	if (!_armed) {
		return false;
	}
	PX4_INFO("ARMING CHECK PASSED");

	hrt_abstime now = hrt_absolute_time();
	if (now - _arming_timestamp < SAFETY_TIME) {
		return false;
	}
	PX4_INFO("SAFETY TIME CHECK PASSED");


	if ( (_nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL)  ||
		     (_nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) ) {
			PX4_INFO("FAILED: VEHICLE FLIGHT MODE CHECK");
			return false;
	}
	PX4_INFO("VEHICLE FLIGHT MODE CHECK PASSED");


	float dist_xy = 0.0f;
	float dist_z = 0.0f;
	dist_xy = sqrtf(((_local_pos_x - _arming_x)*(_local_pos_x - _arming_x)) +  (_local_pos_y - _arming_y)*(_local_pos_y - _arming_y));
	dist_z = -(_local_pos_z - _arming_z);

	if (dist_xy < SAFETY_DISTANCE) {
		return false;
	}
	PX4_INFO("SAFE DISTANCE CHECK PASSED");

	if (dist_z < SAFETY_ALTITUDE) {
		return false;
	}
	PX4_INFO("SAFE ALTITUDE CHECK PASSED");

	actuator_outputs_s outputs{};
	if (_actuator_outputs_sub.updated()) {
		PX4_INFO("ACTUATOR OUTPUTS UPDATED");
		_actuator_outputs_sub.copy(&outputs);
	}

	float pwm_min_val = (float)_param_pwm_min.get();
	float pwm_max_val = (float)_param_pwm_max.get();
	float min_throttle_pwm = pwm_min_val + 0.1f * (pwm_max_val - pwm_min_val);

	bool motors_ok = true;
	for (size_t i = 0; i < 4; ++i) {  // Assuming quadcopter, adjust if needed
		if (outputs.output[i] > 0.0f && outputs.output[i] < min_throttle_pwm) {
			motors_ok = false;
			break;
		}
	}

	if (!motors_ok) {
		return false;
	}
	// PX4_INFO("MOTOR PWM OUTPUT CHECK PASSED");

	return true;
}

void CustomFuse::check_ack()
{
	// Handle acknowledgments
	vehicle_command_ack_s ack;
	if (_vehicle_command_ack_sub.updated()) {
		_vehicle_command_ack_sub.copy(&ack);
		if (ack.timestamp > _last_ack_timestamp && ack.command == MAV_CMD_USER_1) {
			_last_ack_timestamp = ack.timestamp;
			if (ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
				bool disengage = _pending_disengage;
				switch (_servo_commanded) {
				case 1:
					_pylon_safety_engaged = !disengage;
					_pylon_safety_ack = true;
					break;
				case 2:
					_pl1_safety_engaged = !disengage;
					_pl1_safety_ack = true;
					break;
				case 3:
					_pl2_safety_engaged = !disengage;
					_pl2_safety_ack = true;
					break;
				default:
					break;
				}
				PX4_INFO("Servo command (target sys %d, comp %d) accepted", ack.target_system, ack.target_component);
			} else if (ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED) {
				PX4_ERR("Servo command (target sys %d, comp %d) denied", ack.target_system, ack.target_component);
				sendInfoToGCS("SERVO COMMAND DENIED");
			} else {
				PX4_INFO("Servo command (target sys %d, comp %d) result: %d", ack.target_system, ack.target_component, ack.result);
				sendInfoToGCS("SERVO COMMAND STATUS N/A");
			}
		}
	}
}

void CustomFuse::sendInfoToGCS(const char *message)
{
	mavlink_log_emergency(&_mavlink_log_pub, "%s", message);
}

void CustomFuse::operate_servo(uint8_t servo_num, bool disengage)
{
	_servo_commanded = servo_num;
	_pending_disengage = disengage;

	if (servo_num < 1 || servo_num > 3) {
		PX4_ERR("Invalid servo number: %d (must be 1-3)", servo_num);
		return;
	}

	uint8_t target_system = MC1_MAV_SYSID;
	uint8_t target_component = MC1_MAV_COMPID;
	float param1 = servo_num;
	float param2 = disengage;

	if (servo_num > 1) {
		target_system = MC2_MAV_SYSID;
		target_component = MC2_MAV_COMPID;
		param1 = servo_num - 1;
	}

	vehicle_command_s vcmd{};
	vcmd.timestamp = hrt_absolute_time();
	vcmd.command = MAV_CMD_USER_1;			// Custom command
	vcmd.param1 = param1;				// Adjusted servo number
	vcmd.param2 = param2;				// Engage/Disengage Command
	vcmd.target_system = target_system;		// Appropriate system ID
	vcmd.target_component = target_component;	// Appropriate component ID
	vcmd.source_system = 1;				// PX4 system ID
	vcmd.source_component = 1;			// PX4 component ID

	if (_vehicle_command_pub.publish(vcmd)) {
		PX4_INFO("Sent servo command: servo %d, disengage %d", servo_num, disengage);
	} else {
		PX4_ERR("Failed to publish servo command");
	}
}

int CustomFuse::reset_state()
{
	// Engage all servos
	operate_servo(PYLON_SERVO_NUM, false);
	px4_usleep(200000); // 0.2 seconds
	operate_servo(PL1_SERVO_NUM, false);
	px4_usleep(200000); // 0.2 seconds
	operate_servo(PL2_SERVO_NUM, false);
	px4_usleep(200000); // 0.2 seconds

	// Reset engagement flags
	_pylon_safety_engaged = true;
	_pl1_safety_engaged = true;
	_pl2_safety_engaged = true;
	_pylon_rc_engaged = true;
	_pl1_rc_engaged = true;
	_pl2_rc_engaged = true;

	_pylon_safety_ack = false;
	_pl1_safety_ack = false;
	_pl2_safety_ack = false;

	// Reset state machine
	_state = ModuleState::ALL_SAFETIES_ENGAGED;
	_servo_commanded = 0;
	_last_ack_timestamp = 0;
	_reset_warn_once = false;
	_conditions_warn_once = false;

	// sendInfoToGCS("SYSTEM STATE RESET. ALL SAFETIES ENGAGED.");
	return PX4_OK;
}

int CustomFuse::print_status()
{
    	PX4_INFO("Custom Fuse Mav Module Status:");
    	PX4_INFO("  Running: %s", is_running() ? "YES" : "NO");
	PX4_INFO("  Last servo commanded: %d", _servo_commanded);
	PX4_INFO("  Current State: %d", static_cast<int>(_state));
	return PX4_OK;
}

void CustomFuse::stop()
{
    	if (_object.load()) {
    		CustomFuse *instance = _object.load();
    		instance->ScheduleClear();
    		PX4_INFO("Custom Fuse Mav Module stopped");
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
    		return instance->reset_state();
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
Custom module to send MAVLink servo commands to STM32 via vehicle_command (forwarded as COMMAND_LONG).
Sends engage/disengage command using MAV_CMD_USER_1.
Supports up to 3 servos, routing to MC1 (1) or MC2 (2-3).
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("custom_fuse", "command");
	PRINT_MODULE_USAGE_COMMAND("start");
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

	// Cleanup
	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

extern "C" __EXPORT int custom_fuse_main(int argc, char *argv[])
{
	return CustomFuse::main(argc, argv);
}
