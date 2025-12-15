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

#include "custom_module.h"

CustomModule::CustomModule() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

CustomModule::~CustomModule()
{

}

bool CustomModule::init()
{
	ScheduleOnInterval(100_ms);
	return true;
}

void CustomModule::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	parameters_update();

	update_topics();

	safety_status_update();

	// update_home();

	_rc_high = rc.channels[((uint8_t)_param_safety_rc_ch.get() - 1)] >= 0.85f;

	if (_rc_high && !_rc_high_once)
	{
		send_info_to_gcs("ENGAGEMENT BUTTON IS        SWITCHED ON");
		_rc_high_once = true;
	}
	else if (!_rc_high)
	{
		_rc_high_once = false;
	}

	bool trigger = _rc_high && (dist_sens.current_distance <= _param_safety_eng_dist.get()) && _safety_check_passed;

	double pwm = trigger ? 1.0f : -1.0f;

	if (trigger != _servo_high) {
		vehicle_command_s cmd{};
		cmd.timestamp = hrt_absolute_time();
		cmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR;
		cmd.param1 = pwm;
		cmd.target_system = 1;
		cmd.target_component = 1;
		cmd.source_system = 1;
		cmd.source_component = 1;
		cmd.confirmation = 0;
		cmd.from_external = false;
		_vehicle_command_pub.publish(cmd);
		_servo_high = trigger;

		if(trigger)
		{
			send_info_to_gcs("SAFETY DISENGAGED");
		}
		else
		{
			send_info_to_gcs("SAFETY ENGAGED");
		}
	}
}

void CustomModule::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s update;
		_parameter_update_sub.copy(&update);
		updateParams();
	}
}

void CustomModule::safety_status_update()
{
	if (!_param_safety_soft_en.get()) {
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
	else if (_rc_high) {
		_safety_check_passed = false;
		if (!_conditions_warn_once) {
			send_info_to_gcs("SAFETY CONDITIONS NOT MET.  ENGAGING SAFETY.");
			_conditions_warn_once = true;
		}
	}
	else {
		_safety_check_passed = false;
		_conditions_warn_once = false;
	}

}

bool CustomModule::conditions_met()
{
	// This safety needs to check executed before returning the result
	// New Home Position Setting
	if (_param_safety_soft_en.get() & 0b10000000) {
		update_home();
	}

	// Arming Check
	if (_param_safety_soft_en.get() & 0b1) {
		if (!_armed) return false;
	}

	// Safety Time Check
	if (_param_safety_soft_en.get() & 0b10) {
		hrt_abstime now = hrt_absolute_time();
		if ((now - _arming_timestamp < SAFETY_TIME) || _arming_timestamp <= 1) return false;
	}

	// Horizontal Distance Check
	if (_param_safety_soft_en.get() & 0b100) {
		float dist_xy = sqrtf((_local_pos_x - _arming_x) * (_local_pos_x - _arming_x) + (_local_pos_y - _arming_y) * (_local_pos_y - _arming_y));
		if (dist_xy < SAFETY_DISTANCE) return false;
	}

	// Vertical Distance Check
	if (_param_safety_soft_en.get() & 0b1000) {
		float dist_z = -(_local_pos_z - _arming_z);
		if (dist_z < SAFETY_ALTITUDE) return false;
	}

	// RTH/RTL + Land Mode Check
	if (_param_safety_soft_en.get() & 0b10000) {
		if (_nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL ||
	    	_nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_LAND) return false;
	}

	// PWM Check
	if (_param_safety_soft_en.get() & 0b100000) {
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
	if (_param_safety_soft_en.get() & 0b1000000) {
		if (_failsafe) return false;
	}

	return true;
}

void CustomModule::update_home()
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

void CustomModule::update_topics()
{
	if (_vehicle_status_sub.updated()) {
		_vehicle_status_sub.copy(&status);
		_nav_state = status.nav_state;
		_arming_state = status.arming_state;
		_failsafe = status.failsafe;
	}


	if (_vehicle_local_position_sub.updated()) {
		_vehicle_local_position_sub.copy(&pos);
		_local_pos_x = pos.x;
		_local_pos_y = pos.y;
		_local_pos_z = pos.z;
		_local_heading = pos.heading;
	}


	if (_vehicle_global_position_sub.updated()) {
		_vehicle_global_position_sub.copy(&global_pos);
		_global_lat = global_pos.lat;
		_global_lon = global_pos.lon;
		_global_alt = global_pos.alt;
	}

	if (_actuator_outputs_sub.updated()) {
		_actuator_outputs_sub.copy(&outputs);
	}

	if (_rc_channels_sub.updated()) {
		_rc_channels_sub.copy(&rc);
	}

	if (_distance_sensor_sub.updated()) {
		_distance_sensor_sub.copy(&dist_sens);
	}
}

void CustomModule::send_info_to_gcs(const char *message)
{
	mavlink_log_emergency(&_mavlink_log_pub, "%s", message);
}

int CustomModule::task_spawn(int argc, char *argv[])
{
	CustomModule *instance = new CustomModule();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int CustomModule::print_status()
{
	return 0;
}

int CustomModule::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int CustomModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Custom Module for disengaging safety based on RC input, LiDAR distance readings and numerous software based safeties.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("custom_module", "command");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int custom_module_main(int argc, char *argv[])
{
	return CustomModule::main(argc, argv);
}
