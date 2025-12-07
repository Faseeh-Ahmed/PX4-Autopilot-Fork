/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

	rc_channels_s rc{};
	_rc_channels_sub.copy(&rc);

	distance_sensor_s dist{};
	_distance_sensor_sub.copy(&dist);

	bool rc_high = rc.channels[7] >= 0.85f;

	bool trigger = rc_high && (dist.current_distance <= 2.0f);

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
	}
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
Custom Module for disengaging safety based on RC input and LiDAR distance readings.

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
