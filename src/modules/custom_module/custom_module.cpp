/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

bool CustomModule::init()
{
	ScheduleOnInterval(100_ms);

	if (!_vehicle_command_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	return true;
}

void CustomModule::Run()
{
	const hrt_abstime now = hrt_absolute_time();
	vehicle_command_s vcmd{};

	if (should_exit()) {
		ScheduleClear();
		_vehicle_command_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	if (_vehicle_command_sub.update(&vcmd)) {
		handle_vehicle_command(now, &vcmd);

	} else {
		handle_vehicle_command(now);
	}
}

void CustomModule::handle_vehicle_command(const hrt_abstime &now,  const vehicle_command_s *vehicle_command)
{
	if (vehicle_command == nullptr) {
		return;
	}

	if (vehicle_command->command == MAV_CMD_USER_1 &&
	vehicle_command->target_system == FC_SYS_ID &&
	vehicle_command->target_component == FC_COMP_ID)
	{
		send_mouse_data_cmd_ack(vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
		send_mouse_data_cmd(vehicle_command->param1,
				vehicle_command->param2,
				vehicle_command->param3,
				vehicle_command->param4,
				vehicle_command->param5);
	}
}

bool CustomModule::send_mouse_data_cmd(const float param1, const float param2, const float param3, const float param4, const float param5)
{
	vehicle_command_s vcmd;
	vcmd.timestamp = hrt_absolute_time();
	vcmd.command = MAV_CMD_USER_1;
	vcmd.target_system = COMPANION_SYS_ID;
	vcmd.target_component = COMPANION_COMP_ID;
	vcmd.param1 = param1;
	vcmd.param2 = param2;
	vcmd.param3 = param3;
	vcmd.param4 = param4;
	vcmd.param5 = param5;
	return _vehicle_command_pub.publish(vcmd);
}

bool CustomModule::send_mouse_data_cmd_ack(const uint8_t command_result)
{
	hrt_abstime now = hrt_absolute_time();
	vehicle_command_ack_s vcmd_ack{};
	vcmd_ack.timestamp = now;
	vcmd_ack.command = MAV_CMD_USER_1;
	vcmd_ack.result = command_result;
	vcmd_ack.target_system = GCS_SYS_ID;
	vcmd_ack.target_component = GCS_COMP_ID;
	return _vehicle_command_ack_pub.publish(vcmd_ack);
}


int CustomModule::print_status()
{

	return 0;
}

int CustomModule::custom_command(int argc, char *argv[])
{
	if (argc >= 1) {
		if (strcmp(argv[0], "status") == 0) {
			get_instance()->print_status();
			return 0;

		}
	}

	return print_usage("Unrecognized command");
}

int CustomModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Receives custom mavlink message containing mouse tracking data and republishes it.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("custom_module", "command");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Shows current mouse tracking data");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
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
		PX4_ERR("Alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

extern "C" __EXPORT int custom_module_main(int argc, char *argv[])
{
	return CustomModule::main(argc, argv);
}
