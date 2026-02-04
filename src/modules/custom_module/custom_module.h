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

/**
 * @file custom_module.h
 * @author Faseeh Ahmed
 */
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#define FC_SYS_ID	1
#define FC_COMP_ID	1
#define COMPANION_SYS_ID	200
#define COMPANION_COMP_ID	201
#define GCS_SYS_ID	100
#define GCS_COMP_ID	101

using namespace time_literals;

/**
 * @brief Custom Module
 *
 * This module receives custom mavlink message and republishes it
 */
class CustomModule : public ModuleBase<CustomModule>, public px4::ScheduledWorkItem
{
public:
	CustomModule();

	/** @see ModuleBase **/
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase. Override "status" output when invoked via Commandline, to give detailed status **/
	int print_status() override;

	// Initializes the module
	bool init();

private:
	/**
	 * @brief Main Run function that runs when subscription callback is triggered
	 */
	void Run() override;

	static constexpr uint16_t MAV_CMD_USER_1 = 31010;

	void handle_vehicle_command(const hrt_abstime &now, const vehicle_command_s *vehicle_command = nullptr);
	bool send_mouse_data_cmd_ack(const uint8_t command_result);
	bool send_mouse_data_cmd(const float param1, const float param2, const float param3, const float param4, const float param5);

	// Subscription
	uORB::SubscriptionCallbackWorkItem _vehicle_command_sub{this, ORB_ID(vehicle_command)};

	// Publications
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<vehicle_command_s> _vehicle_command_pub{ORB_ID(vehicle_command)};

};
