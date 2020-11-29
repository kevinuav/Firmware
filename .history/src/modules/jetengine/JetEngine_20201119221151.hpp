/****************************************************************************
 *
 *   Copyright (c) 2017-2019 PX4 Development Team. All rights reserved.
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
 *
 * Online and offline geotagging from camera feedback
 *
 * @author Mohammed Kabir <kabir@uasys.io>
 */

#pragma once

#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/WorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/camera_capture.h>
#include <uORB/topics/camera_trigger.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/actuator_jet.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>

#include <drivers/drv_hrt.h>
//#include "sPort_data.h"
#include <sys/ioctl.h>





class JetEngine : public ModuleBase<JetEngine>, public ModuleParams, public px4::WorkItem
{
public:
	JetEngine();
	~JetEngine() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:

	void Run() override;

	uORB::SubscriptionCallbackWorkItem _jet_sub{this, ORB_ID(actuator_jet)};

	uORB::Subscription	_actuator{ORB_ID(actuator_jet)};

	uORB::Publication<actuator_controls_s>		_actuators_0_pub{ORB_ID(actuator_controls_0)};

	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};

	actuator_armed_s _armed{};
	actuator_jet_s 					_act{};
	actuator_controls_s 				_actuators{};

	bool _engine_started = false;

	uint32_t _armed_time = 0;

	bool _throttle_armed = false;


/*
	DEFINE_PARAMETERS(
		(ParamBool<px4::params::CAM_CAP_FBACK>) _param_camera_capture_feedback
	)
*/

	const char *device_name = "/dev/ttyS3"; /* default USART8 */

	const char *Baudrate = "115200";

	int _uart1;

	static int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);
};
