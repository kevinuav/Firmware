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
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
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
#include <uORB/topics/engine_status.h>
#include <uORB/topics/parameter_update.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

#include <drivers/drv_hrt.h>
//#include "sPort_data.h"
#include <sys/ioctl.h>





class JetEngine : public ModuleBase<JetEngine>, public ModuleParams,public px4::ScheduledWorkItem
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

	uORB::Publication<engine_status_s>		_engine_status_pub{ORB_ID(engine_status)};

	actuator_armed_s _armed{};
	actuator_jet_s 					_act{};
	actuator_controls_s 				_actuators{};

	engine_status_s					_engine_status{};

	bool _engine_started = false;

	uint32_t _armed_time = 0;

	bool _throttle_armed = false;




	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::FUEL_CONSUM_RATE>) _fuel_consum_rate,
		(ParamFloat<px4::params::FUEL_TANK_CAP>) _fuel_tank_cap
	)

	const char *device_name = "/dev/ttyS3"; /* default USART4 */



	int _Baudrate = 115200;

	int _uart4;

	int _fd;

	double _fuelcomsuption = 0;

	const uint64_t _delaytime = 100000;

	const uint64_t _min_rpm = 50000;

	const uint64_t _max_rpm = 160000;

	uint8_t _buf[512];

	char _port[512] {};

	uint8_t _rx_buffer[512];
	uint16_t _rx_buffer_bytes{};

	hrt_abstime _last_read{0};

	uint8_t _status;

	uint8_t _fault;

	uint8_t _last_fault;

	uint64_t _engine_stop_time = 0;

	bool _restarting = false;

	bool _restarted = true;

	static int set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop);

	 int status();

	 int start();

	 int stop();

	 int collect();

	 int parseChar(uint8_t);

	 int handle(int);

	 int uart_init();
};
