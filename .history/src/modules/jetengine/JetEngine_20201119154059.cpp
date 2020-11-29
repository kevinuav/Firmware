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
#include "JetEngine.hpp"


JetEngine::JetEngine() :
	ModuleParams(nullptr),
	WorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

bool
JetEngine::init()
{

warnx("1111111");
	if (!_jet_sub.registerCallback()) {
		PX4_ERR("jet engine callback registration failed!");
		return false;
	}

const int uart = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK);


	return true;
}

void
JetEngine::Run()
{
	if (should_exit()) {
		_jet_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}
PX4_INFO("111111122222222");

	const uint8_t x7E[] = { 0x7D, 0x5E };

	write(uart,x7E, sizeof(x7E));

	if (_actuator.update(&_act)) {

	//	warnx("act0=%f act1=%f act2=%f",(double)_act.control[0],(double)_act.control[1],(double)_act.control[2]);
/* lazily publish the setpoint only once available */

		if(_armed_sub.update(&_armed))
		{
			_throttle_armed = _armed.armed;
			if(_throttle_armed)
			{
			_armed_time = (uint32_t)_armed.timestamp/1000.0f - _armed.armed_time_ms;
			warnx("armed time stamp is %d  time stamp is %d armed time is %d",_armed.armed_time_ms,(int)_armed.timestamp,_armed_time);
			}
			else
			{
				_armed_time = 0;
			}
		}

		if(!_engine_started)
		{
			if (_throttle_armed )
				{
					if(_armed_time <= 500)
					{
						_act.control[actuator_jet_s::INDEX_THROTTLE] = 0.0f;
					}
					else if(_armed_time <= 1000)
					{
						_act.control[actuator_jet_s::INDEX_THROTTLE] = 1.0f;
					}else
					{
						_engine_started = true;
						warnx("engine started");
						/* code */
					}

				}
				else
				{
					_armed_time = 0;
				}

		}


		if(!_throttle_armed)
		{
			_act.control[actuator_jet_s::INDEX_THROTTLE] = 0.1f;
			_engine_started = false;
		//	warnx("engine cutted off");
		}

		_actuators.control[0] = _act.control[0];
		_actuators.control[1] = _act.control[1];
		_actuators.control[2] = _act.control[2];
		_actuators.control[3] = _act.control[3];
		_actuators.control[4] = _act.control[4];
		_actuators.control[5] = _act.control[5];
		_actuators.control[6] = _act.control[6];
		_actuators.control[7] = _act.control[7];

		_actuators_0_pub.publish(_actuators);
	}
}

int
JetEngine::task_spawn(int argc, char *argv[])
{
	JetEngine *instance = new JetEngine();

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

static int sPort_open_uart(const char *uart_name, struct termios *uart_config, struct termios *uart_config_original)
{
	/* Open UART */
	const int uart = open(uart_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (uart < 0) {
		PX4_ERR("Error opening port: %s (%i)", uart_name, errno);
		return -1;
	}

	/* Back up the original UART configuration to restore it after exit */
	int termios_state;

	if ((termios_state = tcgetattr(uart, uart_config_original)) < 0) {
		PX4_ERR("tcgetattr %s: %d\n", uart_name, termios_state);
		close(uart);
		return -1;
	}

	/* Fill the struct for the new configuration */
	tcgetattr(uart, uart_config);

	/* Disable output post-processing */
	uart_config->c_oflag &= ~OPOST;

	uart_config->c_cflag |= (CLOCAL | CREAD);    /* ignore modem controls */
	uart_config->c_cflag &= ~CSIZE;
	uart_config->c_cflag |= CS8;         /* 8-bit characters */
	uart_config->c_cflag &= ~PARENB;     /* no parity bit */
	uart_config->c_cflag &= ~CSTOPB;     /* only need 1 stop bit */
	uart_config->c_cflag &= ~CRTSCTS;    /* no hardware flowcontrol */

	/* setup for non-canonical mode */
	uart_config->c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
	uart_config->c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

	/* Set baud rate */
	const speed_t speed = B9600;

	if (cfsetispeed(uart_config, speed) < 0 || cfsetospeed(uart_config, speed) < 0) {
		PX4_ERR("%s: %d (cfsetispeed, cfsetospeed)\n", uart_name, termios_state);
		close(uart);
		return -1;
	}

	if ((termios_state = tcsetattr(uart, TCSANOW, uart_config)) < 0) {
		PX4_ERR("%s (tcsetattr)\n", uart_name);
		close(uart);
		return -1;
	}

	return uart;

int
JetEngine::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int
JetEngine::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description


)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("jetengine", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int jetengine_main(int argc, char *argv[])
{
	return JetEngine::main(argc, argv);
}
