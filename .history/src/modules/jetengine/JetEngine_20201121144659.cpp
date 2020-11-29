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
#include <termios.h>



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



 _uart1 = open("/dev/ttyS3", O_RDWR | O_NONBLOCK | O_NOCTTY);

 set_opt(_uart1, Baudrate, 8, 'N', 1);

 if (_uart1 < 0) {
		warnx("ERROR opening UART4, aborting..\n");
		return _uart1;
 		}
		 else {
			 warnx("succefully opening UART4 %d",_uart1);
		 }
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


	uint8_t sample_uart[] = {'S', 'A', 'M', 'P', 'L', 'E', ' ', '\n'};

//	warnx("jjjjjjjjrrrrr");
	while(1)
	{
	write(_uart1,sample_uart, sizeof(sample_uart));
	}

//	read(_uart1,&letter[0],sizeof(letter));

//	warnx("letter= %c",letter[0]);

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
JetEngine::set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio, oldtio;
	//保存测试现有串口参数设置，在这里如果串口号等出错，会有相关的出错信息
	if (tcgetattr(fd, &oldtio) != 0)
	{
		perror("SetupSerial 1");
		return -1;
	}
	memset(&newtio, 0, sizeof(newtio));
	//步骤一，设置字符大小
	newtio.c_cflag |= CLOCAL | CREAD;
	newtio.c_cflag &= ~CSIZE;
	//设置停止位
	switch (nBits)
	{
		case 7:
			newtio.c_cflag |= CS7;
			break;
		case 8:
			newtio.c_cflag |= CS8;
			break;
	}
	//设置奇偶校验位
	switch (nEvent)
	{
		case 'O': //奇数
			newtio.c_cflag |= PARENB;
			newtio.c_cflag |= PARODD;
			newtio.c_iflag |= (INPCK | ISTRIP);
			break;
		case 'E': //偶数
			newtio.c_iflag |= (INPCK | ISTRIP);
			newtio.c_cflag |= PARENB;
			newtio.c_cflag &= ~PARODD;
			break;
		case 'N': //无奇偶校验位
			newtio.c_cflag &= ~PARENB;
			break;
	}
	switch (nSpeed)
	{
		case 9600:
			cfsetispeed(&newtio, B9600);
			cfsetospeed(&newtio, B9600);
			break;

		case 19200:
			cfsetispeed(&newtio, B19200);
			cfsetospeed(&newtio, B19200);
			break;

		case 38400:
			cfsetispeed(&newtio, B38400);
			cfsetospeed(&newtio, B38400);
			break;

		case 57600:
			cfsetispeed(&newtio, B57600);
			cfsetospeed(&newtio, B57600);
			break;

		case 115200:
			cfsetispeed(&newtio, B115200);
			cfsetospeed(&newtio, B115200);
			break;

		case 230400:
			cfsetispeed(&newtio, B230400);
			cfsetospeed(&newtio, B230400);
			break;

		default:
			cfsetispeed(&newtio, B115200);
			cfsetospeed(&newtio, B115200);
			break;
	}
	//设置停止位
	if (nStop == 1)
	{
		newtio.c_cflag &= ~CSTOPB;
	}
	else if (nStop == 2)
	{
		newtio.c_cflag |= CSTOPB;
	}
	//设置等待时间和最小接收字符
	newtio.c_cc[VTIME] = 0;
	newtio.c_cc[VMIN] = 0;
	//处理未接收字符
	tcflush(fd, TCIFLUSH);
	//激活新配置
	if ((tcsetattr(fd, TCSANOW, &newtio)) != 0)
	{
		perror("com set error");
		return -1;
	}
	printf("set done!\n");
	return 0;
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
