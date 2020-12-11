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
#include <fcntl.h>
#include <string.h>
#include <ctime>
#include <stdlib.h>
#include <stdio.h>

#include <uORB/topics/mavlink_log.h>
#include <systemlib/mavlink_log.h>

extern orb_advert_t mavlink_log_pub;


JetEngine::JetEngine() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::UART4)
{
}

bool
JetEngine::init()
{
	if (!_jet_sub.registerCallback()) {
		PX4_ERR("jet engine callback registration failed!");
		return false;
	}
	_uart4 = open("/dev/ttyS3", O_RDWR | O_NONBLOCK | O_NOCTTY);

 	set_opt(_uart4, _Baudrate, 8, 'N', 1);

	ScheduleOnInterval(100000, 0);

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

	_uart4 = open("/dev/ttyS3", O_RDWR | O_NONBLOCK | O_NOCTTY);
while(1){

	status();
	px4_usleep(100000);
	if (_actuator.update(&_act)) {


		if(_armed_sub.update(&_armed))
		{
			_throttle_armed = _armed.armed;
			if(_throttle_armed)
			{
			_armed_time = (uint32_t)_armed.timestamp/1000.0f - _armed.armed_time_ms;
		//	warnx("armed time stamp is %d  time stamp is %d armed time is %d",_armed.armed_time_ms,(int)_armed.timestamp,_armed_time);
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
						start();
						warnx("engine start");
						mavlink_log_critical(&mavlink_log_pub, "engine start");
					}


				}
				else
				{
					_armed_time = 0;
				}

		}else if(!_throttle_armed)
		{
			_act.control[actuator_jet_s::INDEX_THROTTLE] = 0.1f;

			_engine_started=false;

			if(_status!=0)
			{
				stop();
				warnx("engine stop");
				mavlink_log_critical(&mavlink_log_pub, "engine stop");
			}
		}

		_actuators.control[0] = _act.control[0];
		_actuators.control[1] = _act.control[1];
		_actuators.control[2] = _act.control[2];
		_actuators.control[3] = _act.control[3];
		_actuators.control[4] = _act.control[4];
		_actuators.control[5] = _act.control[5];
		_actuators.control[6] = _act.control[6];
		_actuators.control[7] = _act.control[7];

		//_actuators_0_pub.publish(_actuators);
	}
}
	ScheduleClear();

	ScheduleOnInterval(100000, 87*9);
	return;
}



int
JetEngine::set_opt(int fd, int nSpeed, int nBits, char nEvent, int nStop)
{
	struct termios newtio,oldtio;
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


int
JetEngine::status()
{
	//const char jet_status[] = {'@', 'H', 'M', 'I', '=', '0', ',', '0','\r','\n'};
	const char jet_status[] = {"@HMI=0,0\r\n"};
//	static uint64_t utimestamp = 0;

/*

	if(utimestamp < hrt_absolute_time())
	{
*/		write(_uart4,jet_status, sizeof(jet_status));
		//int bytes_available = 0;
		//::ioctl(_uart4, FIONREAD, (unsigned long)&bytes_available);
		//if(bytes_available)
		collect();
//		utimestamp = delaytime + hrt_absolute_time();
	//}
	return true;
}

int
JetEngine::start()
{
	//const char jet_start[] = {'@', 'C','.','H', 'M', 'I', '=', '1', ',', '0','\r','\n'};
	const char jet_start[] ={"@C.HMI=1,0\r\n"};
	write(_uart4,jet_start, sizeof(jet_start));
		return true;

}

int
JetEngine::stop()
{
	//const char jet_stop[] = {'@', 'C','.','H', 'M', 'I', '=', '0', ',', '0','\r','\n'};
	const char jet_stop[] = {"@C.HMI=0,0\r\n"};
	write(_uart4,jet_stop, sizeof(jet_stop));
		return true;

}


int
JetEngine::collect()
{

		while(1)
		{
			int j = 0;
			int bytes_count = 0;
			/* then poll or read for new data */
		//	warnx("before read");
			int ret=read(_uart4, _buf, 512);
		//	warnx("after read");
			if (ret < 0) {
				/* something went wrong when polling */
				return -1;

			} else if (ret == 0) {
				/* Timeout while polling or just nothing read if reading, let's
				 * stay here, and use timeout below. */
				return 0;
			} else if (ret > 0) {
				/* if we have new data from GPS, go handle it */
				bytes_count = ret;
			}
		/* pass received bytes to the packet decoder */
			while (j < bytes_count) {
				int l = 0;

				if ((l = parseChar(_buf[j])) > 0) {
					/* return to configure during configuration or to the gps driver during normal work
					 * if a packet has arrived */
					handle(l);

				}

				j++;
			}
			/* everything is read */
			j = bytes_count = 0;

		return ret;

		}


}

int JetEngine::parseChar(uint8_t b)
{
	int iRet = 0;
	static uint8_t a0,a1,a2;
	a2 = a1;
	a1 = a0;
	a0 = b;
	if(a0==0xff&&a1==0xff&&a2==0xff)
	{
		iRet = _rx_buffer_bytes;
		_rx_buffer_bytes = 0;

	}
	else
	{
		_rx_buffer[_rx_buffer_bytes++] = b;
		return 0;
	}

	return iRet;
}

int JetEngine::handle(int len)
{
	char *endp;

	bool val=0;

	char *bufptr = (char *)(_rx_buffer + 7);
	for(int i=0; i<(len-7);i++)
	{
		if(*(bufptr+i)=='=')
		{
			if(*(bufptr+i+1)=='"')
			{
				bufptr =bufptr+i+2;
				val=0;
			}
		else
		{
		bufptr = bufptr+i+1;
		val=1;
		}
		}
	}

	if (memcmp(_rx_buffer+7, "n1", 2) == 0)
	{
		double temp=0;
		if(val)
		temp = strtod(bufptr, &endp);
		_engine_status.temp=temp;
		warnx("temp=%f",temp);
	}else
	if (memcmp(_rx_buffer+7, "va1", 3) == 0)
	{
		uint64_t rpm;
		double engine_load=0.0f;
		rpm = strtol(bufptr, &endp,10);
		_engine_status.rpm=rpm;
		if(rpm>_min_rpm)
		{
			engine_load = (rpm-_min_rpm)/(_max_rpm-rpm);
		}
		if(rpm>_max_rpm)engine_load = 1;
		_engine_status.engine_load = engine_load;
		warnx("rpm=%d",(int)rpm);
		warnx("engine load=%f",engine_load);
	}else
	if (memcmp(_rx_buffer+7, "va4", 3) == 0)
	{
		static uint8_t last_status;
		_status = strtol(bufptr, &endp,10);

		_engine_status.status=_status;
		warnx("status=%d",_status);
		if(last_status!=_status)
		{
		switch(_status)
		{
		case engine_state::standby1	:mavlink_log_critical(&mavlink_log_pub, "engine stand by");break;
		case engine_state::starting	:mavlink_log_critical(&mavlink_log_pub, "engine starting....");break;
	//	case engine_state::igniting	:mavlink_log_critical(&mavlink_log_pub, "engine igniting...");break;
	//	case engine_state::ignited	:mavlink_log_critical(&mavlink_log_pub, "engine ignited!");break;
	//	case engine_state::warming1	:mavlink_log_critical(&mavlink_log_pub, "engine warming up 1");break;
	//	case engine_state::warming2	:mavlink_log_critical(&mavlink_log_pub, "engine warming up 2");break;
	//	case engine_state::warming3	:mavlink_log_critical(&mavlink_log_pub, "engine warming up 3");break;
	//	case engine_state::warming4	:mavlink_log_critical(&mavlink_log_pub, "engine warming up 4");break;
	//	case engine_state::warming5	:mavlink_log_critical(&mavlink_log_pub, "engine warming up 5");break;
	//	case engine_state::declutch	:mavlink_log_critical(&mavlink_log_pub, "engine declutch");break;
		case engine_state::idle1	:mavlink_log_critical(&mavlink_log_pub, "engine idle");break;
	//	case engine_state::accelerating	:mavlink_log_critical(&mavlink_log_pub, "engine accelerating");break;
	//	case 12	:mavlink_log_critical(&mavlink_log_pub, "engine ");break;
	//	case 13	:mavlink_log_critical(&mavlink_log_pub, "engine ");break;
	//	case 14	:mavlink_log_critical(&mavlink_log_pub, "engine stop");break;
	//	case 15	:mavlink_log_critical(&mavlink_log_pub, "engine stop");break;
	//	case 16	:mavlink_log_critical(&mavlink_log_pub, "engine stop");break;
		case engine_state::cooling1	:mavlink_log_critical(&mavlink_log_pub, "engine cooling down");break;
	//	case engine_state::cooling2	:mavlink_log_critical(&mavlink_log_pub, "engine cooling down2");break;
	//	case engine_state::cooling3	:mavlink_log_critical(&mavlink_log_pub, "engine cooling down3");break;
	//	case engine_state::cooling4	:mavlink_log_critical(&mavlink_log_pub, "engine cooling down4");break;
		case engine_state::idle2	:mavlink_log_critical(&mavlink_log_pub, "engine idle");break;
		case engine_state::stoping	:mavlink_log_critical(&mavlink_log_pub, "engine stop");break;
	//	case engine_state::NoRC		:mavlink_log_critical(&mavlink_log_pub, "engine no RC input");break;
		case engine_state::standby2	:mavlink_log_critical(&mavlink_log_pub, "engine stand by");break;
	//	case 25	:mavlink_log_critical(&mavlink_log_pub, "engine stop");break;
	//	case 26	:mavlink_log_critical(&mavlink_log_pub, "engine stop");break;
	//	case 27	:mavlink_log_critical(&mavlink_log_pub, "engine stop");break;
	//	case 28	:mavlink_log_critical(&mavlink_log_pub, "engine maximum power");break;
		case engine_state::restarting	:mavlink_log_critical(&mavlink_log_pub, "engine restarting...");break;
	//	case 30	:mavlink_log_critical(&mavlink_log_pub, "throttle curve learning");break;


		default: if(_status>30)mavlink_log_critical(&mavlink_log_pub, "engine error");
		}
		}
		last_status= _status;

	}else
	if (memcmp(_rx_buffer+7, "va5", 3) == 0)
	{

		_fault = strtol(bufptr, &endp,10);
		_engine_status.fault=_fault;
		_now = hrt_absolute_time();
		if(!_last_fault&&_fault==engine_fault::flameout)
		{
			_engine_stop_time=hrt_absolute_time();
			mavlink_log_critical(&mavlink_log_pub, "engine flame out!");
		}
		if(_now-_engine_stop_time>200000)   //waiting for the ECU's responding 0.2s
			{
				if(_status==engine_state::restarting) _restarting = true;    //restarting coz it response "restarting"
			}
		if(_restarting)
		{
			if(_now-_engine_stop_time>10000000)  //waiting for 3 seconds coz the restarting procedure take about 2.5s
			{
			if(_fault==engine_fault::flameout)        //make sure if the engine has been restarted successfully?
			{
				_restarted = false;
			}
			else
			{
				_restarted = true;
			}
			_restarting = false;	//no matter if is the restarting success the restarting procedure has timeout and finish
			}
		}
		if(_fault==engine_fault::ignfail||(_fault==engine_fault::flameout&&!_restarted&&!_restarting))_engine_status.engine_failure=true; //so if the engine has flameout it should be in the restarting procedure or already has been restarted successfully
		else _engine_status.engine_failure=false;
		warnx("fault=%d",_fault);
		_last_fault = _fault;
	}

	if (memcmp(_rx_buffer+7, "t7", 2) == 0)
	{
		uint8_t min,sec;
		uint16_t running_time;
		min = strtol(bufptr, &endp,10);
		bufptr = endp + 1;
		sec = strtol(bufptr,&endp,10);
		running_time=60*min + sec;
		_engine_status.running_time=running_time;
		warnx("running time=%d:%d",min,sec);
		warnx("running time in seconds=%d",running_time);
	}

	if (memcmp(_rx_buffer+7, "j0", 2) == 0)
	{
		uint64_t read_elapsed = hrt_elapsed_time(&_last_read);
		double pump,fuel_consum_rate=_fuel_consum_rate.get();
		double fuel_tank_cap=_fuel_tank_cap.get();
		pump = strtod(bufptr, &endp);
		_engine_status.pump=pump;
		warnx("pump=%f",pump);
		_fuelcomsuption += fuel_consum_rate*pump*read_elapsed/1000000;
		double fuel_unused = fuel_tank_cap - _fuelcomsuption;
		warnx("fuel ununsed=%f",fuel_unused);

		_engine_status.fuel_consumed = _fuelcomsuption;
		_last_read= hrt_absolute_time();
	}
	_engine_status_pub.publish(_engine_status);

return 1;
}
