/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file emlid_reach.cpp
 *
 * @author Bastien Auneau <bastien.auneau@while-true.fr>
 */

#include "emlid_reach.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <ctime>
#include <cmath>


GPSDriverEmlidReach::GPSDriverEmlidReach(GPSCallbackPtr callback, void *callback_user, struct vehicle_gps_position_s *gps_position) :
	GPSHelper(callback, callback_user),
	_gps_position(gps_position)
{
}

int
GPSDriverEmlidReach::configure(unsigned &baudrate, OutputMode output_mode)
{
	//PX4_INFO("EmlidReach: Blop !!");

	// TODO RTK
	if (output_mode != OutputMode::GPS) {
		GPS_WARN("EMLIDREACH: Unsupported Output Mode %i", (int)output_mode);
		return -1;
	}

	// TODO check for different baudrates : 56000, 57600, 115200, 128000, 153600
	// TODO others baudrates are too low or high

	// disable auto select fro now
	if (baudrate == 0)
		return -1;

	PX4_INFO("EmlidReach: config with baudrate: %d", baudrate);

	/* set baudrate first */
	if (GPSHelper::setBaudrate(baudrate) != 0) {
		return -1;
	}

	PX4_INFO("EmlidReach: config OK");

	return 0;
}

int
GPSDriverEmlidReach::receive(unsigned timeout)
{
	uint8_t buf[GPS_READ_BUFFER_SIZE];

	/* timeout additional to poll */
	//gps_abstime time_started = gps_absolute_time();

	while (true) {
		int ret = read(buf, sizeof(buf), timeout);
		if (ret > 0) {
			/*std::stringstream ss;
			//
			for (int i=0; i<ret; i++){
				ss << buf[i];
			}*/
			//PX4_INFO("EmlidReach: read: %d", ret);
			//PX4_INFO("EmlidReach: recv: %s", buf);
			for (int i=0; i<ret; i++){
				parseChar(buf[i]);
			}
		}else{
			usleep(20000);
		}
	}
}

int
GPSDriverEmlidReach::parseChar(uint8_t b)
{
	int ret = 0;
	/*switch (_decode_state){
	case NMEA0183_State::init:
		PX4_INFO("EmlidReach: in init");
		break;
	}*/
	return ret;
}





