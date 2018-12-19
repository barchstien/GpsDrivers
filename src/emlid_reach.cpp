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
#include <cstring>

#include <stdlib.h>

#define SYMBOL_START		0x24	// $
#define SYMBOL_CHECKSUM		0x2a	// *
#define SYMBOL_CR			0x0d	// <CR>
#define SYMBOL_LF			0x0a	// <LF>

#define NMEA_TALKER_OFFSET	1
#define NMEA_TALKER_LEN		2

// NMEA talker's id
#define NMEA_TALKER_GLONASS		"GL"
#define NMEA_TALKER_GPS			"GP"
#define NMEA_TALKER_GNSS		"GN"	// Mixed GPS and GLONASS data, according to IEIC 61162-1
#define NMEA_TALKER_GALILEO		"GA"
#define NMEA_TALKER_BEIDOU_1	"BD"
#define NMEA_TALKER_BEIDOU_2	"GB"

#define NMEA_TYPE_OFFSET	3
#define NMEA_TYPE_LEN		3

// NMEA sentence types
#define NMEA_Fix_information						"GGA"
#define NMEA_Overall_Satellite_data					"GSA"
#define NMEA_GPS_Pseudorange_Noise_Statistics		"GST"
#define NMEA_Detailed_Satellite_data				"GSV"
#define NMEA_recommended_minimum_data_for_gps		"RMC"
#define NMEA_Vector_track_an_Speed_over_the_Ground	"VTG"

// TODO
// Heading_Magnetic	"HDT"
// Heading_True		"HVD"
// Time_Date_UTC	"ZDA"


GPSDriverEmlidReach::GPSDriverEmlidReach(GPSCallbackPtr callback, void *callback_user, struct vehicle_gps_position_s *gps_position) :
	GPSHelper(callback, callback_user),
	_gps_position(gps_position)
{
}

int
GPSDriverEmlidReach::configure(unsigned &baudrate, OutputMode output_mode)
{
	//GPS_INFO("EmlidReach: Blop !!");

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

	GPS_INFO("EmlidReach: config with baudrate: %d", baudrate);

	/* set baudrate first */
	if (GPSHelper::setBaudrate(baudrate) != 0) {
		return -1;
	}

	GPS_INFO("EmlidReach: config OK");

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
			for (int i=0; i<ret; i++){
				int len = parseChar(buf[i]);
				if (len > 0) {
					handleNmeaSentence();
				}
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
	switch (_decode_state) {
	case NMEA_0183_State::init:
		if (b == SYMBOL_START) {
			_decode_state = NMEA_0183_State::got_start_byte;
			_rx_buff_len = 0;
			_rx_buff[_rx_buff_len ++] = b;
		}
		break;

	case NMEA_0183_State::got_start_byte:
		if (b == SYMBOL_START) {
			_decode_state = NMEA_0183_State::got_start_byte;
			_rx_buff_len = 0;
			_rx_buff[_rx_buff_len ++] = b;
			GPS_WARN("EMLIDREACH: NMEA message truncated");
		} else if (b == SYMBOL_CHECKSUM) {
			_decode_state = NMEA_0183_State::got_checksum_byte;
			//memset(_checksum_buff, '0', sizeof(_checksum_buff));
			_checksum_buff_len = 0;
		} else if (_rx_buff_len >= sizeof(_rx_buff)) {
			GPS_WARN("EMLIDREACH: NMEA message overflow");
			_decode_state = NMEA_0183_State::init;
		} else {
			_rx_buff[_rx_buff_len++] = b;
		}
		break;

	case NMEA_0183_State::got_checksum_byte:
		if (b == SYMBOL_START) {
			_decode_state = NMEA_0183_State::got_start_byte;
			_rx_buff_len = 0;
			_rx_buff[_rx_buff_len ++] = b;
			GPS_WARN("EMLIDREACH: NMEA message truncated");
		} else if (b == SYMBOL_CR || b == SYMBOL_LF) {
			// found the end of line, undo last increment
			//_rx_buff_len -= 1;
			// compute expected checksum
			// https://en.wikipedia.org/wiki/NMEA_0183#C_implementation_of_checksum_generation
			int cs = 0;
			for (unsigned i=1; i<_rx_buff_len; i++){
				cs ^= _rx_buff[i];
			}

			// convert read checksum to int
			int read_cs = 0;
			read_cs = strtol(_checksum_buff, nullptr, 16);
			if (read_cs == 0) {
				if (errno == ERANGE) {
					GPS_WARN("EMLIDREACH: NMEA checksum extraction failed: %s", _checksum_buff);
					_decode_state = NMEA_0183_State::init;
				}
			}

			if (cs != read_cs){
				GPS_WARN("EMLIDREACH: NMEA checksum failed, expectd %02x, got %02x \n %s", cs, read_cs, _rx_buff);
				_decode_state = NMEA_0183_State::init;
			} else {
				//GPS_INFO("EMLIDREACH: NMEA sentence completed \n %s", _rx_buff);
				// return length of formed buffer and re-init state machine for next sentence
				ret = _rx_buff_len;
				_decode_state = NMEA_0183_State::init;
				
			}
		} else if (_checksum_buff_len >= NMEA_CHECKSUM_LEN) {
			GPS_WARN("EMLIDREACH: NMEA message checksum overflow");
			_decode_state = NMEA_0183_State::init;
		} else {
			_checksum_buff[_checksum_buff_len++] = b;
		}

		break;
	}

	return ret;
}


void
GPSDriverEmlidReach::handleNmeaSentence() {

	// TODO GA vs GL vs GP vs GN talkers ??
	// Should switch with priority GP, GN, GA (GPS, GPS+GLONASS, GALILEO)
	// For now, let's just use GN (GPS + GLONASS)
	if (strncmp(_rx_buff + NMEA_TALKER_OFFSET, NMEA_TALKER_GNSS, NMEA_TALKER_LEN) != 0) {
		// ignore anything that is not GPS+GLONASS
		return;
	}

	if (strncmp(_rx_buff + NMEA_TYPE_OFFSET, NMEA_Fix_information, NMEA_TYPE_LEN) == 0) {

	} else if (strncmp(_rx_buff + NMEA_TYPE_OFFSET, NMEA_Overall_Satellite_data, NMEA_TYPE_LEN) == 0) {

	} else if (strncmp(_rx_buff + NMEA_TYPE_OFFSET, NMEA_GPS_Pseudorange_Noise_Statistics, NMEA_TYPE_LEN) == 0) {

	} else if (strncmp(_rx_buff + NMEA_TYPE_OFFSET, NMEA_Detailed_Satellite_data, NMEA_TYPE_LEN) == 0) {

	} else if (strncmp(_rx_buff + NMEA_TYPE_OFFSET, NMEA_recommended_minimum_data_for_gps, NMEA_TYPE_LEN) == 0) {

	} else if (strncmp(_rx_buff + NMEA_TYPE_OFFSET, NMEA_Vector_track_an_Speed_over_the_Ground, NMEA_TYPE_LEN) == 0) {

	} else {
		GPS_INFO("EMLIDREACH: NMEA message type unknown \n %s \n %c %c %c", _rx_buff, _rx_buff[3], _rx_buff[4], _rx_buff[5]);
	}


	_nmea_cnt ++;
	if (_nmea_cnt % 100 == 0)
		GPS_INFO("EMLIDREACH: NMEA message number %d received", _nmea_cnt);
	
	return;
}





