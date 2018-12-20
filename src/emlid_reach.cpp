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

#define NMEA_FIELD_MAX_LEN	14

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

#define EMLID_UNUSED(x) (void)x;


GPSDriverEmlidReach::GPSDriverEmlidReach(GPSCallbackPtr callback, void *callback_user, struct vehicle_gps_position_s *gps_position) :
	GPSHelper(callback, callback_user),
	_gps_position(gps_position)
{
	
}


int
GPSDriverEmlidReach::configure(unsigned &baudrate, OutputMode output_mode)
{
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
	while (true) {
		if (_read_buff_ptr >= _read_buff + _read_buff_len) {
			// no more data in buff to process, read from serial
			_read_buff_len = read(_read_buff, sizeof(_read_buff), timeout);
			if (_read_buff_len > 0) {
				_read_buff_ptr = _read_buff;
			}
		} else {
			// process data in buffer from previous read
			while (_read_buff_ptr < _read_buff + _read_buff_len) {
				if (parseChar(*(_read_buff_ptr++)) > 0) {
					if (handleNmeaSentence()) {
						return 1;
					}
				}
			}
		}
	}
}


void
GPSDriverEmlidReach::nmeaParserRestart()
{
	_decode_state = NMEA_0183_State::got_start_byte;
	_nmea_buff_len = 0;
}


int
GPSDriverEmlidReach::parseChar(uint8_t b)
{
	int ret = 0;
	switch (_decode_state) {
	case NMEA_0183_State::init:
		if (b == SYMBOL_START) {
			nmeaParserRestart();
			_nmea_buff[_nmea_buff_len ++] = b;
		}
		break;

	case NMEA_0183_State::got_start_byte:
		if (b == SYMBOL_START) {
			nmeaParserRestart();
			_nmea_buff[_nmea_buff_len ++] = b;
			GPS_WARN("EMLIDREACH: NMEA message truncated");
		} else if (b == SYMBOL_CHECKSUM) {
			_decode_state = NMEA_0183_State::got_checksum_byte;
			_checksum_buff_len = 0;
		} else if (_nmea_buff_len >= NMEA_SENTENCE_MAX_LEN) {
			GPS_WARN("EMLIDREACH: NMEA message overflow");
			_decode_state = NMEA_0183_State::init;
		} else {
			_nmea_buff[_nmea_buff_len++] = b;
		}
		break;

	case NMEA_0183_State::got_checksum_byte:
		if (b == SYMBOL_START) {
			nmeaParserRestart();
			_nmea_buff[_nmea_buff_len ++] = b;
			GPS_WARN("EMLIDREACH: NMEA message truncated");
		} else if (b == SYMBOL_CR || b == SYMBOL_LF) {
			// compute expected checksum
			// https://en.wikipedia.org/wiki/NMEA_0183#C_implementation_of_checksum_generation
			int cs = 0;
			for (unsigned i=1; i<_nmea_buff_len; i++){
				cs ^= _nmea_buff[i];
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
				GPS_WARN("EMLIDREACH: NMEA checksum failed, expectd %02x, got %02x \n %s", cs, read_cs, _nmea_buff);
				_decode_state = NMEA_0183_State::init;
			} else {
				//GPS_INFO("EMLIDREACH: NMEA sentence completed len:%d \n %s", _nmea_buff_len, _nmea_buff);
				// return length of formed buffer and re-init state machine for next sentence
				// null terminate in case it's printed later on, while debug/fixing
				// safe length-wise because NMEA_SENTENCE_MAX_LEN could contain checksum, CR and LF
				_nmea_buff[_nmea_buff_len] = '\0';
				ret = _nmea_buff_len;
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


bool
GPSDriverEmlidReach::handleNmeaSentence()
{
	// TODO GA vs GL vs GP vs GN talkers ??
	// Should switch with priority GP, GN, GA (GPS, GPS+GLONASS, GALILEO)
	// For now, let's just use GN (GPS + GLONASS)
	if (strncmp(_nmea_buff + NMEA_TALKER_OFFSET, NMEA_TALKER_GNSS, NMEA_TALKER_LEN) != 0) {
		// ignore anything that is not GPS+GLONASS
		return false;
	}

	// pass talker + type field, ie $--GGA
	char *ptr = _nmea_buff + 6;
	char *end_ptr;
	bool update = false;

	if (strncmp(_nmea_buff + NMEA_TYPE_OFFSET, NMEA_Fix_information, NMEA_TYPE_LEN) == 0) {
		// $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh<CR><LF>
		//        UTC-time  Lat     N|S        E|W #sat   Alt   Geodial-separation
		//                            Long       Quality      Unit-alt    Diff-ref-station
		//                                            H-dilution    Unit-Geodial
		//                                                            Age-diff-GPS
		// $GNGGA,203735.20,5954.5926558,N,01046.5572464,E,1,07,1.0,6.317,M,38.953,M,0.0,

		//int hour, min, sec, millisec;
		double tmp_time = 0.0, lat = 0.0, lon = 0.0, alt = 0.0;
		char ns = '?', ew = '?';
		int num_of_sat = 0;
		uint8_t fix_quality = 0;
		double horizontal_dilution_precision = 0;

		if (ptr && *(++ptr) != ',') { tmp_time = strtod(ptr, &end_ptr); ptr = end_ptr; }

		if (ptr && *(++ptr) != ',') { lat = strtod(ptr, &end_ptr); ptr = end_ptr; }

		if (ptr && *(++ptr) != ',') { ns = *(ptr++); }

		if (ptr && *(++ptr) != ',') { lon = strtod(ptr, &end_ptr); ptr = end_ptr; }

		if (ptr && *(++ptr) != ',') { ew = *(ptr++); }

		if (ptr && *(++ptr) != ',') { fix_quality = strtol(ptr, &end_ptr, 10); ptr = end_ptr; }

		if (ptr && *(++ptr) != ',') { num_of_sat = strtol(ptr, &end_ptr, 10); ptr = end_ptr; }

		if (ptr && *(++ptr) != ',') { horizontal_dilution_precision = strtod(ptr, &end_ptr); ptr = end_ptr; }

		if (ptr && *(++ptr) != ',') { alt = strtod(ptr, &end_ptr); ptr = end_ptr; }

		EMLID_UNUSED(tmp_time);
		EMLID_UNUSED(horizontal_dilution_precision);

		if (ns == 'S') {
			lat = -lat;
		}

		if (ew == 'W') {
			lon = -lon;
		}

		/* convert from degrees, minutes and seconds to degrees * 1e7 */
		_gps_position->lat = static_cast<int>((int(lat * 0.01) + (lat * 0.01 - int(lat * 0.01)) * 100.0 / 60.0) * 10000000);
		_gps_position->lon = static_cast<int>((int(lon * 0.01) + (lon * 0.01 - int(lon * 0.01)) * 100.0 / 60.0) * 10000000);
		_gps_position->alt = static_cast<int>(alt * 1000);
//		_rate_count_lat_lon++;

		// 2d vs 3d fix
		uint8_t fix_type = num_of_sat >= 4 ? 3 : 2;
		if (fix_quality == 0) 		{ _gps_position->fix_type = 0; }
		else if (fix_quality == 1)	{ _gps_position->fix_type = fix_type; }	// 2d or 3d
		else if (fix_quality <= 3)	{ _gps_position->fix_type = 4; }		// RTCM code differential
		else if (fix_quality == 4)	{ _gps_position->fix_type = 6; }		// 5: RTK float
		else if (fix_quality == 5)	{ _gps_position->fix_type = 5; }

		_gps_position->timestamp = gps_absolute_time();

		_gps_position->vel_m_s = 0;                                  /**< GPS ground speed (m/s) */
		_gps_position->vel_n_m_s = 0;                                /**< GPS ground speed in m/s */
		_gps_position->vel_e_m_s = 0;                                /**< GPS ground speed in m/s */
		_gps_position->vel_d_m_s = 0;                                /**< GPS ground speed in m/s */
		_gps_position->cog_rad =
			0;                                  /**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
		_gps_position->vel_ned_valid = true;                         /**< Flag to indicate if NED speed is valid */
		_gps_position->c_variance_rad = 0.1f;

		update = true;

	} else if (strncmp(_nmea_buff + NMEA_TYPE_OFFSET, NMEA_Overall_Satellite_data, NMEA_TYPE_LEN) == 0) {
		// GSA

	} else if (strncmp(_nmea_buff + NMEA_TYPE_OFFSET, NMEA_GPS_Pseudorange_Noise_Statistics, NMEA_TYPE_LEN) == 0) {
		// GST

	} else if (strncmp(_nmea_buff + NMEA_TYPE_OFFSET, NMEA_Detailed_Satellite_data, NMEA_TYPE_LEN) == 0) {
		//GSV

	} else if (strncmp(_nmea_buff + NMEA_TYPE_OFFSET, NMEA_recommended_minimum_data_for_gps, NMEA_TYPE_LEN) == 0) {
		// RMC

	} else if (strncmp(_nmea_buff + NMEA_TYPE_OFFSET, NMEA_Vector_track_an_Speed_over_the_Ground, NMEA_TYPE_LEN) == 0) {
		// GTV

	} else {
		GPS_INFO("EMLIDREACH: NMEA message type unknown \n %s \n %c %c %c", _nmea_buff, _nmea_buff[3], _nmea_buff[4], _nmea_buff[5]);
	}



	// TOD is this required so that GPS timing it taken ?
	if (update) {
//		_gps_position->timestamp_time_relative = (int32_t)(_last_timestamp_time - _gps_position->timestamp);
	}

	_nmea_cnt ++;
	if (_nmea_cnt % 100 == 0)
		GPS_INFO("EMLIDREACH: NMEA message number %d received", _nmea_cnt);
	
	return update;
}





