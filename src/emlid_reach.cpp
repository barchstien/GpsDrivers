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

// Emlid reach emits messages :
//   GAGSA, GAGSV
//   GLGSA, GLGSV
//   GPGSA, GPGSV
//   GNGGA, GNGST, GNRMC, GNVTG

// NMEA talker's id
#define NMEA_TALKER_GALILEO		"GA"
#define NMEA_TALKER_GLONASS		"GL"
#define NMEA_TALKER_GNSS		"GN"	// Mixed GPS and GLONASS data, according to IEIC 61162-1
#define NMEA_TALKER_GPS			"GP"

#define NMEA_TYPE_OFFSET	3
#define NMEA_TYPE_LEN		3

// NMEA sentence types
#define NMEA_Fix_information					"GGA"	//< GN talker only
#define NMEA_Overall_Satellite_data				"GSA"	//< GA|GL|GP
#define NMEA_GPS_Pseudorange_Noise_Statistics	"GST"	//< GN talker only
#define NMEA_Satellite_in_view					"GSV"	//< GA|GL|GP
#define NMEA_recommended_minimum_data_for_gps	"RMC"	//< GN talker only
#define NMEA_Vector_Track_and_speed_over_Ground	"VTG"	//< GN talker only

#define EMLID_UNUSED(x) (void)x;

#define GPS_PI		3.141592653589793238462643383280

#define AUTO_DETECT_MAX_TIMEOUT		100	// large because small max timeout set in read() call
#define AUTO_DETECT_MAX_PARSE_ERR	2
#define AUTO_DETECT_MAX_READ_NMEA	10


GPSDriverEmlidReach::GPSDriverEmlidReach(GPSCallbackPtr callback, void *callback_user, struct vehicle_gps_position_s *gps_position) :
	GPSHelper(callback, callback_user),
	_gps_position(gps_position)
{}


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
#if 0
	unsigned baud_allowed[]{4800, 9600, 14400, 19200, 28800, 38400, 56000, 57600, 115200, 128000, 153600, 230400, 256000, 460800};
	EMLID_UNUSED(baud_allowed);
#endif
	// disable auto select for now
	if (baudrate == 0)
		return -1;
#if 0
	for (unsigned i=0; i<sizeof(baud_allowed) / sizeof(baud_allowed[0]); i++) {
		if (baudrate > 0 && baudrate != baud_allowed[i]) {
			continue;
		}
		_nmea_parse_err_cnt = 0;
		_nmea_cnt = 0;
		GPS_INFO("EmlidReach: config with baudrate: %d", baudrate);
		if (GPSHelper::setBaudrate(baudrate) != 0) {
			continue;
		}
		// TODO test connection (timeout, bad CRC)
		if (! testConnection()) {
			continue;
		}

		GPS_INFO("EmlidReach: config OK");
		return 0;
	}
#else
	if (GPSHelper::setBaudrate(baudrate) == 0) {
		return 0;
	}
#endif
	return -1;
}


bool
GPSDriverEmlidReach::testConnection()
{
	unsigned timeout_cnt = 0;
	while (timeout_cnt < AUTO_DETECT_MAX_TIMEOUT 
		&& _nmea_parse_err_cnt < AUTO_DETECT_MAX_PARSE_ERR
		&& _nmea_cnt < AUTO_DETECT_MAX_READ_NMEA)
	{
		if (receive(50000) == 0) { // timeout larger than what defined in read() are truncated
			timeout_cnt ++;
		}
	}
	GPS_WARN("+++++     _nmea_cnt %d  timeout_cnt %d  _nmea_parse_err_cnt %d", _nmea_cnt, timeout_cnt, _nmea_parse_err_cnt);
	return timeout_cnt < AUTO_DETECT_MAX_TIMEOUT || _nmea_parse_err_cnt < AUTO_DETECT_MAX_PARSE_ERR;
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
			} else {
				// timeout occured
				//return 0;
			}
		} else {
			// process data in buffer from previous read
			while (_read_buff_ptr < _read_buff + _read_buff_len) {
				int ret = parseChar(*(_read_buff_ptr++));
				if (ret > 0) {
					_nmea_cnt ++;
					if (handleNmeaSentence()) {
						return 1;
					}
				} else if (ret < 0) {
					_nmea_parse_err_cnt ++;
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
			ret = -1;
		} else if (b == SYMBOL_CHECKSUM) {
			_decode_state = NMEA_0183_State::got_checksum_byte;
			_checksum_buff_len = 0;
		} else if (_nmea_buff_len >= NMEA_SENTENCE_MAX_LEN) {
			GPS_WARN("EMLIDREACH: NMEA message overflow");
			_decode_state = NMEA_0183_State::init;
			ret = -1;
		} else {
			_nmea_buff[_nmea_buff_len++] = b;
		}
		break;

	case NMEA_0183_State::got_checksum_byte:
		if (b == SYMBOL_START) {
			nmeaParserRestart();
			_nmea_buff[_nmea_buff_len ++] = b;
			GPS_WARN("EMLIDREACH: NMEA message truncated");
			ret = -1;
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
					ret = -1;
				}
			}

			if (cs != read_cs){
				GPS_WARN("EMLIDREACH: NMEA checksum failed, expectd %02x, got %02x \n %s", cs, read_cs, _nmea_buff);
				_decode_state = NMEA_0183_State::init;
				ret = -1;
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
			ret = -1;
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
	// pass talker + type field, ie $--GGA
	char *ptr = _nmea_buff + 6;
	char *end_ptr;
	bool update = false;

	if (strncmp(_nmea_buff + NMEA_TYPE_OFFSET, NMEA_Fix_information, NMEA_TYPE_LEN) == 0) {
		// $--GGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh<CR><LF>
		//        1         2       3 4        5 6 7  8   9   10 11 12 13 14
		// 1. UTC time
		// 2. Latitude
		// 3. N|S
		// 4. Longitude
		// 5. E|W
		// 6. fix quality
		//   6.0 - fix not available,
		//   6.1 - GPS fix,
		//   6.2 - Differential GPS fix (values above 2 are 2.3 features)
		//   6.3 = PPS fix
		//   6.4 = Real Time Kinematic
		//   6.5 = Float RTK
		//   6.6 = estimated (dead reckoning)
		//   6.7 = Manual input mode
		//   6.8 = Simulation mode
		// 7. Number of satellites
		// 8. HDOP
		// 9. Altitude
		// 10. Altitude unit
		// 11. Geoidal separation
		// 12. Geoidal separation unit, the difference between the WGS-84 earth ellipsoid and mean-sea-level (geoid), "-" means mean-sea-level below ellipsoid 
		// 13. Age of differentil GPS data
		// 14. Differential reference station ID
		//
		// $GNGGA,203735.20,5954.5926558,N,01046.5572464,E,1,07,1.0,6.317,M,38.953,M,0.0,

		//int hour, min, sec, millisec;
		double tmp_time = 0.0, lat = 0.0, lon = 0.0, alt = 0.0, geoidal_sep = 0.0;
		char ns = '?', ew = '?', alt_unit = '?';
		int sat_in_view = 0;
		uint8_t fix_quality = 0;
		double hdop = 0;

		if (ptr && *(++ptr) != ',') { tmp_time = strtod(ptr, &end_ptr); ptr = end_ptr; }

		if (ptr && *(++ptr) != ',') { lat = strtod(ptr, &end_ptr); ptr = end_ptr; }

		if (ptr && *(++ptr) != ',') { ns = *(ptr++); }

		if (ptr && *(++ptr) != ',') { lon = strtod(ptr, &end_ptr); ptr = end_ptr; }

		if (ptr && *(++ptr) != ',') { ew = *(ptr++); }

		if (ptr && *(++ptr) != ',') { fix_quality = strtol(ptr, &end_ptr, 10); ptr = end_ptr; }

		if (ptr && *(++ptr) != ',') { sat_in_view = strtol(ptr, &end_ptr, 10); ptr = end_ptr; }

		if (ptr && *(++ptr) != ',') { hdop = strtod(ptr, &end_ptr); ptr = end_ptr; }

		if (ptr && *(++ptr) != ',') { alt = strtod(ptr, &end_ptr); ptr = end_ptr; }

		if (ptr && *(++ptr) != ',') { alt_unit = *(ptr++); }

		if (ptr && *(++ptr) != ',') { geoidal_sep = strtod(ptr, &end_ptr); ptr = end_ptr; }

		EMLID_UNUSED(tmp_time);
		EMLID_UNUSED(hdop);	// use hdop and vdop from GSA
		EMLID_UNUSED(alt_unit);

		// TODO where is this time coming from ?
		_gps_position->timestamp = gps_absolute_time();

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

		// Geoidal separation = alt_ellipsoid - alt_geoid
		_gps_position->alt_ellipsoid = geoidal_sep + _gps_position->alt;

		_rate_count_lat_lon++;

		_gps_position->eph = _eph;
		_gps_position->epv = _epv;

		_gps_position->hdop = _hdop;
		_gps_position->vdop = _vdop;

		// vehicle_gps_position_s::fix_type
		//  0-1: no fix,
		//  2: 2D fix,
		//  3: 3D fix,
		//  4: RTCM code differential,
		//  5: Real-Time Kinematic, float,
		//  6: Real-Time Kinematic, fixed,
		//  8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.

		uint8_t fix_mode = sat_in_view >= 4 ? 3 : 2; // 3d vs 2d fix
		if (fix_quality == 0) 		{ _gps_position->fix_type = 0; }
		else if (fix_quality == 1)	{ _gps_position->fix_type = fix_mode; }
		else if (fix_quality <= 3)	{ _gps_position->fix_type = 4; }		// differential & PPS
		else if (fix_quality == 4)	{ _gps_position->fix_type = 6; }		// RTK fixed
		else if (fix_quality == 5)	{ _gps_position->fix_type = 5; }		// RTF float
		else 						{ _gps_position->fix_type = 0; }

		_gps_position->satellites_used = sat_in_view;

#if 0
		_gps_position->vel_m_s = 0;		/**< GPS ground speed (m/s) */
		_gps_position->vel_n_m_s = 0;	/**< GPS North velocity (m/s) */
		_gps_position->vel_e_m_s = 0;	/**< GPS East velocity (m/s) */
		_gps_position->vel_d_m_s = 0;	/**< GPS Down velocity (m/s) */
		_gps_position->cog_rad = 0;		/**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
		_gps_position->vel_ned_valid = false;	/**< Flag to indicate if NED speed is valid */
#endif
		computeNedVelocity();

//		_gps_position->c_variance_rad = 0.1f;

		update = true;

	} else if (strncmp(_nmea_buff + NMEA_TYPE_OFFSET, NMEA_Overall_Satellite_data, NMEA_TYPE_LEN) == 0) {
		// Talker ids : GA|GL|GP
		// $--GSA,a,a,x,x,x,x,x,x,x,x,x,x,x,x,x,x,x.x,x.x,x.x*hh<CR><LF>
		//        1 2 3                         14 15 16  17
		// 1. M=Manual (2d vs 3d fix), A=Automatic
		// 2. 1=no_fix, 2=2D_fix, 3=3D_fix
		// 3. 1st satellite used for fix
		// [...]
		// 14. 12th satellite used for fix
		// 15. PDOP
		// 16. HDOP
		// 17. VDOP

		double pdop = 0.0, hdop = 0.0, vdop = 0.0;
		uint8_t fix_mode = 0;

		if (ptr && *(++ptr) != ',') { /* ignore 1. */ }
		if (ptr && *(++ptr) != ',') { fix_mode = *(ptr++); }
		/* ignore 3. to 14. */
		for (int i=3; i<= 14; i++) {
			if (ptr && *(++ptr) != ',') { }
		}
		if (ptr && *(++ptr) != ',') { pdop = strtod(ptr, &end_ptr); ptr = end_ptr; }
		if (ptr && *(++ptr) != ',') { hdop = strtod(ptr, &end_ptr); ptr = end_ptr; }
		if (ptr && *(++ptr) != ',') { vdop = strtod(ptr, &end_ptr); ptr = end_ptr; }

		_hdop = hdop;
		_vdop = vdop;

		EMLID_UNUSED(pdop);
		EMLID_UNUSED(fix_mode);


	} else if (strncmp(_nmea_buff + NMEA_TYPE_OFFSET, NMEA_GPS_Pseudorange_Noise_Statistics, NMEA_TYPE_LEN) == 0) {
		// $--GST,hhmmss.ss,x,x,x,x,x,x,x,*hh<CR><LF>
		//        1         2 3 4 5 6 7 8
		// 1. UTC time
		// 2. Total RMS standard deviation of ranges inputs to the navigation solution
		// 3. Standard deviation (meters) of semi-major axis of error ellipse
		// 4. Standard deviation (meters) of semi-minor axis of error ellipse
		// 5. Orientation of semi-major axis of error ellipse (true north degrees)
		// 6. Standard deviation (meters) of latitude error
		// 7. Standard deviation (meters) of longitude error
		// 8. Standard deviation (meters) of altitude error
		// 

		double tmp_time = 0.0, rms_err = 0.0, dev_maj = 0.0, dev_min = 0.0;
		double north_deg = 0.0, lat_err = 0.0, lon_err = 0.0, alt_err = 0.0;

		if (ptr && *(++ptr) != ',') { tmp_time = strtod(ptr, &end_ptr); ptr = end_ptr; }
		if (ptr && *(++ptr) != ',') { rms_err = strtod(ptr, &end_ptr); ptr = end_ptr; }
		if (ptr && *(++ptr) != ',') { dev_maj = strtod(ptr, &end_ptr); ptr = end_ptr; }
		if (ptr && *(++ptr) != ',') { dev_min = strtod(ptr, &end_ptr); ptr = end_ptr; }
		if (ptr && *(++ptr) != ',') { north_deg = strtod(ptr, &end_ptr); ptr = end_ptr; }
		if (ptr && *(++ptr) != ',') { lat_err = strtod(ptr, &end_ptr); ptr = end_ptr; }
		if (ptr && *(++ptr) != ',') { lon_err = strtod(ptr, &end_ptr); ptr = end_ptr; }
		if (ptr && *(++ptr) != ',') { alt_err = strtod(ptr, &end_ptr); ptr = end_ptr; }

		EMLID_UNUSED(tmp_time);
		EMLID_UNUSED(rms_err);
		EMLID_UNUSED(dev_maj);
		EMLID_UNUSED(dev_min);
		EMLID_UNUSED(north_deg);

		_eph = sqrtf(static_cast<float>(lat_err) * static_cast<float>(lat_err)
					   + static_cast<float>(lon_err) * static_cast<float>(lon_err));
		_epv = static_cast<float>(alt_err);

		// TODO _gps_position->s_variance_m_s = 0;


	} else if (strncmp(_nmea_buff + NMEA_TYPE_OFFSET, NMEA_Satellite_in_view, NMEA_TYPE_LEN) == 0) {
		//GSV
		// ignore for now, describe each satellite locked elevation, azimuth and SNR GA|GL|GP

	} else if (strncmp(_nmea_buff + NMEA_TYPE_OFFSET, NMEA_recommended_minimum_data_for_gps, NMEA_TYPE_LEN) == 0) {
		// $--RMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxx,x.x,a,m,*hh<CR><LF>
		//        1         2 3       4 5        6 7   8   9    10  11 12
		// 1. UTC Time
		// 2. Status, V=Navigation receiver warning A=Valid 
		// 3. Latitude
		// 4. N|S
		// 5. Longitude
		// 6. E|W
		// 7. Speed over ground, knots
		// 8. Track made good, degrees true
		// 9. Date ddmmyy
		// 10. Magnetic Variation, degrees
		// 11. E|W
		// 12. FAA mode indicator

		// TODO Date here !


	} else if (strncmp(_nmea_buff + NMEA_TYPE_OFFSET, NMEA_Vector_Track_and_speed_over_Ground, NMEA_TYPE_LEN) == 0) {
		// VTG
		// $--VTG,x.x,T,x.x,M,x.x,N,x.x,K,m,*hh<CR><LF>
		//        1   2 3   4 5   6 7   8 9
		// 1. Track Degrees, course [0 - 360] to true north
		// 2. T = True
		// 3. Track Degrees, to magnetic north, to ignore
		// 4. M = Magnetic
		// 5. Speed Knots, use kmph !
		// 6. N = Knots
		// 7. Speed Kilometers Per Hour
		// 8. K = Kilometers Per Hour
		// 9. FAA mode indicator (NMEA 2.3 and later)

		_course_deg = -1;
		_speed_kmph = 0;

		if (ptr && *(++ptr) != ',') { _course_deg = strtod(ptr, &end_ptr); ptr = end_ptr; }
		/* ignore 2. to 6. */
		for (int i=2; i<= 6; i++) {
			if (ptr && *(++ptr) != ',') { }
		}
		if (ptr && *(++ptr) != ',') { _speed_kmph = strtod(ptr, &end_ptr); ptr = end_ptr; }

		//GPS_INFO("EMLIDREACH: $--VTG _course_deg %f  _speed_kmph %f", _course_deg, _speed_kmph);

	} else {
		GPS_INFO("EMLIDREACH: NMEA message type unknown \n %s \n %c %c %c", _nmea_buff, _nmea_buff[3], _nmea_buff[4], _nmea_buff[5]);
	}

	return update;
}


void
GPSDriverEmlidReach::computeNedVelocity()
{
	if (_course_deg < 0) {
		// no data to compute NED velocity from
		_gps_position->vel_m_s = 0;
		_gps_position->vel_n_m_s = 0;
		_gps_position->vel_e_m_s = 0;
		_gps_position->vel_d_m_s = 0;
		_gps_position->cog_rad = 0;		/**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
		_gps_position->vel_ned_valid = false;	/**< Flag to indicate if NED speed is valid */
		return;
	}

	_gps_position->vel_m_s = _speed_kmph / 3.6;
	_gps_position->vel_n_m_s = static_cast<float>(sin(_course_deg)) * _gps_position->vel_m_s;
	_gps_position->vel_e_m_s = static_cast<float>(cos(_course_deg)) * _gps_position->vel_m_s;
	_gps_position->vel_d_m_s = 0;
	// in rad, -PI..PI */
	_gps_position->cog_rad = _course_deg * 2 * GPS_PI / 360 - GPS_PI;
	_gps_position->vel_ned_valid = true;
}




