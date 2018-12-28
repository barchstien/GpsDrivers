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

//// NMEA
#define NMEA_SYMBOL_START    0x24 // $
#define NMEA_SYMBOL_CHECKSUM 0x2a // *
#define NMEA_SYMBOL_CR       0x0d // <CR>
#define NMEA_SYMBOL_LF       0x0a // <LF>

#define NMEA_TALKER_OFFSET 1
#define NMEA_TALKER_LEN    2

// Emlid reach emits messages :
//   GAGSA, GAGSV
//   GLGSA, GLGSV
//   GPGSA, GPGSV
//   GNGGA, GNGST, GNRMC, GNVTG

// NMEA talker's id
#define NMEA_TALKER_GALILEO "GA"
#define NMEA_TALKER_GLONASS "GL"
#define NMEA_TALKER_GNSS    "GN"    // Mixed GPS and GLONASS data, according to IEIC 61162-1
#define NMEA_TALKER_GPS     "GP"

#define NMEA_TYPE_OFFSET 3
#define NMEA_TYPE_LEN    3

// NMEA sentence types
#define NMEA_Fix_information                    "GGA" //< GN talker only
#define NMEA_Overall_Satellite_data             "GSA" //< GA|GL|GP
#define NMEA_GPS_Pseudorange_Noise_Statistics   "GST" //< GN talker only
#define NMEA_Satellite_in_view                  "GSV" //< GA|GL|GP
#define NMEA_recommended_minimum_data_for_gps   "RMC" //< GN talker only
#define NMEA_Vector_Track_and_speed_over_Ground "VTG" //< GN talker only

//// ERB
// 'R' Ox82 | 'E' Ox45 | ID | LENGTH (2B little endian) | payload | CHECKSUM (2B)
#define ERB_SYNC_1 0x45 // E
#define ERB_SYNC_2 0x52 // R

#define ERB_ID_VERSION         0x01
#define ERB_ID_GEODIC_POSITION 0x02
#define ERB_ID_NAV_STATUS      0x03
#define ERB_ID_DOPS            0x04
#define ERB_ID_VELOCITY_NED    0x05
#define ERB_ID_SPACE_INFO      0x06 // not used, reduces stack usage
#define ERB_ID_RTK             0x07 // RTK, TODO really ?


#define EMLID_UNUSED(x) (void)x;

#define GPS_PI 3.141592653589793238462643383280

#define AUTO_DETECT_MAX_TIMEOUT       10 // if more detect failed AND
#define AUTO_DETECT_MAX_PARSE_ERR     2  // if more detect failed AND
#define AUTO_DETECT_MAX_READ_SENTENCE 10 // if more detect succeed

#define TYPE_STR(t) ((t == PARSER_TYPE::NMEA) ? "NMEA" : "ERB")


GPSDriverEmlidReach::GPSDriverEmlidReach(GPSCallbackPtr callback, void *callback_user, 
	struct vehicle_gps_position_s *gps_position, struct satellite_info_s *satellite_info) :
	GPSHelper(callback, callback_user),
	_gps_position(gps_position), _satellite_info(satellite_info)
{
	memset(_sat_info_array, 0, static_cast<int>(NMEA_TALKER::SIZE) * sizeof(satellite_info_s));
}


int
GPSDriverEmlidReach::configure(unsigned &baudrate, OutputMode output_mode)
{
	// TODO RTK
	if (output_mode != OutputMode::GPS) {
		GPS_WARN("EMLIDREACH: Unsupported Output Mode %i", (int)output_mode);
		return -1;
	}

	unsigned baud_allowed[]{57600, 115200, 230400};
	PARSER_TYPE types[]{PARSER_TYPE::NMEA, PARSER_TYPE::ERB};

	for (unsigned k=0; k<sizeof(types) / sizeof(types[0]); k++) {
		_parser_type = types[k];
		for (unsigned i=0; i<sizeof(baud_allowed) / sizeof(baud_allowed[0]); i++) {
			if (baudrate > 0 && baudrate != baud_allowed[i]) {
				continue;
			}
			_parse_err_cnt = 0;
			_sentence_cnt = 0;
			_decode_state.nmea = NMEA_0183_State::init;
			_decode_state.erb  = ERB_State::init;

			if (GPSHelper::setBaudrate(baud_allowed[i]) != 0) {
				continue;
			}
			GPS_INFO("EmlidReach: testConnection: %d type: %s", baud_allowed[i], TYPE_STR(_parser_type));
			if (! testConnection()) {
				continue;
			}

			GPS_INFO("EmlidReach: config OK with baudrate: %d type: %s", baud_allowed[i], TYPE_STR(_parser_type));
			baudrate = baud_allowed[i];
			return 0;
		}
	}

	return -1;
}


bool
GPSDriverEmlidReach::testConnection()
{
	_testing_connection = true;

	unsigned timeout_cnt = 0;
	while (timeout_cnt < AUTO_DETECT_MAX_TIMEOUT 
		&& _parse_err_cnt < AUTO_DETECT_MAX_PARSE_ERR
		&& _sentence_cnt < AUTO_DETECT_MAX_READ_SENTENCE)
	{
		if (receive(50000) == 0) { // timeout larger than what defined in read() are truncated
			timeout_cnt ++;
		}
	}
	GPS_WARN("+++++     _sentence_cnt %d  timeout_cnt %d  _parse_err_cnt %d", _sentence_cnt, timeout_cnt, _parse_err_cnt);

	_testing_connection = false;
	return timeout_cnt < AUTO_DETECT_MAX_TIMEOUT && _parse_err_cnt < AUTO_DETECT_MAX_PARSE_ERR;
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
				//GPS_INFO("ERB: %s", _read_buff);
				//GPS_INFO("_parse_err_cnt: %d", _parse_err_cnt);
			} else {
				// timeout occured
				if (_testing_connection) {
					return 0;
				}
			}
		} else {
			// process data in buffer from previous read
			while (_read_buff_ptr < _read_buff + _read_buff_len) {
				int ret = 0;
				if (_parser_type == PARSER_TYPE::NMEA) {
					ret = nmeaParseChar(*(_read_buff_ptr++));
				} else {
					ret = erbParseChar(*(_read_buff_ptr++));
				}
				if (ret > 0) {
					_sentence_cnt ++;
					if (_parser_type == PARSER_TYPE::NMEA) {
						ret = handleNmeaSentence();
					} else {
						ret = handleErbSentence();
					}
					if (ret > 0) {
						return ret;
					}
				} else if (ret < 0) {
					_parse_err_cnt ++;
				}
			}
		}
	}
}



//// ERB

int
GPSDriverEmlidReach::erbParseChar(uint8_t b)
{
	int ret = 0;
	//GPS_INFO(" ERB PARSE: %02x", b);
	switch (_decode_state.erb) {
	case ERB_State::init:
		if (b == ERB_SYNC_1) {
			_buff_len = 0;
			_buff[_buff_len ++] = b;
			_decode_state.erb = ERB_State::got_sync_1;
			//GPS_INFO(" ERB GOT SYNC 1");
		}
		break;
	case ERB_State::got_sync_1:
		if (b == ERB_SYNC_1) {
			_buff_len = 0;
			_buff[_buff_len ++] = b;
			_decode_state.erb = ERB_State::got_sync_1;
			_parse_err_cnt ++;
		} else if (b == ERB_SYNC_2) {
			_buff[_buff_len ++] = b;
			_decode_state.erb = ERB_State::got_sync_2;
			//GPS_INFO(" ERB GOT SYNC 2");
		} else {
			_decode_state.erb = ERB_State::init;
			_parse_err_cnt ++;
		}
		break;
	case ERB_State::got_sync_2:
		if (b >= ERB_ID_VERSION && b <= ERB_ID_RTK) {
			_buff[_buff_len ++] = b;
			// debug TODO
			//_decode_state.erb = ERB_State::got_id;
			ret = 1;
			_decode_state.erb = ERB_State::init;
			//GPS_INFO(" ERB GOT Header %s", _buff);
		} else {
			_decode_state.erb = ERB_State::init;
			_parse_err_cnt ++;
			GPS_INFO(" ERB GOT Err on %02x", b);
		}
		break;
	case ERB_State::got_id:
		
		break;
	case ERB_State::got_len_1:
		
		break;
	case ERB_State::got_len_2:
		
		break;
	case ERB_State::got_payload:
		
		break;
	case ERB_State::got_CK_A:
		
		break;
	}

	return ret;
}

int
GPSDriverEmlidReach::handleErbSentence()
{
//	char *ptr = _buff + 6;
//	char *end_ptr;
	int ret = 0;

	// debug tests
	if (_testing_connection)
		ret = 1;

	return ret;
}


//// NMEA

void
GPSDriverEmlidReach::nmeaParserRestart()
{
	_decode_state.nmea = NMEA_0183_State::got_start_byte;
	_buff_len = 0;
}


int
GPSDriverEmlidReach::nmeaParseChar(uint8_t b)
{
	int ret = 0;
	switch (_decode_state.nmea) {
	case NMEA_0183_State::init:
		if (b == NMEA_SYMBOL_START) {
			nmeaParserRestart();
			_buff[_buff_len ++] = b;
		}
		break;

	case NMEA_0183_State::got_start_byte:
		if (b == NMEA_SYMBOL_START) {
			nmeaParserRestart();
			_buff[_buff_len ++] = b;
			GPS_WARN("EMLIDREACH: NMEA message truncated");
			ret = -1;
		} else if (b == NMEA_SYMBOL_CHECKSUM) {
			_decode_state.nmea = NMEA_0183_State::got_checksum_byte;
			_checksum_buff_len = 0;
		} else if (_buff_len >= SENTENCE_MAX_LEN) {
			GPS_WARN("EMLIDREACH: NMEA message overflow");
			_decode_state.nmea = NMEA_0183_State::init;
			ret = -1;
		} else {
			_buff[_buff_len++] = b;
		}
		break;

	case NMEA_0183_State::got_checksum_byte:
		if (b == NMEA_SYMBOL_START) {
			nmeaParserRestart();
			_buff[_buff_len ++] = b;
			GPS_WARN("EMLIDREACH: NMEA message truncated");
			ret = -1;
		} else if (b == NMEA_SYMBOL_CR) {
			// compute expected checksum
			// https://en.wikipedia.org/wiki/NMEA_0183#C_implementation_of_checksum_generation
			int cs = 0;
			for (unsigned i=1; i<_buff_len; i++){
				cs ^= _buff[i];
			}

			// convert read checksum to int
			int read_cs = 0;
			read_cs = strtol(_checksum_buff, nullptr, 16);
			if (read_cs == 0) {
				if (errno == ERANGE) {
					GPS_WARN("EMLIDREACH: NMEA checksum extraction failed: %s", _checksum_buff);
					_decode_state.nmea = NMEA_0183_State::init;
					ret = -1;
				}
			}

			if (cs != read_cs){
				GPS_WARN("EMLIDREACH: NMEA checksum failed, expectd %02x, got %02x \n %s", cs, read_cs, _buff);
				_decode_state.nmea = NMEA_0183_State::init;
				ret = -1;
			} else {
				_decode_state.nmea = NMEA_0183_State::wait_for_LF;
			}
		} else if (_checksum_buff_len >= CHECKSUM_LEN) {
			GPS_WARN("EMLIDREACH: NMEA message checksum overflow");
			_decode_state.nmea = NMEA_0183_State::init;
			ret = -1;
		} else {
			_checksum_buff[_checksum_buff_len++] = b;
		}

		break;

	case NMEA_0183_State::wait_for_LF:
		if (b == NMEA_SYMBOL_LF) {
			//GPS_INFO("EMLIDREACH: NMEA sentence completed len:%d \n %s", _buff_len, _buff);
			// return length of formed buffer and re-init state machine for next sentence
			// null terminate in case it's printed later on, while debug/fixing
			// safe length-wise because SENTENCE_MAX_LEN could contain checksum, CR and LF
			_buff[_buff_len] = '\0';
			ret = _buff_len;
			_decode_state.nmea = NMEA_0183_State::init;
		} else {
			ret = -1;
		}
		break;

	}

	return ret;
}


int
GPSDriverEmlidReach::handleNmeaSentence()
{
	// pass talker + type field, ie $--GGA
	char *ptr = _buff + 6;
	char *end_ptr;
	int ret = 0;

	if (strncmp(_buff + NMEA_TYPE_OFFSET, NMEA_Fix_information, NMEA_TYPE_LEN) == 0) {
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

		_gps_position->hdop = hdop;
		_gps_position->vdop = 0;

		// vehicle_gps_position_s::fix_type
		//  0-1: no fix,
		//  2: 2D fix,
		//  3: 3D fix,
		//  4: RTCM code differential,
		//  5: Real-Time Kinematic, float,
		//  6: Real-Time Kinematic, fixed,
		//  8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.

		uint8_t fix_mode = sat_in_view >= 4 ? 3 : 2; // 3d vs 2d fix
		if (fix_quality == 0)      { _gps_position->fix_type = 0; }
		else if (fix_quality == 1) { _gps_position->fix_type = fix_mode; }
		else if (fix_quality <= 3) { _gps_position->fix_type = 4; } // differential & PPS
		else if (fix_quality == 4) { _gps_position->fix_type = 6; } // RTK fixed
		else if (fix_quality == 5) { _gps_position->fix_type = 5; } // RTF float
		else                       { _gps_position->fix_type = 0; }

		_gps_position->satellites_used = sat_in_view;

		// emlid forum says to calculate from last GPS position with low filter, for NMEA
		computeNedVelocity();

		ret = 1;

	} else if (strncmp(_buff + NMEA_TYPE_OFFSET, NMEA_Overall_Satellite_data, NMEA_TYPE_LEN) == 0) {
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

		EMLID_UNUSED(hdop);
		EMLID_UNUSED(vdop);
		EMLID_UNUSED(pdop);
		EMLID_UNUSED(fix_mode);


	} else if (strncmp(_buff + NMEA_TYPE_OFFSET, NMEA_GPS_Pseudorange_Noise_Statistics, NMEA_TYPE_LEN) == 0) {
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


	} else if (strncmp(_buff + NMEA_TYPE_OFFSET, NMEA_Satellite_in_view, NMEA_TYPE_LEN) == 0) {
		// GSV
		// Describe each satellite locked elevation, azimuth and SNR GA|GL|GP
		// $--GSV,x,x,x,x,x,x,x,...*hh<CR><LF>
		//        1 2 3 4 5 6 7
		// 1. Total GSV messages in group
		// 2. Index of this GSV message, starting at 1
		// 3. Total number of sat in view
		// 4. Satellite PRN number (Pseudo-Random Noise or Gold Codes)
		// 5. Elevation in degrees [-90, 90]
		// 6. Azimth in degrees to true north [000, 359]
		// 7. SNR in dB [00, 99]

		unsigned total_gsv_msg = 0, index_gsv_msg = 0, total_sat = 0;

		unsigned talker_ind = 0;
		if (strncmp(_buff + NMEA_TALKER_OFFSET, NMEA_TALKER_GALILEO, NMEA_TALKER_LEN) == 0) {
			talker_ind = static_cast<int>(NMEA_TALKER::GA);
		} else if (strncmp(_buff + NMEA_TALKER_OFFSET, NMEA_TALKER_GLONASS, NMEA_TALKER_LEN) == 0) {
			talker_ind = static_cast<int>(NMEA_TALKER::GL);
		} else if (strncmp(_buff + NMEA_TALKER_OFFSET, NMEA_TALKER_GPS, NMEA_TALKER_LEN) == 0) {
			talker_ind = static_cast<int>(NMEA_TALKER::GP);
		} else {
			// Talker id not supported, ignore message
			return 0;
		}

		// extract data
		if (ptr && *(++ptr) != ',') { total_gsv_msg = strtol(ptr, &end_ptr, 10); ptr = end_ptr; }
		if (ptr && *(++ptr) != ',') { index_gsv_msg = strtol(ptr, &end_ptr, 10); ptr = end_ptr; }
		if (ptr && *(++ptr) != ',') { total_sat = strtol(ptr, &end_ptr, 10); ptr = end_ptr; }

		if (total_gsv_msg == 0 || index_gsv_msg == 0 || total_sat== 0) {
			return 0;
		}

		if (index_gsv_msg == 1) {
			// start of new group
			memset(&_sat_info_array[talker_ind], 0, sizeof(satellite_info_s));
		}

		unsigned sat_cnt = (index_gsv_msg - 1) * 4;
		for (unsigned i=0; i<4 && sat_cnt < total_sat; i++) {
			unsigned id = 0, elevation = 0, azimuth = 0, snr = 0;

			if (ptr && *(++ptr) != ',') { id = strtol(ptr, &end_ptr, 10); ptr = end_ptr; }
			if (ptr && *(++ptr) != ',') { elevation = strtol(ptr, &end_ptr, 10); ptr = end_ptr; }
			if (ptr && *(++ptr) != ',') { azimuth = strtol(ptr, &end_ptr, 10); ptr = end_ptr; }
			if (ptr && *(++ptr) != ',') { snr = strtol(ptr, &end_ptr, 10); ptr = end_ptr; }

			_sat_info_array[talker_ind].svid[sat_cnt]      = id;
			_sat_info_array[talker_ind].used[sat_cnt]      = snr > 0;
			_sat_info_array[talker_ind].elevation[sat_cnt] = elevation;
			_sat_info_array[talker_ind].azimuth[sat_cnt]   = azimuth;
			_sat_info_array[talker_ind].snr[sat_cnt]       = snr;
			sat_cnt ++;
		}

		if (total_sat == sat_cnt && _satellite_info){
			// sequence ended
			_sat_info_array[talker_ind].count = sat_cnt;
			if (sat_cnt > satellite_info_s::SAT_INFO_MAX_SATELLITES) {
				_sat_info_array[talker_ind].count = satellite_info_s::SAT_INFO_MAX_SATELLITES;
			}
			_sat_info_array[talker_ind].timestamp = gps_absolute_time();

			memcpy(_satellite_info, &_sat_info_array[talker_ind], sizeof(satellite_info_s));
			ret = 2;
		}

	} else if (strncmp(_buff + NMEA_TYPE_OFFSET, NMEA_recommended_minimum_data_for_gps, NMEA_TYPE_LEN) == 0) {
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


	} else if (strncmp(_buff + NMEA_TYPE_OFFSET, NMEA_Vector_Track_and_speed_over_Ground, NMEA_TYPE_LEN) == 0) {
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

		//if (_course_deg >= 0)
		// GPS_INFO("EMLIDREACH: $--VTG _course_deg %f  _speed_kmph %f", _course_deg, _speed_kmph);

	} else {
		GPS_INFO("EMLIDREACH: NMEA message type unknown \n %s \n %c %c %c", _buff, _buff[3], _buff[4], _buff[5]);
	}

	return ret;
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
		_gps_position->cog_rad = 0;	 /**< Course over ground (NOT heading, but direction of movement) in rad, -PI..PI */
		_gps_position->vel_ned_valid = false; /**< Flag to indicate if NED speed is valid */
		return;
	}

	_gps_position->vel_m_s = _speed_kmph / 3.6;
	// sin/cos goes clock wise, starting from east
	// course goes anti clock wise, starting from north
	_gps_position->vel_n_m_s = static_cast<float>(cosf(_course_deg * GPS_PI / 180.0)) * _gps_position->vel_m_s;
	_gps_position->vel_e_m_s = static_cast<float>(sinf(_course_deg * GPS_PI / 180.0)) * _gps_position->vel_m_s;
	_gps_position->vel_d_m_s = 0;
	// in rad, -PI..PI */
	_gps_position->cog_rad = _course_deg * GPS_PI / 180.0 - GPS_PI;
	_gps_position->vel_ned_valid = true;
	_rate_count_vel++;
}



