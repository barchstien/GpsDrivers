/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file emlid_reach.h
 *
 * @author Bastien Auneau <bastien.auneau@while-true.fr>
 */

/***********************
Questions:
 1. TODO support ERB https://files.emlid.com/ERB.pdf
 3. TODO time_utc_usec
 4. gps_absolute_time() whereis it from ? <--- usec since close to start, always counting up
 5. TODO ? date set upon RMV msg ?... not sure coz want to do ERB now...
************************/


#pragma once

#include "gps_helper.h"
#include "../../definitions.h"

// Emlid documentation
//   https://docs.emlid.com/reachm-plus/
// NMEA references
//   https://en.wikipedia.org/wiki/NMEA_0183
//   http://www.catb.org/gpsd/NMEA.html

#define NMEA_SENTENCE_MAX_LEN	82	// includes '$',<CR> and <LF> 
#define NMEA_CHECKSUM_LEN		2


/**
 * Driver class for Emlid Reach
 * Populates caller provided vehicle_gps_position_s when GGA received
 * Other NMEA messages are cached by the driver to complete messages emitted upon GGA
 */
class GPSDriverEmlidReach : public GPSHelper
{
public:
	GPSDriverEmlidReach(GPSCallbackPtr callback, void *callback_user, 
		struct vehicle_gps_position_s *gps_position,
		struct satellite_info_s *satellite_info
	);
	virtual ~GPSDriverEmlidReach() = default;

	int receive(unsigned timeout) override;
	int configure(unsigned &baudrate, OutputMode output_mode) override;

private:

	enum class NMEA_0183_State {
		init = 0,
		got_start_byte,		// $
		got_checksum_byte	// *
	};

	enum class NMEA_TALKER {
		GA = 0,
		GL,
		GP,
		SIZE
	};

	/** NMEA parser state machine */
	NMEA_0183_State _decode_state{NMEA_0183_State::init};

	/** Buffer used to receive data from serial*/
	uint8_t _read_buff[GPS_READ_BUFFER_SIZE];
	unsigned _read_buff_len{0};
	/** Pointer to next received byte to be processed, 
	 *  as data may been left unprocessed after NMEA message completed
	 */
	uint8_t *_read_buff_ptr{_read_buff + GPS_READ_BUFFER_SIZE};

	/** Buffer used by parser to build NMEA sentences */
	char _nmea_buff[NMEA_SENTENCE_MAX_LEN];
	unsigned _nmea_buff_len{0};

	/** Buffer used by parser to build NMEA checksum */
	char _checksum_buff[NMEA_CHECKSUM_LEN + 1]{0, 0, '\0'};
	unsigned _checksum_buff_len{0};

	/** Pointer provided by caller, ie gps.cpp */
	struct vehicle_gps_position_s *_gps_position {nullptr};
	/** Pointer provided by caller, gps.cpp */
	struct satellite_info_s *_satellite_info {nullptr};

	bool _testing_connection{false};
	unsigned _nmea_parse_err_cnt{0};
	unsigned _nmea_cnt{0};


	///// NMEA messages caches /////
	/** eph from GST message */
	double _eph{0.0};
	/** epv from GST message */
	double _epv{0.0};
	/** epv from VTG message */
	double _course_deg{-1};
	/** epv from VTG message */
	double _speed_kmph{0};

	/** Satellite info for GA, GL, GP (galileo, glonass, gps) */
	struct satellite_info_s _sat_info_array[static_cast<int>(NMEA_TALKER::SIZE)];
	//unsigned _sat_info_cnt[static_cast<int>(NMEA_TALKER::SIZE)];


	/** Set NMEA parser state when found $ start byte */
	void nmeaParserRestart();
	/** Feed NMEA parser with received bytes from serial 
	 * @return len of decoded message, 0 if not completed, -1 if error
	 */
	int parseChar(uint8_t b);

	/** Fit an NMEA sentence into vehicle_gps_position_s, to be used by caller, ie GPSHelper 
	 *  @return true if gps_position has been updated
	 */
	int handleNmeaSentence();

	void computeNedVelocity();

	bool testConnection();

};

