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

#pragma once

#include "gps_helper.h"
#include "../../definitions.h"

// NMEA references
// https://en.wikipedia.org/wiki/NMEA_0183
// http://www.catb.org/gpsd/NMEA.html

#define NMEA_SENTENCE_MAX_LEN	82	// includes '$',<CR> and <LF> 
#define NMEA_CHECKSUM_LEN		2

/**
 * Driver class for Emlid Reach
 */
class GPSDriverEmlidReach : public GPSHelper
{
public:
	GPSDriverEmlidReach(GPSCallbackPtr callback, void *callback_user, struct vehicle_gps_position_s *gps_position);
	virtual ~GPSDriverEmlidReach() = default;

	int receive(unsigned timeout) override;
	int configure(unsigned &baudrate, OutputMode output_mode) override;

private:

	enum class NMEA_0183_State {
		init = 0,
		got_start_byte,		// $
		got_checksum_byte	// *
	};

	/** NMEA parser state machine */
	NMEA_0183_State _decode_state{NMEA_0183_State::init};

	/** Buffer used by parser to build NMEA sentences */
	char _rx_buff[NMEA_SENTENCE_MAX_LEN];
	unsigned _rx_buff_len{0};

	/** Buffer used by parser to build NMEA checksum */
	char _checksum_buff[NMEA_CHECKSUM_LEN + 1]{0, 0, '\0'};
	unsigned _checksum_buff_len{0};

	/** Pointer to object provided by caller, ie GPSHelper */
	struct vehicle_gps_position_s *_gps_position {nullptr};

	/** Set NMEA parser state when found $ start byte */
	void nmeaParserRestart();
	/** Feed NMEA parser with received bytes from serial */
	int parseChar(uint8_t b);

// TODO delete
#if 0
	/** Find next ',' (comma) separated field from NMEA sentence 
	 *  @param sentence the NMEA sentence
	 *  @param offset from which to search for a field
	 *  @param pointer to sentence content where a field begins
	 *  @param length of the field found
	 */
	void getField(char* sentence, unsigned sentence_len, unsigned offset, char** field, unsigned* field_len);
#endif

	/** Fit an NMEA sentence into vehicle_gps_position_s, to be used by caller, ie GPSHelper */
	bool handleNmeaSentence();


	// TODO delete 
	// debug 
	unsigned _nmea_cnt{0};

};
