/****************************************************************************
 *
 *   Copyright (c) 2012-2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_iridium.h
 *
 */

#pragma once

#include "../../mavlink/include/mavlink/v2.0/common/mavlink.h"
#include "drivers/drv_hrt.h"

class MavlinkIridium
{
public:
	MavlinkIridium();

	const char *get_name() const
	{
		return MavlinkIridium::get_name_static();
	}

	static const char *get_name_static()
	{
		return "HIGH_LATENCY2";
	}

	static uint16_t get_id_static()
	{
		return MAVLINK_MSG_ID_HIGH_LATENCY2;
	}

	uint16_t get_id()
	{
		return get_id_static();
	}

	unsigned get_size()
	{
		return MAVLINK_MSG_ID_HIGH_LATENCY2_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
	}

	bool const_rate()
	{
		return true;
	}

	void updateData(const hrt_abstime t);
	void printData();

private:
	int _actuator_sub_0;
	int _actuator_sub_1;
	int _airspeed_sub;
	int _attitude_sp_sub;
	int _battery_sub;
	int _estimator_status_sub;
	int _pos_ctrl_status_sub;
	int _geofence_sub;
	int _global_pos_sub;
	int _gps_sub;
	int _mission_result_sub;
	int _status_sub;
	int _status_flags_sub;
	int _tecs_status_sub;
	int _wind_sub;
	mavlink_high_latency2_t* msg;

protected:

	void write_airspeed();

	void write_attitude_sp();

	void write_battery_status();

	void write_estimator_status();

	void write_fw_ctrl_status();

	void write_geofence_result();

	void write_global_position();

	void write_mission_result();

	void write_tecs_status();

	void write_vehicle_status();

	void write_vehicle_status_flags();

	void write_wind_estimate();
	
	void write_gps();

	void set_default_values();
};
