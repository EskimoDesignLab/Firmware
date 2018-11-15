/****************************************************************************
 *
 *   Copyright (c) 2015-2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_iridium.cpp
 *
 */

#include "mavlink_iridium.h"

#include <commander/px4_custom_mode.h>
#include <lib/ecl/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/geofence_result.h>
#include <uORB/topics/mission_result.h>
#include <uORB/topics/position_controller_status.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_status_flags.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/wind_estimate.h>
#include <uORB/uORB.h>

using matrix::wrap_2pi;

MavlinkIridium::MavlinkIridium()
{
	highLatencyActive = false;
}

void MavlinkIridium::subscribeToTopics()
{
	_actuator_sub_0 = orb_subscribe(ORB_ID(actuator_controls_0));
	_actuator_sub_1 = orb_subscribe(ORB_ID(actuator_controls_1));
	_airspeed_sub = orb_subscribe(ORB_ID(airspeed));
	_attitude_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_battery_sub = orb_subscribe(ORB_ID(battery_status));
	_estimator_status_sub = orb_subscribe(ORB_ID(estimator_status));
	_pos_ctrl_status_sub = orb_subscribe(ORB_ID(position_controller_status));
	_geofence_sub = orb_subscribe(ORB_ID(geofence_result));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_mission_result_sub = orb_subscribe(ORB_ID(mission_result));
	_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_status_flags_sub = orb_subscribe(ORB_ID(vehicle_status_flags));
	_tecs_status_sub = orb_subscribe(ORB_ID(tecs_status));
	_wind_sub = orb_subscribe(ORB_ID(wind_estimate));
}


void MavlinkIridium::updateData(const hrt_abstime t)
{
	set_default_values();

	write_airspeed();
	write_attitude_sp();
	write_battery_status();
	write_estimator_status();
	write_fw_ctrl_status();
	write_geofence_result();
	write_global_position();
	write_mission_result();
	write_tecs_status();
	write_vehicle_status();
	write_vehicle_status_flags();
	write_wind_estimate();
	write_gps();

	iridiumMessage.msg.timestamp = t / 1000;
	iridiumMessage.msg.type = 1;
	iridiumMessage.msg.autopilot = MAV_AUTOPILOT_PX4;
}

void MavlinkIridium::write_airspeed()
{
	struct airspeed_s airspeed;

	orb_copy(ORB_ID(airspeed), _airspeed_sub, &airspeed);

	if (airspeed.confidence < 0.95f) { // the same threshold as for the commander
		iridiumMessage.msg.failure_flags |= HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE;
	}

	iridiumMessage.msg.airspeed = airspeed.indicated_airspeed_m_s * 5;
	iridiumMessage.msg.temperature_air = airspeed.air_temperature_celsius;
}

void MavlinkIridium::write_attitude_sp()
{
	struct vehicle_attitude_setpoint_s attitude_sp;

	orb_copy(ORB_ID(vehicle_attitude_setpoint), _attitude_sp_sub, &attitude_sp);

	iridiumMessage.msg.target_heading = static_cast<uint8_t>(math::degrees(wrap_2pi(attitude_sp.yaw_body)) * 0.5f);
}

void MavlinkIridium::write_battery_status()
{
	struct battery_status_s battery;

	orb_copy(ORB_ID(battery_status), _battery_sub, &battery);


	if (battery.warning > battery_status_s::BATTERY_WARNING_LOW) {
		iridiumMessage.msg.failure_flags |= HL_FAILURE_FLAG_BATTERY;
	}
	iridiumMessage.msg.battery = int32_t(battery.remaining * 100.0f);

}

void MavlinkIridium::write_estimator_status()
{
	struct estimator_status_s estimator_status;

	orb_copy(ORB_ID(estimator_status), _estimator_status_sub, &estimator_status);

	if (estimator_status.gps_check_fail_flags > 0 ||
		estimator_status.filter_fault_flags > 0 ||
		estimator_status.innovation_check_flags > 0) {
		iridiumMessage.msg.failure_flags |= HL_FAILURE_FLAG_ESTIMATOR;
	}
}

void MavlinkIridium::write_fw_ctrl_status()
{
	position_controller_status_s pos_ctrl_status = {};

	orb_copy(ORB_ID(position_controller_status), _pos_ctrl_status_sub, &pos_ctrl_status);

	iridiumMessage.msg.target_distance = pos_ctrl_status.wp_dist * 0.1f;

}

void MavlinkIridium::write_geofence_result()
{
	struct geofence_result_s geofence;

	orb_copy(ORB_ID(geofence_result), _geofence_sub, &geofence);

	if (geofence.geofence_violated) {
		iridiumMessage.msg.failure_flags |= HL_FAILURE_FLAG_GEOFENCE;
	}
}

void MavlinkIridium::write_global_position()
{
	struct vehicle_global_position_s global_pos;

	orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &global_pos);

	// iridiumMessage.msg.latitude = global_pos.lat * 1e7;
	// iridiumMessage.msg.longitude = global_pos.lon * 1e7;

	if (global_pos.alt > 0) {
		iridiumMessage.msg.altitude = global_pos.alt + 0.5f;

	} else {
		iridiumMessage.msg.altitude = global_pos.alt - 0.5f;
	}

	iridiumMessage.msg.heading = static_cast<uint8_t>(math::degrees(wrap_2pi(global_pos.yaw)) * 0.5f);
	iridiumMessage.msg.groundspeed = sqrtf(global_pos.vel_n * global_pos.vel_n + global_pos.vel_e * global_pos.vel_e) * 5.0f;
	iridiumMessage.msg.climb_rate = fabsf(global_pos.vel_d) * 10.0f;

}

void MavlinkIridium::write_mission_result()
{
	struct mission_result_s mission_result;

	orb_copy(ORB_ID(mission_result), _mission_result_sub, &mission_result);

	iridiumMessage.msg.wp_num = mission_result.seq_current;
}

void MavlinkIridium::write_tecs_status()
{
	struct tecs_status_s tecs_status;

	orb_copy(ORB_ID(tecs_status), _tecs_status_sub, &tecs_status);
	iridiumMessage.msg.airspeed_sp = tecs_status.airspeed_sp * 5.0f; 
	iridiumMessage.msg.target_altitude = tecs_status.altitude_sp;

}

void MavlinkIridium::write_vehicle_status()
{
	struct vehicle_status_s status;

	orb_copy(ORB_ID(vehicle_status), _status_sub, &status);

	highLatencyActive = status.high_latency_data_link_active;
	PX4_INFO("High latency : %d",highLatencyActive);

	if ((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE)
		&& !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE)) {
		iridiumMessage.msg.failure_flags |= HL_FAILURE_FLAG_ABSOLUTE_PRESSURE;
	}

	if (((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_ACCEL)
			&& !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_ACCEL)) ||
		((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_ACCEL2) &&
			!(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_ACCEL2))) {
		iridiumMessage.msg.failure_flags |= HL_FAILURE_FLAG_3D_ACCEL;
	}

	if (((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_GYRO)
			&& !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_GYRO)) ||
		((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_GYRO2) &&
			!(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_GYRO2))) {
		iridiumMessage.msg.failure_flags |= HL_FAILURE_FLAG_3D_GYRO;
	}

	if (((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_MAG)
			&& !(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_MAG)) ||
		((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_SENSOR_3D_MAG2) &&
			!(status.onboard_control_sensors_health & MAV_SYS_STATUS_SENSOR_3D_MAG2))) {
		iridiumMessage.msg.failure_flags |= HL_FAILURE_FLAG_3D_MAG;
	}

	if ((status.onboard_control_sensors_enabled & MAV_SYS_STATUS_TERRAIN)
		&& !(status.onboard_control_sensors_health & MAV_SYS_STATUS_TERRAIN)) {
		iridiumMessage.msg.failure_flags |= HL_FAILURE_FLAG_TERRAIN;
	}

	if (status.rc_signal_lost) {
		iridiumMessage.msg.failure_flags |= HL_FAILURE_FLAG_RC_RECEIVER;
	}

	if (status.engine_failure) {
		iridiumMessage.msg.failure_flags |= HL_FAILURE_FLAG_ENGINE;
	}

	if (status.mission_failure) {
		iridiumMessage.msg.failure_flags |= HL_FAILURE_FLAG_MISSION;
	}

	// flight mode
	union px4_custom_mode custom_mode;
	switch (status.nav_state) {
		case vehicle_status_s::NAVIGATION_STATE_MANUAL:
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_MANUAL;
			break;
		case vehicle_status_s::NAVIGATION_STATE_ACRO:
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_ACRO;
			break;
		case vehicle_status_s::NAVIGATION_STATE_RATTITUDE:
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_RATTITUDE;
			break;
		case vehicle_status_s::NAVIGATION_STATE_STAB:
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_STABILIZED;
			break;
		case vehicle_status_s::NAVIGATION_STATE_ALTCTL:
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_ALTCTL;
			break;
		case vehicle_status_s::NAVIGATION_STATE_POSCTL:
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_POSCTL;
			break;
		case vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF:
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_TAKEOFF;
			break;
		case vehicle_status_s::NAVIGATION_STATE_AUTO_MISSION:
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_MISSION;break;
		case vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER:
			custom_mode.main_mode = PX4_CUSTOM_MAIN_MODE_AUTO;
			custom_mode.sub_mode = PX4_CUSTOM_SUB_MODE_AUTO_LOITER;	break;
	}
	iridiumMessage.msg.custom_mode = custom_mode.custom_mode_hl;

	if (status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		struct actuator_controls_s actuator = {};
		orb_copy(ORB_ID(actuator_controls_0), _actuator_sub_0, &actuator);


		if (status.is_vtol && !status.is_rotary_wing) {
			iridiumMessage.msg.throttle = actuator.control[actuator_controls_s::INDEX_THROTTLE] * 100.0f; 
		}

		else {				
			iridiumMessage.msg.throttle = actuator.control[actuator_controls_s::INDEX_THROTTLE] * 100.0f;
		}

		} else {
			iridiumMessage.msg.throttle = 0;
		}
}

void MavlinkIridium::write_vehicle_status_flags()
{
	struct vehicle_status_flags_s status_flags;

	orb_copy(ORB_ID(vehicle_status_flags), _status_flags_sub, &status_flags);


	if (!status_flags.condition_global_position_valid) { //TODO check if there is a better way to get only GPS failure
		iridiumMessage.msg.failure_flags |= HL_FAILURE_FLAG_GPS;
	}

	if (status_flags.offboard_control_signal_lost && status_flags.offboard_control_signal_found_once) {
		iridiumMessage.msg.failure_flags |= HL_FAILURE_FLAG_OFFBOARD_LINK;
	}

}

void MavlinkIridium::write_wind_estimate()
{
	struct wind_estimate_s wind;

	orb_copy(ORB_ID(wind_estimate), _wind_sub, &wind);

	iridiumMessage.msg.wind_heading = static_cast<uint8_t>(
					math::degrees(wrap_2pi(atan2f(wind.windspeed_east, wind.windspeed_north))) * 0.5f);
	iridiumMessage.msg.windspeed = sqrtf(wind.windspeed_north * wind.windspeed_north + wind.windspeed_east * wind.windspeed_east) * 5.0f;
}


void MavlinkIridium::write_gps()
{
	struct vehicle_gps_position_s gps;
	orb_copy(ORB_ID(vehicle_gps_position), _gps_sub, &gps);

	iridiumMessage.msg.latitude = gps.lat;
	iridiumMessage.msg.longitude = gps.lon;
	iridiumMessage.msg.eph = (int)(gps.eph * 10);
	iridiumMessage.msg.epv = (int)(gps.epv * 10);
}


void MavlinkIridium::set_default_values()
{
	iridiumMessage.msg.airspeed = 0;
	iridiumMessage.msg.airspeed_sp = 0;
	iridiumMessage.msg.altitude = 0;
	iridiumMessage.msg.autopilot = MAV_AUTOPILOT_ENUM_END;
	iridiumMessage.msg.battery = -1;
	iridiumMessage.msg.climb_rate = 0;
	iridiumMessage.msg.custom0 = INT8_MIN;
	iridiumMessage.msg.custom1 = INT8_MIN;
	iridiumMessage.msg.custom2 = INT8_MIN;
	iridiumMessage.msg.eph = UINT8_MAX;
	iridiumMessage.msg.epv = UINT8_MAX;
	iridiumMessage.msg.failure_flags = 0;
	iridiumMessage.msg.custom_mode = 0;
	iridiumMessage.msg.groundspeed = 0;
	iridiumMessage.msg.heading = 0;
	iridiumMessage.msg.latitude = 0;
	iridiumMessage.msg.longitude = 0;
	iridiumMessage.msg.target_altitude = 0;
	iridiumMessage.msg.target_distance = 0;
	iridiumMessage.msg.target_heading = 0;
	iridiumMessage.msg.temperature_air = INT8_MIN;
	iridiumMessage.msg.throttle = 0;
	iridiumMessage.msg.timestamp = 0;
	iridiumMessage.msg.type = MAV_TYPE_ENUM_END;
	iridiumMessage.msg.wind_heading = 0;
	iridiumMessage.msg.windspeed = 0;
	iridiumMessage.msg.wp_num = UINT16_MAX;
}

void MavlinkIridium::printData()
{
	PX4_INFO("Timestamp : %d", iridiumMessage.msg.timestamp);
	PX4_INFO("Latitude : %d", iridiumMessage.msg.latitude);
	PX4_INFO("Longitude : %d", iridiumMessage.msg.longitude);
	PX4_INFO("Custom mode : %d", iridiumMessage.msg.custom_mode);
	PX4_INFO("Altitude : %d", iridiumMessage.msg.altitude);
	PX4_INFO("Target altitude : %d", iridiumMessage.msg.target_altitude);
	PX4_INFO("Target distance : %d", iridiumMessage.msg.target_distance);
	PX4_INFO("Waypoint number : %d", iridiumMessage.msg.wp_num);
	PX4_INFO("Failure flags : %d", iridiumMessage.msg.failure_flags);
	PX4_INFO("MAV type : %d", iridiumMessage.msg.type);
	PX4_INFO("Autopilot : %d", iridiumMessage.msg.autopilot);
	PX4_INFO("Heading : %d", iridiumMessage.msg.heading);
	PX4_INFO("Target heading : %d", iridiumMessage.msg.target_heading);
	PX4_INFO("Throttle : %d", iridiumMessage.msg.throttle);
	PX4_INFO("Airspeed : %d", iridiumMessage.msg.airspeed);
	PX4_INFO("Airspeed setpoint : %d", iridiumMessage.msg.airspeed_sp);
	PX4_INFO("Groundspeed : %d", iridiumMessage.msg.groundspeed);
	PX4_INFO("Windspeed : %d", iridiumMessage.msg.windspeed);
	PX4_INFO("Wind heading : %d", iridiumMessage.msg.wind_heading);
	PX4_INFO("EPH : %d", iridiumMessage.msg.eph);
	PX4_INFO("EPV : %d", iridiumMessage.msg.epv);
	PX4_INFO("Air temperature : %d", iridiumMessage.msg.temperature_air);
	PX4_INFO("Climb rate : %d", iridiumMessage.msg.climb_rate);
	PX4_INFO("Battery remaining : %d", iridiumMessage.msg.battery);
	PX4_INFO("Custom 0 : %d", iridiumMessage.msg.custom0);
	PX4_INFO("Custom 1 : %d", iridiumMessage.msg.custom1);
	PX4_INFO("Custom 2 : %d", iridiumMessage.msg.custom2);
}

bool MavlinkIridium::getHighLatencyStatus()
{
	return highLatencyActive;
}

char* MavlinkIridium::getMessage()
{
	return iridiumMessage.buf;
}
