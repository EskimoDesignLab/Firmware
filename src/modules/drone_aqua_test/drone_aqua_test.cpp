/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
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
 * @file drone_aqua_test.cpp
 * Implementation of a perching wall controller.
 *
 * @author Thomas Courteau 	<thomas.robichaud.courteau@gmail.com>
 *
 */

#include <drivers/drv_pwm_output.h>

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <time.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_accel.h>
#include <arch/board/board.h>
#include <uORB/uORB.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
//#include <uORB/topics/fw_virtual_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
//#include <uORB/topics/fw_virtual_rates_setpoint.h>
//#include <uORB/topics/mc_virtual_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/distance_sensor.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/pid/pid.h>
#include <geo/geo.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <mathlib/mathlib.h>

#include <ecl/attitude_fw/ecl_pitch_controller.h>
#include <ecl/attitude_fw/ecl_roll_controller.h>
#include <ecl/attitude_fw/ecl_yaw_controller.h>
#include <ecl/attitude_fw/ecl_wheel_controller.h>
#include <platforms/px4_defines.h>

// on inclu le nouveau UORB topic spécifique pour le driver charging i2c
#include <uORB/topics/charging_info.h>
#include <uORB/topics/charging_info_2.h>
#include <uORB/topics/wake_up_slave_info_2.h>
#include <uORB/topics/wake_up_slave_info.h>

//#include "control_wall_landing.h"

#include <vector>

#define SENSOR_COUNT_MAX		3
#define DT 0.005f
#define R2D 57.2957795f
#define D2R  0.0174533f

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int drone_aqua_test_main(int argc, char *argv[]);

//********************************************//
// DECLARATION DE LA CLASSE DU PERCHING WALL////
//********************************************//
class DroneAquaTest
{
public:
	/**
	 * Constructor
	 */
	DroneAquaTest();

	/**
	 * Destructor, also kills the main task.
	 */
	~DroneAquaTest();

	/**
	 * Start the main task.
	 *
	 * @return	OK on success.
	 */
	int		start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:

	// ID POUR LA COMMANDE DE LA TETE DU FLYING WING
#define INDEX_SERVO_ROT 1
#define INDEX_WIRE_POS_UP 2
#define INDEX_WIRE_POS_DOWN 3


	bool		_task_should_exit;		/**< if true, attitude control task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */

	bool		_wall_landing;
	bool		_threshold_accel;
	bool        _landed;
	bool        _take_off;          /**< Take off from the wall */
	bool        _accel_stop;        /**< if true, stop hover control */
	bool        _climb;             /**< climb mode, fly into wall */
	bool        _climb_take_off;    /**< climb mode, fly backwards */

	bool _flag_qd_calculated;
	math::Quaternion _qAtt2Des;
	math::Quaternion _qd_offset;
	math::Quaternion _qDes;
	math::Quaternion _qAtt;
	math::Vector<3> _EulAtt2Des;
	math::Vector<3> _euler_error_old;
	math::Vector<3> _derivate;
	math::Vector<3> _integrate;
	math::Vector<3> _integrate_old;
	std::vector<float> _outputs;
	float _dt;
	float _yaw_sp;
	float _hover_speed_comm;
	float _speed_z_axis;
	float _speed_z_axis_old;
	float _speed_error;
	float _speed_error_old;
	float _speed_derivate;
	float _speed_integrate;
	float _speed_integrate_old;


	struct {
		std::vector<float> Kp;
		std::vector<float> Kd;
		std::vector<float> Ki;
	} _gain;


	int		_control_task;			/**< task handle */

	int		_ctrl_state_sub;	/**< control state subscription */
	int		_accel_sub;			/**< accelerometer subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_attitude_sub;			/**< raw rc channels data subscription */
	int		_vcontrol_mode_sub;		/**< vehicle status subscription */
	int 		_params_sub;			/**< notification of parameter updates */
	int 		_manual_sub;			/**< notification of manual control updates */
	int		_global_pos_sub;		/**< global position subscription */
	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int 		_charging_info_sub; /* pour le module de balancing i2c */
	int 		_wake_up_slave_info_sub;

	int _counter;

	orb_advert_t	_rate_sp_pub;			/**< rate setpoint publication */
	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint point */
	orb_advert_t	_actuators_0_pub;		/**< actuator control group 0 setpoint */
	orb_advert_t	_actuators_2_pub;		/**< actuator control group 1 setpoint (Airframe) */

	orb_advert_t     _charging_info_topic_2;          /* pour publish charging i2c */
	orb_advert_t     _wake_up_slave_topic_2;

	orb_id_t _rates_sp_id;	// pointer to correct rates setpoint uORB metadata structure
	orb_id_t _actuators_id;	// pointer to correct actuator controls0 uORB metadata structure


	orb_id_t _attitude_setpoint_id;

	struct charging_info_s				_charging;	/* charging i2c topic */
	struct wake_up_slave_info_s			_wake_up_slave;

	struct control_state_s				_ctrl_state;	/**< control state */
	struct accel_report				_accel;			/**< body frame accelerations */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct vehicle_rates_setpoint_s			_rates_sp;	/* attitude rates setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s			_vcontrol_mode;		/**< vehicle control mode */
	struct actuator_controls_s			_actuators;		/**< actuator control inputs */
	struct actuator_controls_s			_actuators_airframe;	/**< actuator control inputs */
	struct vehicle_global_position_s		_global_pos;		/**< global position */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
	struct vehicle_land_detected_s			_vehicle_land_detected;	/**< vehicle land detected */

	struct charging_info_2_s 				report_charging_2;
	struct wake_up_slave_info_2_s 				report_wake_up_slave_2;

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_nonfinite_input_perf;		/**< performance counter for non finite input */
	perf_counter_t	_nonfinite_output_perf;		/**< performance counter for non finite output */

	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */
	bool		_debug;				/**< if set to true, print debug output */

	float _flaps_cmd_last;
	float _flaperons_cmd_last;

	float _time;
	float _time_yaw_sp;
	float _time_climb;
	float _g;


	struct {
		float p_tc;
		float p_p;
		float p_i;
		float p_ff;
		float p_rmax_pos;
		float p_rmax_neg;
		float p_integrator_max;
		float r_tc;
		float r_p;
		float r_i;
		float r_ff;
		float r_integrator_max;
		float r_rmax;
		float y_p;
		float y_i;
		float y_d;
		float y_ff;
		float y_integrator_max;
		float y_coordinated_min_speed;
		int32_t y_coordinated_method;
		float y_rmax;
		float w_p;
		float w_i;
		float w_ff;
		float w_integrator_max;
		float w_rmax;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;

		float trim_roll;
		float trim_pitch;
		float trim_yaw;
		float rollsp_offset_deg;		/**< Roll Setpoint Offset in deg */
		float pitchsp_offset_deg;		/**< Pitch Setpoint Offset in deg */
		float rollsp_offset_rad;		/**< Roll Setpoint Offset in rad */
		float pitchsp_offset_rad;		/**< Pitch Setpoint Offset in rad */
		float man_roll_max;				/**< Max Roll in rad */
		float man_pitch_max;			/**< Max Pitch in rad */

		float flaps_scale;				/**< Scale factor for flaps */
		float flaperon_scale;			/**< Scale factor for flaperons */

		int vtol_type;					/**< VTOL type: 0 = tailsitter, 1 = tiltrotor */

		// timing parameter for takeoff custom
		float take_off_custom_time_01;
		//float take_off_custom_time_02;
		//float take_off_custom_time_03;
		//float take_off_custom_time_04;
		//float take_off_custom_time_05;
		//float take_off_custom_time_06;
		//float take_off_custom_time_07;
		float take_off_custom_time_08;
		float take_off_custom_time_09;
		float take_off_custom_time_10;
		float take_off_custom_time_11;
		float take_off_horizontal_pos;
		float take_off_up_pos;
		float take_off_down_pos;
		float take_off_pitch_kp;
		float take_off_pitch_kd;
		float take_off_yaw_kp;
		float take_off_yaw_kd;
        float take_off_rudder_offset;
        float take_off_custom_pitch;
		float take_off_nose_kp;
		float take_off_nose_kd;

	}		_parameters;			/**< local copies of interesting parameters */

	struct {

		param_t p_tc;
		param_t p_p;
		param_t p_i;
		param_t p_ff;
		param_t p_rmax_pos;
		param_t p_rmax_neg;
		param_t p_integrator_max;
		param_t r_tc;
		param_t r_p;
		param_t r_i;
		param_t r_ff;
		param_t r_integrator_max;
		param_t r_rmax;
		param_t y_p;
		param_t y_i;
		param_t y_d;
		param_t y_ff;
		param_t y_integrator_max;
		param_t y_coordinated_min_speed;
		param_t y_coordinated_method;
		param_t y_rmax;
		param_t w_p;
		param_t w_i;
		param_t w_ff;
		param_t w_integrator_max;
		param_t w_rmax;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;

		param_t trim_roll;
		param_t trim_pitch;
		param_t trim_yaw;
		param_t rollsp_offset_deg;
		param_t pitchsp_offset_deg;
		param_t man_roll_max;
		param_t man_pitch_max;

		param_t flaps_scale;
		param_t flaperon_scale;

		param_t vtol_type;

		// timing parameter for takeoff custom
		param_t take_off_custom_time_01;
		//param_t take_off_custom_time_02;
		//param_t take_off_custom_time_03;
		//param_t take_off_custom_time_04;
		//param_t take_off_custom_time_05;
		//param_t take_off_custom_time_06;
		//param_t take_off_custom_time_07;
		param_t take_off_custom_time_08;
		param_t take_off_custom_time_09;
		param_t take_off_custom_time_10;
		param_t take_off_custom_time_11;
		param_t take_off_horizontal_pos;
		param_t take_off_up_pos;
		param_t take_off_down_pos;
		param_t take_off_pitch_kp;
		param_t take_off_pitch_kd;
		param_t take_off_yaw_kp;
		param_t take_off_yaw_kd;
        param_t take_off_rudder_offset;
        param_t take_off_custom_pitch;
		param_t take_off_nose_kp;
		param_t take_off_nose_kd;



	}		_parameter_handles;		/**< handles for interesting parameters */

	// Rotation matrix and euler angles to extract from control state
	math::Matrix<3, 3> _R;
	float _roll;
	float _pitch;
	float _yaw;

	ECL_RollController				_roll_ctrl;
	ECL_PitchController				_pitch_ctrl;
	ECL_YawController				_yaw_ctrl;
	ECL_WheelController			_wheel_ctrl;

	//Control_wall_landing _control_wall_landing;


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

	/**
	 * Check for accel updates.
	 */
	void		vehicle_accel_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Check for global position updates.
	 */
	void		global_pos_poll();

	/**
	 * Check for vehicle status updates.
	 */
	void		vehicle_status_poll();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude controller collection task.
	 */
	void		task_main();

};

namespace drone_aqua_test_control
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
	static const int ERROR = -1;

	DroneAquaTest	*g_control = nullptr;
}

//********************************************//
// CONSTRUCTOR DE LA CLASSE                 ////
//********************************************//
DroneAquaTest::DroneAquaTest() :

		_task_should_exit(false),
		_task_running(false),
		_wall_landing(false),
		_threshold_accel(false),
		_landed(false),
		_take_off(false),
		_accel_stop(false),
		_climb(false),
		_climb_take_off(false),

		_flag_qd_calculated(false),
		_EulAtt2Des(0),
		_euler_error_old(0),
		_derivate(0),
		_integrate(0),
		_integrate_old(0),
		_outputs(4),
		_dt(0.005),
		_yaw_sp(0.0f),
		_hover_speed_comm(0.0f),
		_speed_z_axis(0.0f),
		_speed_z_axis_old(0.0f),
		_speed_error(0.0f),
		_speed_error_old(0.0f),
		_speed_derivate(0.0f),
		_speed_integrate(0.0f),
		_speed_integrate_old(0.0f),

		_control_task(-1),

		/* subscriptions */
		_ctrl_state_sub(-1),
		_accel_sub(-1),
		_vcontrol_mode_sub(-1),
		_params_sub(-1),
		_manual_sub(-1),
		_global_pos_sub(-1),
		_vehicle_status_sub(-1),
		_vehicle_land_detected_sub(-1),
		//_distance_sub(-1),
		_charging_info_sub(-1),
		_wake_up_slave_info_sub(-1),

		/* publications */
		_rate_sp_pub(nullptr),
		_attitude_sp_pub(nullptr),
		_actuators_0_pub(nullptr),
		_actuators_2_pub(nullptr),

		_charging_info_topic_2(nullptr),
		_wake_up_slave_topic_2(nullptr),

		_rates_sp_id(0),
		_actuators_id(0),
		_attitude_setpoint_id(0),

		/* performance counters */
		_loop_perf(perf_alloc(PC_ELAPSED, "fwa_dt")),

		_nonfinite_input_perf(nullptr),
		_nonfinite_output_perf(nullptr),
		/* states */
		_setpoint_valid(false),
		_debug(false),
		_flaps_cmd_last(0),
		_flaperons_cmd_last(0),
		_time(0),
		_time_yaw_sp(0),
		_time_climb(0),
		_g(9.81f)
{
	/* safely initialize structs */
	_ctrl_state = {};
	_accel = {};
	_att_sp = {};
	_rates_sp = {};
	_manual = {};
	_vcontrol_mode = {};
	_actuators = {};
	_actuators_airframe = {};
	_global_pos = {};
	_vehicle_status = {};
	_vehicle_land_detected = {};
	//_distance ={};

	_counter = 0;


	_parameter_handles.p_tc = param_find("FW_P_TC");
	_parameter_handles.p_p = param_find("FW_PR_P");
	_parameter_handles.p_i = param_find("FW_PR_I");
	_parameter_handles.p_ff = param_find("FW_PR_FF");
	_parameter_handles.p_rmax_pos = param_find("FW_P_RMAX_POS");
	_parameter_handles.p_rmax_neg = param_find("FW_P_RMAX_NEG");
	_parameter_handles.p_integrator_max = param_find("FW_PR_IMAX");

	_parameter_handles.r_tc = param_find("FW_R_TC");
	_parameter_handles.r_p = param_find("FW_RR_P");
	_parameter_handles.r_i = param_find("FW_RR_I");
	_parameter_handles.r_ff = param_find("FW_RR_FF");
	_parameter_handles.r_integrator_max = param_find("FW_RR_IMAX");
	_parameter_handles.r_rmax = param_find("FW_R_RMAX");

	_parameter_handles.y_p = param_find("FW_YR_P");
	_parameter_handles.y_i = param_find("FW_YR_I");
	_parameter_handles.y_ff = param_find("FW_YR_FF");
	_parameter_handles.y_integrator_max = param_find("FW_YR_IMAX");
	_parameter_handles.y_rmax = param_find("FW_Y_RMAX");

	_parameter_handles.w_p = param_find("FW_WR_P");
	_parameter_handles.w_i = param_find("FW_WR_I");
	_parameter_handles.w_ff = param_find("FW_WR_FF");
	_parameter_handles.w_integrator_max = param_find("FW_WR_IMAX");
	_parameter_handles.w_rmax = param_find("FW_W_RMAX");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");

	_parameter_handles.y_coordinated_min_speed = param_find("FW_YCO_VMIN");
	_parameter_handles.y_coordinated_method = param_find("FW_YCO_METHOD");

	_parameter_handles.trim_roll = param_find("TRIM_ROLL");
	_parameter_handles.trim_pitch = param_find("TRIM_PITCH");
	_parameter_handles.trim_yaw = param_find("TRIM_YAW");
	_parameter_handles.rollsp_offset_deg = param_find("FW_RSP_OFF");
	_parameter_handles.pitchsp_offset_deg = param_find("FW_PSP_OFF");

	_parameter_handles.man_roll_max = param_find("FW_MAN_R_MAX");
	_parameter_handles.man_pitch_max = param_find("FW_MAN_P_MAX");

	_parameter_handles.flaps_scale = param_find("FW_FLAPS_SCL");
	_parameter_handles.flaperon_scale = param_find("FW_FLAPERON_SCL");

	_parameter_handles.vtol_type = param_find("VT_TYPE");

	// timing for each step of aqua drone take off
	_parameter_handles.take_off_custom_time_01 = param_find("TK_WAIT_TIME");
	//_parameter_handles.take_off_custom_time_02 = param_find("TK_CUSTM_T2");
	//_parameter_handles.take_off_custom_time_03 = param_find("TK_UP_TIME");
	//_parameter_handles.take_off_custom_time_04 = param_find("TK_CUSTM_T4");
	//_parameter_handles.take_off_custom_time_05 = param_find("TK_CUSTM_T5");
	//_parameter_handles.take_off_custom_time_06 = param_find("TK_CUSTM_T6");
	//_parameter_handles.take_off_custom_time_07 = param_find("TK_CUSTM_T7");
	_parameter_handles.take_off_custom_time_08 = param_find("TK_IDLE_UP_TIME");
	_parameter_handles.take_off_custom_time_09 = param_find("TK_FULL_UP_TIME");
	_parameter_handles.take_off_custom_time_10 = param_find("TK_CON_TIME");
	_parameter_handles.take_off_custom_time_11 = param_find("TK_FULL_DN_TIME");
	_parameter_handles.take_off_horizontal_pos = param_find("TK_HOR_POS");
	_parameter_handles.take_off_up_pos = param_find("TK_UP_POS");
	_parameter_handles.take_off_down_pos = param_find("TK_DN_POS");
	_parameter_handles.take_off_pitch_kp = param_find("TK_CON_KP");
	_parameter_handles.take_off_pitch_kd = param_find("TK_CON_KD");
	_parameter_handles.take_off_yaw_kp = param_find("TK_YAW_KP");
	_parameter_handles.take_off_rudder_offset = param_find("TK_RUD_OFF");
    _parameter_handles.take_off_yaw_kd = param_find("TK_YAW_KD");
    _parameter_handles.take_off_custom_pitch = param_find("TK_CUSTM_PITCH");
	_parameter_handles.take_off_nose_kp = param_find("TK_NOSE_KP");
	_parameter_handles.take_off_nose_kd = param_find("TK_NOSE_KD");



	_qd_offset.from_euler(0.0f, 1.57f, 0.0f);
	_qDes.from_euler(0.0f, 0.0f, 0.0f);

	// changed from 4 to 5 (DM)
	_gain.Kp.reserve(5);
	_gain.Kd.reserve(5);
	_gain.Ki.reserve(5);

	/* fetch initial parameter values */
	parameters_update();
}

//********************************************//
// DESTRUCTOR DE LA CLASSE                  ////
//********************************************//
DroneAquaTest::~DroneAquaTest()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	perf_free(_loop_perf);
	perf_free(_nonfinite_input_perf);
	perf_free(_nonfinite_output_perf);

	drone_aqua_test_control::g_control = nullptr;
}

//********************************************//
// PARAMETER UPDATE                         ////
//********************************************//
int
DroneAquaTest::parameters_update()
{

	param_get(_parameter_handles.p_tc, &(_parameters.p_tc));
	param_get(_parameter_handles.p_p, &(_parameters.p_p));
	param_get(_parameter_handles.p_i, &(_parameters.p_i));
	param_get(_parameter_handles.p_ff, &(_parameters.p_ff));
	param_get(_parameter_handles.p_rmax_pos, &(_parameters.p_rmax_pos));
	param_get(_parameter_handles.p_rmax_neg, &(_parameters.p_rmax_neg));
	param_get(_parameter_handles.p_integrator_max, &(_parameters.p_integrator_max));

	param_get(_parameter_handles.r_tc, &(_parameters.r_tc));
	param_get(_parameter_handles.r_p, &(_parameters.r_p));
	param_get(_parameter_handles.r_i, &(_parameters.r_i));
	param_get(_parameter_handles.r_ff, &(_parameters.r_ff));

	param_get(_parameter_handles.r_integrator_max, &(_parameters.r_integrator_max));
	param_get(_parameter_handles.r_rmax, &(_parameters.r_rmax));

	param_get(_parameter_handles.y_p, &(_parameters.y_p));
	param_get(_parameter_handles.y_i, &(_parameters.y_i));
	param_get(_parameter_handles.y_ff, &(_parameters.y_ff));
	param_get(_parameter_handles.y_integrator_max, &(_parameters.y_integrator_max));
	param_get(_parameter_handles.y_coordinated_min_speed, &(_parameters.y_coordinated_min_speed));
	param_get(_parameter_handles.y_coordinated_method, &(_parameters.y_coordinated_method));
	param_get(_parameter_handles.y_rmax, &(_parameters.y_rmax));

	param_get(_parameter_handles.w_p, &(_parameters.w_p));
	param_get(_parameter_handles.w_i, &(_parameters.w_i));
	param_get(_parameter_handles.w_ff, &(_parameters.w_ff));
	param_get(_parameter_handles.w_integrator_max, &(_parameters.w_integrator_max));
	param_get(_parameter_handles.w_rmax, &(_parameters.w_rmax));

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));

	param_get(_parameter_handles.trim_roll, &(_parameters.trim_roll));
	param_get(_parameter_handles.trim_pitch, &(_parameters.trim_pitch));
	param_get(_parameter_handles.trim_yaw, &(_parameters.trim_yaw));
	param_get(_parameter_handles.rollsp_offset_deg, &(_parameters.rollsp_offset_deg));
	param_get(_parameter_handles.pitchsp_offset_deg, &(_parameters.pitchsp_offset_deg));
	_parameters.rollsp_offset_rad = math::radians(_parameters.rollsp_offset_deg);
	_parameters.pitchsp_offset_rad = math::radians(_parameters.pitchsp_offset_deg);
	param_get(_parameter_handles.man_roll_max, &(_parameters.man_roll_max));
	param_get(_parameter_handles.man_pitch_max, &(_parameters.man_pitch_max));
	_parameters.man_roll_max = math::radians(_parameters.man_roll_max);
	_parameters.man_pitch_max = math::radians(_parameters.man_pitch_max);

	param_get(_parameter_handles.flaps_scale, &_parameters.flaps_scale);
	param_get(_parameter_handles.flaperon_scale, &_parameters.flaperon_scale);

	param_get(_parameter_handles.vtol_type, &_parameters.vtol_type);

	/* pitch control parameters */
	_pitch_ctrl.set_time_constant(_parameters.p_tc);
	_pitch_ctrl.set_k_p(_parameters.p_p);
	_pitch_ctrl.set_k_i(_parameters.p_i);
	_pitch_ctrl.set_k_ff(_parameters.p_ff);
	_pitch_ctrl.set_integrator_max(_parameters.p_integrator_max);
	_pitch_ctrl.set_max_rate_pos(math::radians(_parameters.p_rmax_pos));
	_pitch_ctrl.set_max_rate_neg(math::radians(_parameters.p_rmax_neg));

	/* roll control parameters */
	_roll_ctrl.set_time_constant(_parameters.r_tc);
	_roll_ctrl.set_k_p(_parameters.r_p);
	_roll_ctrl.set_k_i(_parameters.r_i);
	_roll_ctrl.set_k_ff(_parameters.r_ff);
	_roll_ctrl.set_integrator_max(_parameters.r_integrator_max);
	_roll_ctrl.set_max_rate(math::radians(_parameters.r_rmax));

	/* yaw control parameters */
	_yaw_ctrl.set_k_p(_parameters.y_p);
	_yaw_ctrl.set_k_i(_parameters.y_i);
	_yaw_ctrl.set_k_ff(_parameters.y_ff);
	_yaw_ctrl.set_integrator_max(_parameters.y_integrator_max);
	_yaw_ctrl.set_coordinated_min_speed(_parameters.y_coordinated_min_speed);
	_yaw_ctrl.set_coordinated_method(_parameters.y_coordinated_method);
	_yaw_ctrl.set_max_rate(math::radians(_parameters.y_rmax));

	/* wheel control parameters */
	_wheel_ctrl.set_k_p(_parameters.w_p);
	_wheel_ctrl.set_k_i(_parameters.w_i);
	_wheel_ctrl.set_k_ff(_parameters.w_ff);
	_wheel_ctrl.set_integrator_max(_parameters.w_integrator_max);
	_wheel_ctrl.set_max_rate(math::radians(_parameters.w_rmax));

	// timing for each step of aqua drone take off
	param_get(_parameter_handles.take_off_custom_time_01, &_parameters.take_off_custom_time_01);
	//param_get(_parameter_handles.take_off_custom_time_02, &_parameters.take_off_custom_time_02);
	//param_get(_parameter_handles.take_off_custom_time_03, &_parameters.take_off_custom_time_03);
	//param_get(_parameter_handles.take_off_custom_time_04, &_parameters.take_off_custom_time_04);
	//param_get(_parameter_handles.take_off_custom_time_05, &_parameters.take_off_custom_time_05);
	//param_get(_parameter_handles.take_off_custom_time_06, &_parameters.take_off_custom_time_06);
	//param_get(_parameter_handles.take_off_custom_time_07, &_parameters.take_off_custom_time_07);
	param_get(_parameter_handles.take_off_custom_time_08, &_parameters.take_off_custom_time_08);
	param_get(_parameter_handles.take_off_custom_time_09, &_parameters.take_off_custom_time_09);
	param_get(_parameter_handles.take_off_custom_time_10, &_parameters.take_off_custom_time_10);
	param_get(_parameter_handles.take_off_custom_time_11, &_parameters.take_off_custom_time_11);
	param_get(_parameter_handles.take_off_horizontal_pos, &_parameters.take_off_horizontal_pos);
	param_get(_parameter_handles.take_off_up_pos, &_parameters.take_off_up_pos);
	param_get(_parameter_handles.take_off_down_pos, &_parameters.take_off_down_pos);
	param_get(_parameter_handles.take_off_pitch_kp, &_parameters.take_off_pitch_kp);
	param_get(_parameter_handles.take_off_pitch_kd, &_parameters.take_off_pitch_kd);
	param_get(_parameter_handles.take_off_yaw_kp, &_parameters.take_off_yaw_kp);
	param_get(_parameter_handles.take_off_yaw_kd, &_parameters.take_off_yaw_kd);
    param_get(_parameter_handles.take_off_rudder_offset, &_parameters.take_off_rudder_offset);
    param_get(_parameter_handles.take_off_custom_pitch, &_parameters.take_off_custom_pitch);
	param_get(_parameter_handles.take_off_nose_kp, &_parameters.take_off_nose_kp);
	param_get(_parameter_handles.take_off_nose_kd, &_parameters.take_off_nose_kd);
	return OK;
}
void
DroneAquaTest::vehicle_control_mode_poll()
{
	bool vcontrol_mode_updated;

	/* Check if vehicle control mode has changed */
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
DroneAquaTest::vehicle_manual_poll()
{
	bool manual_updated;

	/* get pilots inputs */
	orb_check(_manual_sub, &manual_updated);

	if (manual_updated) {

		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void
DroneAquaTest::vehicle_accel_poll()
{
	/* check if there is a new position */
	bool accel_updated;
	orb_check(_accel_sub, &accel_updated);

	if (accel_updated) {
		orb_copy(ORB_ID(sensor_accel), _accel_sub, &_accel);
	}
}

void
DroneAquaTest::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool att_sp_updated;
	orb_check(_att_sp_sub, &att_sp_updated);

	if (att_sp_updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
		_setpoint_valid = true;
	}
}

void
DroneAquaTest::global_pos_poll()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

void
DroneAquaTest::vehicle_status_poll()
{
	/* check if there is new status information */
	bool vehicle_status_updated;
	orb_check(_vehicle_status_sub, &vehicle_status_updated);

	if (vehicle_status_updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_rates_sp_id) {
			if (_vehicle_status.is_vtol) {
				_rates_sp_id = ORB_ID(fw_virtual_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_virtual_fw);
				_attitude_setpoint_id = ORB_ID(fw_virtual_attitude_setpoint);

			} else {
				_rates_sp_id = ORB_ID(vehicle_rates_setpoint);
				_actuators_id = ORB_ID(actuator_controls_0);

				// TEST CONTROL ACTUATEUR GROUP 2
				//_actuators2_id = ORB_ID(actuator_controls_1);

				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}
}

void
DroneAquaTest::task_main_trampoline(int argc, char *argv[])
{
	drone_aqua_test_control::g_control->task_main();
}
void
DroneAquaTest::task_main() {

	_wake_up_slave_topic_2 = orb_advertise(ORB_ID(wake_up_slave_info_2), &report_wake_up_slave_2);
	_charging_info_topic_2 = orb_advertise(ORB_ID(charging_info_2), &report_charging_2);

	/*
     * do subscriptions ()
     */
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_accel_sub = orb_subscribe_multi(ORB_ID(sensor_accel), 0);
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));

	_wake_up_slave_info_sub = orb_subscribe(ORB_ID(wake_up_slave_info));
	_charging_info_sub = orb_subscribe(ORB_ID(charging_info));

	// Set l'intervale à 200Hz ( boucle de controle )
	orb_set_interval(_ctrl_state_sub, 5);

	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_setpoint_poll();
	vehicle_accel_poll();
	vehicle_control_mode_poll();
	vehicle_manual_poll();
	vehicle_status_poll();

	/* wakeup source */
	px4_pollfd_struct_t fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _ctrl_state_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	static bool mode_seq0 = false;
	static bool mode_seq2 = false;
	static bool mode_seq7 = false;
	static bool mode_seq8 = false;
	static bool mode_seq9 = false;
	static bool mode_seq10 = false;
	static bool mode_seq11 = false;
	float err0 = 0.0;

	static bool flagidle = false;

    static int present_time = 0;
    static int _countPrint =0;
    while (!_task_should_exit) {

		static int loop_counter = 0;
		//static int time_count = 0;

		// VARIABLES UTILES À LA MANOEUVRE DE DÉCOLLAGE

		/*****************************************************/

		//static int compteur = 0;

		//static bool mode_seq0 = false;
		//static bool mode_seq2 = false;
		//static bool mode_seq7 = false;
		//static bool mode_seq8 = false;
		//static bool mode_seq9 = false;
		//static bool mode_seq10 = false;
		//static bool mode_seq11 = false;
		//float err0 = 0.0;

		//static bool flagidle = false;

		//static int present_time = 0;

		/*****************************************************/

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		perf_begin(_loop_perf);

		// UPDATE DES PARAMETRES SI EVENT DU 200 HZ
		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		// LORSQUE LUPDATE DES PARAMETRE SURVIENT -> ON VA DANS LE IF
		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {
			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();


			/* guard against too large deltaT's */
			if (deltaT > 1.0f) {
				deltaT = 0.01f;
			}

			// uORB des control state (acc, gyro, compass, etc. qui viennent dun autre process)
			orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

			/* get current rotation matrix and euler angles from control state quaternions */
			math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
			_R = q_att.to_dcm();

			math::Vector<3> euler_angles;
			euler_angles = _R.to_euler();
			_roll = euler_angles(0);
			_pitch = euler_angles(1);
			_yaw = euler_angles(2);

			vehicle_status_poll();
			vehicle_accel_poll();
			vehicle_manual_poll();
			vehicle_control_mode_poll();


			//****************************************************************************************************//
			//A FARE -> METTRE CETTE SÉQUENCE DANS UNE FONCTION APPROPRIÉE ET RENOMMER LE MODULE "AQUA_TAKE_OFF"
			//****************************************************************************************************//

			// CES COMMANDES DE LAVION VIENNENT DE LA MANETTE (MODE SEMI MANUEL POUR LINSTANT)
			_actuators.control[actuator_controls_s::INDEX_ROLL] = _manual.y; //0.0f + _parameters.trim_roll;
			_actuators.control[actuator_controls_s::INDEX_PITCH] = _manual.x; //0.3f + _parameters.trim_pitch;
			_actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r; // 0.0f + _parameters.trim_yaw
			_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z; // pass-through de la commande du trigger

            float r2servo = (_parameters.take_off_up_pos - _parameters.take_off_horizontal_pos) / (3.14159f / 2);


            // WAIT AVANT LA SEQUENCE (FALCULTATIF)
            if (mode_seq0) {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
                _actuators_airframe.control[1] = _parameters.take_off_horizontal_pos; //0.28f;
                _actuators_airframe.control[2] = _parameters.take_off_rudder_offset;

				if (hrt_absolute_time() - present_time >=
					(int) _parameters.take_off_custom_time_01) // 2 sec
				{
					present_time = hrt_absolute_time();
					mode_seq0 = false;
					mode_seq2 = true;
					//flagidle = true;
				}
			}

            // ACTIVE LE SERVO POUR REMONTER LE PIVOT
            if (mode_seq2) {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
                _actuators_airframe.control[1] = _parameters.take_off_up_pos;
                _actuators_airframe.control[2] = _parameters.take_off_rudder_offset;
                if (hrt_absolute_time() - present_time >=
                    1000000) //(int)_parameters.take_off_custom_time_03) // 1 sec
                {
                    present_time = hrt_absolute_time();
                    mode_seq2 = false;
                    mode_seq7 = true;
                }
            }

            // GAB : IDLE DU THRUST A 30% PENDANT UN CERTAIN TEMPS ==> Etienne Edit : Debut du full throttle et du controleur
            if (mode_seq7) {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 1.0f;
				_qDes.from_euler(0.0f, _parameters.take_off_custom_pitch, 0.0f);
				_qAtt = q_att;
				_qAtt2Des = _qAtt.conjugated() * _qDes;
				_EulAtt2Des = _qAtt2Des.to_euler();
//                Boucle pour le print et l'incrementation de l'indice compteur
                if (++_countPrint >= 200)
                {
                    warn("Error real : %0.3f , %0.3f , %0.3f", (double)(_EulAtt2Des(0)*R2D), (double)(_EulAtt2Des(1)*R2D), (double)(_EulAtt2Des(2)*R2D));
                    _countPrint = 0;
                }

				_actuators_airframe.control[1] = (_parameters.take_off_pitch_kp*_EulAtt2Des(1) - _parameters.take_off_pitch_kd*_ctrl_state.pitch_rate) * r2servo + _parameters.take_off_horizontal_pos;
				_actuators_airframe.control[2] = (_parameters.take_off_yaw_kp*_EulAtt2Des(2) - _parameters.take_off_yaw_kd*_ctrl_state.yaw_rate)+_parameters.take_off_rudder_offset;


				if (hrt_absolute_time() - present_time >=
					(int) _parameters.take_off_custom_time_08) // 2 sec
				{
					present_time = hrt_absolute_time();
					mode_seq7 = false;
					mode_seq8 = true;
				}
			}

			// FULL THROTTLE PENDANT UN CERTAIN TEMPS
			if (mode_seq8) {

                float _elevDes = 20.0f*D2R;
                // Present attitude from Quaternion to Euler ZXY
                float _headingNow = atan2f(-2.0f * (_ctrl_state.q[1] * _ctrl_state.q[2] - _ctrl_state.q[0] * _ctrl_state.q[3]), 1.0f - 2.0f * (_ctrl_state.q[1] * _ctrl_state.q[1] + _ctrl_state.q[3] * _ctrl_state.q[3]));
                float _bankNow = asinf(2.0f * (_ctrl_state.q[2] * _ctrl_state.q[3] + _ctrl_state.q[0] * _ctrl_state.q[1]));
                // Quaternion with the right heading and elevation from nose down movement of present attitude - From Euler Rotation ZXY to Quaternion
                float cang[3] = {cosf(_headingNow/2.0f) , cosf(_bankNow/2.0f) , cosf(_elevDes/cosf(_bankNow)/2.0f)};
                float sang[3] = {sinf(_headingNow/2.0f) , sinf(_bankNow/2.0f) , sinf(_elevDes/cosf(_bankNow)/2.0f)};
                math::Quaternion _qElev;
                _qElev(0) = cang[0]*cang[1]*cang[2]-sang[0]*sang[1]*sang[2];
                _qElev(1) = cang[0]*sang[1]*cang[2]-sang[0]*cang[1]*sang[2];
                _qElev(2) = cang[0]*cang[1]*sang[2]+sang[0]*sang[1]*cang[2];
                _qElev(3) = cang[0]*sang[1]*sang[2]+sang[0]*cang[1]*cang[2];
                math::Vector<3> _eElev = _qElev.to_euler();
                // Quaternion desired from forcing Bank=0 to Quaternion with the right heading and elevation
                _qDes.from_euler(0.0f, _eElev(1), _eElev(2));
                _qAtt2Des = q_att.conjugated() * _qDes;
                // Euler angle error from Quaternion error - Rotation YXZ to exclude yaw movement as required by the error calculation and allow pitch movement >90°
                float _pitchErr = atan2f(2.0f * (_qAtt2Des(1) * _qAtt2Des(3) + _qAtt2Des(0) * _qAtt2Des(2)), 1.0f - 2.0f * (_qAtt2Des(1) * _qAtt2Des(1) + _qAtt2Des(2) * _qAtt2Des(2)));
                float  _rollErr = asinf(-2.0f * (_qAtt2Des(2) * _qAtt2Des(3) - _qAtt2Des(0) * _qAtt2Des(1)));
                float   _yawErr = atan2f(2.0f * (_qAtt2Des(1) * _qAtt2Des(2) + _qAtt2Des(0) * _qAtt2Des(3)), 1.0f - 2.0f * (_qAtt2Des(1) * _qAtt2Des(1) + _qAtt2Des(3) * _qAtt2Des(3)));

                //Boucle pour le print et l'incrementation de l'indice compteur
                if (++_countPrint >= 200)
                {
                    warn("Error Calc YXZ : %0.3f , %0.3f , %0.3f", (double)(_pitchErr*R2D), (double)(_rollErr*R2D), (double)(_yawErr*R2D));
                    _countPrint = 0;
                }

                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 1.0f;
                _actuators_airframe.control[1] = (_pitchErr*_parameters.take_off_nose_kp - _ctrl_state.pitch_rate*_parameters.take_off_nose_kd) * r2servo + _parameters.take_off_horizontal_pos;
                _actuators_airframe.control[2] = _parameters.take_off_rudder_offset;
                _actuators.control[actuator_controls_s::INDEX_ROLL] =  _parameters.trim_roll;
                _actuators.control[actuator_controls_s::INDEX_PITCH] = _parameters.trim_pitch;
				if (hrt_absolute_time() - present_time >=
					(int) _parameters.take_off_custom_time_09) // 120 ms
				{
					present_time = hrt_absolute_time();
					mode_seq8 = false;
					mode_seq9 = true;
				}
			}

            // MET LA TETE DU PIVOT À LHORIZONTAL ET GARDE FULL THROTTLE
            if (mode_seq9) {
                float err = _parameters.take_off_custom_pitch - _pitch;
                float derr = (err - err0) / DT;
                err0 = err;
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 1.0f;
                _actuators_airframe.control[1] =
                        (_parameters.take_off_pitch_kp * err + _parameters.take_off_pitch_kd * derr) * r2servo +
                        _parameters.take_off_horizontal_pos;
                _actuators_airframe.control[1] = _parameters.take_off_down_pos;


				if (hrt_absolute_time() - present_time >=
					(int) _parameters.take_off_custom_time_10) // Étienne 1000sec pour débugger
				{
					present_time = hrt_absolute_time();
					mode_seq9 = false;
					mode_seq10 = true;
				}
			}

			//MAINTIENT FULL THROTTLE POUR UN CERTAIN TEMPS
			if (mode_seq10) {
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = 1.0f;
				_actuators_airframe.control[1] = _parameters.take_off_horizontal_pos; //0.28f;

				//if(hrt_absolute_time() - present_time >= 2000000)//(int)_parameters.take_off_custom_time_11) // 2 sec
				if (hrt_absolute_time() - present_time >=
					(int) _parameters.take_off_custom_time_11) // 2 sec
				{
					present_time = hrt_absolute_time();
					mode_seq10 = false;
					mode_seq11 = true;
				}
			}

			if (mode_seq11) {
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z; // pass-through de la commande du trigger
				_actuators_airframe.control[1] = _parameters.take_off_horizontal_pos; //0.28f;
			}

			// COMMANDE DE ZÉRO SI VÉHICULE PAS ARMÉ
			if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {

				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _actuators.control[actuator_controls_s::INDEX_THROTTLE];


				if (flagidle == false) {
					flagidle = true;
					present_time = hrt_absolute_time();
					mode_seq0 = true;
				}

            } else {
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f; // quant nuttx boot le thrust est a 0
                _actuators_airframe.control[1] = _parameters.take_off_horizontal_pos; //0.28f; // le servo ne bouge pas
                _actuators_airframe.control[2] = _parameters.take_off_rudder_offset;

				mode_seq0 = false;
				mode_seq2 = false;
				mode_seq7 = false;
				mode_seq8 = false;
				mode_seq9 = false;
				mode_seq10 = false;
				mode_seq11 = false;

				flagidle = false;
			}


			// SÉCURITÉ -> MODE MANUEL ENCLENCHÉ
			if (_manual.kill_switch == manual_control_setpoint_s::SWITCH_POS_ON) // si en mode manuel
			{
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
				_actuators_airframe.control[1] = _parameters.take_off_horizontal_pos; //0.28f; // le servo ne bouge pas

				mode_seq0 = false;
				mode_seq2 = false;
				mode_seq7 = false;
				mode_seq8 = false;
				mode_seq9 = false;
				mode_seq10 = false;
				mode_seq11 = false;
			}

			/* lazily publish the setpoint only once available */
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _ctrl_state.timestamp;
			_actuators_airframe.timestamp = hrt_absolute_time();
			_actuators_airframe.timestamp_sample = _ctrl_state.timestamp;

			/* publish the actuator controls */
			if (_actuators_0_pub != nullptr) {
				orb_publish(_actuators_id, _actuators_0_pub, &_actuators);

			} else if (_actuators_id) {
				_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
			}

			if (_actuators_2_pub != nullptr) {
				/* publish the actuator controls*/
				orb_publish(ORB_ID(actuator_controls_2), _actuators_2_pub, &_actuators_airframe);

			} else {
				/* advertise and publish */
				_actuators_2_pub = orb_advertise(ORB_ID(actuator_controls_2), &_actuators_airframe);
			}
		}

		loop_counter++;
		perf_end(_loop_perf);
	}

	warnx("exiting.\n");

	_control_task = -1;
	_task_running = false;
	_time = 0;
	_counter = 0;
}

int
DroneAquaTest::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("pivot test",
									   SCHED_DEFAULT,
									   SCHED_PRIORITY_MAX - 5,
									   1300,
									   (px4_main_t)&DroneAquaTest::task_main_trampoline,
									   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int drone_aqua_test_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: drone_aqua_test {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (drone_aqua_test_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		drone_aqua_test_control::g_control = new DroneAquaTest;

		if (drone_aqua_test_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != drone_aqua_test_control::g_control->start()) {
			delete drone_aqua_test_control::g_control;
			drone_aqua_test_control::g_control = nullptr;
			warn("start failed");
			return 1;
		}

		/* check if the waiting is necessary at all */
		if (drone_aqua_test_control::g_control == nullptr || !drone_aqua_test_control::g_control->task_running()) {

			/* avoid memory fragmentation by not exiting start handler until the task has fully started */
			while (drone_aqua_test_control::g_control == nullptr || !drone_aqua_test_control::g_control->task_running()) {
				usleep(50000);
				printf(".");
				fflush(stdout);
			}

			printf("\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (drone_aqua_test_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete drone_aqua_test_control::g_control;
		drone_aqua_test_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (drone_aqua_test_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
