/****************************************************************************
 *
 *   Copyright (c) 2013-2017 PX4 Development Team. All rights reserved.
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
 * @file fw_att_control_main.c
 * Implementation of a generic attitude controller based on classic orthogonal PIDs.
 *
 * @author Lorenz Meier 	<lm@inf.ethz.ch>
 * @author Thomas Gubler 	<thomasgubler@gmail.com>
 * @author Roman Bapst		<bapstr@ethz.ch>
 * @author Thomas Courteau  <thomas.robichaud.courteau@gmail.com>
 * @author Dino Mehanovic   <dino.mehanovic@usherbrooke.ca>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include <drivers/drv_hrt.h>
#include <ecl/attitude_fw/ecl_pitch_controller.h>
#include <ecl/attitude_fw/ecl_roll_controller.h>
#include <ecl/attitude_fw/ecl_wheel_controller.h>
#include <ecl/attitude_fw/ecl_yaw_controller.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include <cmath>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/uORB.h>
#include <vtol_att_control/vtol_type.h>

#include <vector>

using matrix::Eulerf;
using matrix::Quatf;

/**
 * Fixedwing attitude control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int perching_wall_main(int argc, char **argv);

class PerchingWall
{
public:
	/**
	 * Constructor
	 */
	PerchingWall();

	/**
	 * Destructor, also kills the main task.
	 */
	~PerchingWall();

	/**
	 * Start the main task.
	 *
	 * @return	PX4_OK on success.
	 */
	int		start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:

	bool		_task_should_exit;		/**< if true, attitude control task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */
	int		_control_task;			/**< task handle */

    bool		_wall_landing;
    bool		_threshold_accel;
    bool        _landed;
    bool        _take_off;          /**< Take off from the wall */
    bool        _accel_stop;        /**< if true, stop hover control */
    bool        _climb;             /**< climb mode, fly slow down prop and keep pushing toward wall */
    bool        _climb_take_off;    /**< climb mode, fly toward commanded position and up */
    bool        _climb_or_descent;    /**< was climbing in the previous step */
    bool        _climb_hover;        /**< climb mode, maintain medium thrust (T ~ T_hover)> */
    bool        _level;             /**< level flight */
    bool        _recovery;          /**< recovery */
    bool        _mb_landed;         /**< maybe landed, wait for 0.7s and check accZ_n */
    bool        _man_flight;        /**< manual flight */

    bool _flag_qd_calculated;
    math::Quaternion _qe;
    math::Quaternion _qd_offset;
    math::Quaternion _qd;
    math::Quaternion _qm;
    math::Vector<3> _euler_error;
    math::Vector<3> _euler_error_old;
    math::Vector<3> _derivate;
    math::Vector<3> _integrate;
    math::Vector<3> _integrate_old;
    std::vector<float> _outputs;
    float _dt;
    float _yaw_sp;
    float _pitch_sp;
    float _hover_speed_comm;
    // float takeoff_correction;
    //float _angle_correction;
    float _speed_z_axis;
    float _speed_z_axis_old;
    float _speed_error;
    float _speed_error_old;
    float _speed_derivate;
    float _speed_integrate;
    float _speed_integrate_old;
    float accelZ;
    float _dist_comm;

    struct {
        std::vector<float> Kp;
        std::vector<float> Kd;
        std::vector<float> Ki;
    } _gain;

	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_battery_status_sub;		/**< battery status subscription */
	int		_ctrl_state_sub;		/**< control state subscription */
	int		_global_pos_sub;		/**< global position subscription */
	int		_manual_sub;			/**< notification of manual control updates */
	int		_params_sub;			/**< notification of parameter updates */
	int		_vcontrol_mode_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int 	_distance_trone_sub;			/**< distance trone subscription */
    int     _distance_vl53l0x_sub;      /**< distance vl53l0x subscription */

	orb_advert_t	_rate_sp_pub;			/**< rate setpoint publication */
	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint point */
	orb_advert_t	_actuators_0_pub;		/**< actuator control group 0 setpoint */
	orb_advert_t	_actuators_2_pub;		/**< actuator control group 1 setpoint (Airframe) */

	orb_id_t _rates_sp_id;	// pointer to correct rates setpoint uORB metadata structure
	orb_id_t _actuators_id;	// pointer to correct actuator controls0 uORB metadata structure
	orb_id_t _attitude_setpoint_id;

	struct actuator_controls_s			_actuators;		/**< actuator control inputs */
	struct actuator_controls_s			_actuators_airframe;	/**< actuator control inputs */
	struct battery_status_s				_battery_status;	/**< battery status */
	struct control_state_s				_ctrl_state;	/**< control state */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct vehicle_control_mode_s			_vcontrol_mode;		/**< vehicle control mode */
	struct vehicle_global_position_s		_global_pos;		/**< global position */
	struct vehicle_land_detected_s			_vehicle_land_detected;	/**< vehicle land detected */
	struct vehicle_rates_setpoint_s			_rates_sp;	/* attitude rates setpoint */
	struct vehicle_status_s				_vehicle_status;	/**< vehicle status */
    struct distance_sensor_s			_distance_trone, _distance_vl53l0x;

	perf_counter_t	_loop_perf;			/**< loop performance counter */
	perf_counter_t	_nonfinite_input_perf;		/**< performance counter for non finite input */
	perf_counter_t	_nonfinite_output_perf;		/**< performance counter for non finite output */

	bool		_setpoint_valid;		/**< flag if the position control setpoint is valid */
	bool		_debug;				/**< if set to true, print debug output */

	float _flaps_applied;
	float _flaperons_applied;

    float _time;
    float _time_yaw_sp;
    float _time_climb;
    float _g;
    float _time_level;


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
		float y_ff;
		float y_integrator_max;
		float y_coordinated_min_speed;
		float roll_to_yaw_ff;
		int32_t y_coordinated_method;
		float y_rmax;

		bool w_en;
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
		float man_roll_scale;			/**< scale factor applied to roll actuator control in pure manual mode */
		float man_pitch_scale;			/**< scale factor applied to pitch actuator control in pure manual mode */
		float man_yaw_scale; 			/**< scale factor applied to yaw actuator control in pure manual mode */

		float acro_max_x_rate_rad;
		float acro_max_y_rate_rad;
		float acro_max_z_rate_rad;

		float flaps_scale;				/**< Scale factor for flaps */
		float flaperon_scale;			/**< Scale factor for flaperons */

		float rattitude_thres;

		int32_t vtol_type;					/**< VTOL type: 0 = tailsitter, 1 = tiltrotor */

		int32_t bat_scale_en;			/**< Battery scaling enabled */

	}		_parameters{};			/**< local copies of interesting parameters */

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
		param_t y_ff;
		param_t y_integrator_max;
		param_t y_coordinated_min_speed;
		param_t roll_to_yaw_ff;
		param_t y_coordinated_method;
		param_t y_rmax;

		param_t w_en;
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
		param_t man_roll_scale;
		param_t man_pitch_scale;
		param_t man_yaw_scale;

		param_t acro_max_x_rate;
		param_t acro_max_y_rate;
		param_t acro_max_z_rate;

		param_t flaps_scale;
		param_t flaperon_scale;

		param_t rattitude_thres;

		param_t vtol_type;

		param_t bat_scale_en;

        /* Gain for hover control */
        param_t Kp_aileron;
        param_t Kp_elevation;
        param_t Kp_rudder;
        param_t Kp_vel_thrust;
        param_t Kd_aileron;
        param_t Kd_elevation;
        param_t Kd_rudder;
        param_t Kd_vel_thrust;
        param_t Ki_aileron;
        param_t Ki_elevation;
        param_t Ki_rudder;
        param_t Ki_vel_thrust;

	}		_parameter_handles{};		/**< handles for interesting parameters */

	// Rotation matrix and euler angles to extract from control state
	math::Matrix<3, 3> _R;
	float _roll;
	float _pitch;
	float _yaw;

	ECL_RollController				_roll_ctrl;
	ECL_PitchController				_pitch_ctrl;
	ECL_YawController				_yaw_ctrl;
	ECL_WheelController			_wheel_ctrl;


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in vehicle control mode.
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for changes in manual inputs.
	 */
	void		vehicle_manual_poll();

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
	 * Check for vehicle land detected updates.
	 */
	void		vehicle_land_detected_poll();

	/**
	 * Check for battery status updates.
	 */
	void		battery_status_poll();

    /**
    * Check for distance sensor
    */
    void 		distance_poll();


    /**
     *
     * @param q
     */
    void		hover_control(const math::Quaternion &q);



    void calculation_error(const math::Quaternion &val_q);
    void discrete_derivative();
    void discrete_integrate();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main attitude controller collection task.
	 */
	void		task_main();

};

namespace perching_control
{
PerchingWall	*g_control = nullptr;
}

PerchingWall::PerchingWall() :

	_task_should_exit(false),
	_task_running(false),
	_control_task(-1),

    _wall_landing(false),
    _threshold_accel(false),
    _landed(false),
    _take_off(false),
    _accel_stop(false),
    _climb(false),
    _climb_take_off(false),
    _climb_or_descent(false),
    _climb_hover(false),
    _level(false),
    _recovery(false),
    _mb_landed(false),
    _man_flight(true),


    _flag_qd_calculated(false),
    _euler_error(0),
    _euler_error_old(0),
    _derivate(0),
    _integrate(0),
    _integrate_old(0),
    _outputs(4),
    _dt(0.005f),
    _yaw_sp(0.0f),
    _pitch_sp(0.0f),
    _hover_speed_comm(0.0f),
    //takeoff_correction(1.0f),
    _speed_z_axis(0.0f),
    _speed_z_axis_old(0.0f),
    _speed_error(0.0f),
    _speed_error_old(0.0f),
    _speed_derivate(0.0f),
    _speed_integrate(0.0f),
    _speed_integrate_old(0.0f),
    // _angle_correction(1.0f),
    accelZ(0.0f),
    _dist_comm(0.0f),

	/* subscriptions */
	_att_sp_sub(-1),
	_battery_status_sub(-1),
	_ctrl_state_sub(-1),
	_global_pos_sub(-1),
	_manual_sub(-1),
	_params_sub(-1),
	_vcontrol_mode_sub(-1),
	_vehicle_land_detected_sub(-1),
	_vehicle_status_sub(-1),
    _distance_trone_sub(-1),
    _distance_vl53l0x_sub(-1),

	/* publications */
	_rate_sp_pub(nullptr),
	_attitude_sp_pub(nullptr),
	_actuators_0_pub(nullptr),
	_actuators_2_pub(nullptr),

	_rates_sp_id(nullptr),
	_actuators_id(nullptr),
	_attitude_setpoint_id(nullptr),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fwa_dt")),
#if 0
	_nonfinite_input_perf(perf_alloc(PC_COUNT, "fwa_nani")),
	_nonfinite_output_perf(perf_alloc(PC_COUNT, "fwa_nano")),
#else
	_nonfinite_input_perf(nullptr),
	_nonfinite_output_perf(nullptr),
#endif
	/* states */
	_setpoint_valid(false),
	_debug(false),
	_flaps_applied(0),
	_flaperons_applied(0),
	_time(0),
	_time_yaw_sp(0),
	_time_climb(0),
	_g(9.81f),
	_time_level(0),
	_roll(0.0f),
	_pitch(0.0f),
	_yaw(0.0f)
{
	/* safely initialize structs */
	_actuators = {};
	_actuators_airframe = {};
	_att_sp = {};
	_battery_status = {};
	_ctrl_state = {};
	_global_pos = {};
	_manual = {};
	_rates_sp = {};
	_vcontrol_mode = {};
	_vehicle_land_detected = {};
	_vehicle_status = {};
	_distance_trone = {};
    _distance_vl53l0x = {};

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
	_parameter_handles.roll_to_yaw_ff = param_find("FW_RLL_TO_YAW_FF");

	_parameter_handles.w_en = param_find("FW_W_EN");
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
	_parameter_handles.man_roll_scale = param_find("FW_MAN_R_SC");
	_parameter_handles.man_pitch_scale = param_find("FW_MAN_P_SC");
	_parameter_handles.man_yaw_scale = param_find("FW_MAN_Y_SC");

	_parameter_handles.acro_max_x_rate = param_find("FW_ACRO_X_MAX");
	_parameter_handles.acro_max_y_rate = param_find("FW_ACRO_Y_MAX");
	_parameter_handles.acro_max_z_rate = param_find("FW_ACRO_Z_MAX");

	_parameter_handles.flaps_scale = param_find("FW_FLAPS_SCL");
	_parameter_handles.flaperon_scale = param_find("FW_FLAPERON_SCL");

	_parameter_handles.rattitude_thres = param_find("FW_RATT_TH");

	_parameter_handles.vtol_type = param_find("VT_TYPE");

	_parameter_handles.bat_scale_en = param_find("FW_BAT_SCALE_EN");

	_qd_offset.from_euler(0.0f, 1.57f, 0.0f);
	_qd.from_euler(0.0f, 0.0f, 0.0f);

	// changed from 4 to 5 (DM)
	_gain.Kp.reserve(5);
	_gain.Kd.reserve(5);
	_gain.Ki.reserve(5);

	_parameter_handles.Kp_aileron = param_find("KP_AILERON");
	_parameter_handles.Kp_elevation = param_find("KP_ELEVATION");
	_parameter_handles.Kp_rudder = param_find("KP_RUDDER");
	_parameter_handles.Kp_vel_thrust = param_find("KP_VEL_THRUST");
	_parameter_handles.Kd_aileron = param_find("KD_AILERON");
	_parameter_handles.Kd_elevation = param_find("KD_ELEVATION");
	_parameter_handles.Kd_rudder = param_find("KD_RUDDER");
	_parameter_handles.Kd_vel_thrust = param_find("KD_VEL_THRUST");
	_parameter_handles.Ki_aileron = param_find("KI_AILERON");
	_parameter_handles.Ki_elevation = param_find("KI_ELEVATION");
	_parameter_handles.Ki_rudder = param_find("KI_RUDDER");
	_parameter_handles.Ki_vel_thrust = param_find("KI_VEL_THRUST");

	/* fetch initial parameter values */
	parameters_update();
}

PerchingWall::~PerchingWall()
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

	perching_control::g_control = nullptr;
}

int
PerchingWall::parameters_update()
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
	param_get(_parameter_handles.roll_to_yaw_ff, &(_parameters.roll_to_yaw_ff));

	int32_t wheel_enabled = 0;
	param_get(_parameter_handles.w_en, &wheel_enabled);
	_parameters.w_en = (wheel_enabled == 1);

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
	param_get(_parameter_handles.man_roll_scale, &(_parameters.man_roll_scale));
	param_get(_parameter_handles.man_pitch_scale, &(_parameters.man_pitch_scale));
	param_get(_parameter_handles.man_yaw_scale, &(_parameters.man_yaw_scale));

	param_get(_parameter_handles.acro_max_x_rate, &(_parameters.acro_max_x_rate_rad));
	param_get(_parameter_handles.acro_max_y_rate, &(_parameters.acro_max_y_rate_rad));
	param_get(_parameter_handles.acro_max_z_rate, &(_parameters.acro_max_z_rate_rad));
	_parameters.acro_max_x_rate_rad = math::radians(_parameters.acro_max_x_rate_rad);
	_parameters.acro_max_y_rate_rad = math::radians(_parameters.acro_max_y_rate_rad);
	_parameters.acro_max_z_rate_rad = math::radians(_parameters.acro_max_z_rate_rad);

	param_get(_parameter_handles.flaps_scale, &_parameters.flaps_scale);
	param_get(_parameter_handles.flaperon_scale, &_parameters.flaperon_scale);

	param_get(_parameter_handles.rattitude_thres, &_parameters.rattitude_thres);

	param_get(_parameter_handles.vtol_type, &_parameters.vtol_type);

	param_get(_parameter_handles.bat_scale_en, &_parameters.bat_scale_en);

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

	param_get(_parameter_handles.Kp_aileron, &_gain.Kp[0]);
	param_get(_parameter_handles.Kp_elevation, &_gain.Kp[1]);
	param_get(_parameter_handles.Kp_rudder, &_gain.Kp[2]);
	param_get(_parameter_handles.Kp_vel_thrust, &_gain.Kp[3]);
	param_get(_parameter_handles.Kd_aileron,  &_gain.Kd[0]);
	param_get(_parameter_handles.Kd_elevation, & _gain.Kd[1]);
	param_get(_parameter_handles.Kd_rudder, &_gain.Kd[2]);
	param_get(_parameter_handles.Kd_vel_thrust,  &_gain.Kd[3]);
	param_get(_parameter_handles.Ki_aileron, &_gain.Ki[0]);
	param_get(_parameter_handles.Ki_elevation, &_gain.Ki[1]);
	param_get(_parameter_handles.Ki_rudder, &_gain.Ki[2]);
	param_get(_parameter_handles.Ki_vel_thrust, &_gain.Ki[3]);

	return PX4_OK;
}

void
PerchingWall::vehicle_control_mode_poll()
{
	bool vcontrol_mode_updated;

	/* Check if vehicle control mode has changed */
	orb_check(_vcontrol_mode_sub, &vcontrol_mode_updated);

	if (vcontrol_mode_updated) {

		orb_copy(ORB_ID(vehicle_control_mode), _vcontrol_mode_sub, &_vcontrol_mode);
	}
}

void
PerchingWall::vehicle_manual_poll()
{
	bool manual_updated;

	/* get pilots inputs */
	orb_check(_manual_sub, &manual_updated);

	if (manual_updated) {

		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
	}
}

void
PerchingWall::vehicle_setpoint_poll()
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
PerchingWall::global_pos_poll()
{
	/* check if there is a new global position */
	bool global_pos_updated;
	orb_check(_global_pos_sub, &global_pos_updated);

	if (global_pos_updated) {
		orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	}
}

void
PerchingWall::vehicle_status_poll()
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
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}
}

void
PerchingWall::vehicle_land_detected_poll()
{
	/* check if there is new status information */
	bool vehicle_land_detected_updated;
	orb_check(_vehicle_land_detected_sub, &vehicle_land_detected_updated);

	if (vehicle_land_detected_updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}
}

void
PerchingWall::battery_status_poll()
{
	/* check if there is a new message */
	bool updated;
	orb_check(_battery_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(battery_status), _battery_status_sub, &_battery_status);
	}
}

void

PerchingWall::distance_poll()
{
	bool distance_updated;
	orb_check(_distance_trone_sub, &distance_updated);

	if (distance_updated) {
		orb_copy(ORB_ID(distance_sensor), _distance_trone_sub, &_distance_trone);
	}

    orb_check(_distance_vl53l0x_sub, &distance_updated);

    if (distance_updated) {
        orb_copy(ORB_ID(distance_sensor), _distance_vl53l0x_sub, &_distance_vl53l0x);
    }
}

void
PerchingWall::hover_control(const math::Quaternion &q)
{
	std::vector<float> outputs(4);

	calculation_error(q);
	discrete_derivative();
	discrete_integrate();

	// Calcul la valeur de chaque sortie
	for (int i = 0; i < (_outputs.size() - 1); ++i) {
		outputs[i] = ((_gain.Kp[i] * _euler_error(i)) + (_derivate(i) * _gain.Kd[i]) + (_integrate(i) * _gain.Ki[i]));

		// Conditions de staturation
		if (outputs[i] >= 1.0f) {
			outputs[i] = 1.0f;
		} else if (outputs[i] <= -1.0f) {
			outputs[i] = -1.0f;
		}
	}

	// Vertical velocity and position controller with switch at given pitch angle

	if (_level) {

		/* outputs[0] = outputs[0]/3;

         if (outputs[0] >= 0.25f) {
             outputs[0] = 0.25f;
         } else if (outputs[0] <= -0.25f) {
             outputs[0] = -0.25f;
         }*/

		outputs[3] = 0.55f;


	}


	else if (_wall_landing) {
		// Force l'élévation à 100% pour ralentir

		// Thrust feedback
		// 1.308 rad: 75 deg
		// 1.1345 rad: 65 deg
        if (q.to_euler()(1) < 1.31f && _hover_speed_comm > -0.0001f) { //&& _hover_speed_comm < 0.0f // making sure to not go back to 0.0f speed command
			_hover_speed_comm = 0.0f;

			// low saturation on ailerons and rudder during pitch-up

			//outputs[0] = outputs[0]/3.0f;
			//outputs[2] = outputs[2]/3.0f;

            //*************************// (2018-03-29)
            // Maintain elevator raised until reaching 1.3 rad (~ 70 deg)
            //if (q.to_euler()(1) < 1.0f){
             //  outputs[1] = 1.0f;
            //}
            //*************************//


            // Ailerons saturation approx until SSD (2017-10-05)
            if (outputs[0] >= 0.2f) { // used to be limited to 0.25
                outputs[0] = 0.2f;
            } else if (outputs[0] <= -0.2f) {
                outputs[0] = -0.2f;
			}

            // Rudder saturation approx until SSD (2017-10-05)
            if (outputs[2] >= 0.6f) { // used to be limited to 0.5
                outputs[2] = 0.6f;
            } else if (outputs[2] <= -0.6f) {
                outputs[2] = -0.6f;
            }


		} else {
            _hover_speed_comm = -0.001f; //was -0.2f... defined at -0.001f to avoid going back to first step of wall landing, while having speed command of approximately zero (2019-04-06)

           // outputs[0] = outputs[0] + 0.2f;

           /*
            if (outputs[2] >= 1.0f) {
                outputs[2] = 1.0f;
            } else if (outputs[2] <= -0.05f) {
                outputs[2] = -0.05f;
            }
            */
		}

		//_hover_speed_comm = 0.0f;

        outputs[3] = ((_gain.Kp[3] * _speed_error) + (_speed_derivate * _gain.Kd[3]) + _speed_integrate * _gain.Ki[3]) + 0.6f; // 0.65f avec prop 8x6 (2018-03-28)... 0.85f avec camera et prop 8x6 --> 2017/11/28
        //outputs[3] = 0.8f;

        // ****************** //
        //outputs[3] = 0.75f; // *****  Thrust constant (~ mg) pour tester LSM Thrust *****
        // ****************** //

        if (outputs[3] < 0.4f){
            outputs[3] = 0.4f;
		} else if (outputs[3] > 1.0f){
			outputs[3] = 1.0f;
		}

		// RECOVERY
	} else if (_recovery) {

        if (_time < 1.5f) {
            _hover_speed_comm = 1.0f;
		} else {
            _hover_speed_comm = 0.1f;
		}

        outputs[3] = ((_gain.Kp[3] * _speed_error) + (_speed_derivate * _gain.Kd[3]) + _speed_integrate * _gain.Ki[3]) + 0.65f;

		if (outputs[3] < 0.23f){
            outputs[3] = 0.23f;		} else if (outputs[3] > 1.0f){
			outputs[3] = 1.0f;
		}


		// Thrust feedback through integration of IMU measurements for vertical velocity
	} else if (_take_off) {
        if (_time < 1.5f) {
			//outputs[3] = 0.68f;
            _hover_speed_comm = 2.0f;
			//outputs[3] = 0.60f;
            outputs[3] = ((_gain.Kp[3] * _speed_error) + (_speed_derivate * _gain.Kd[3]) + _speed_integrate * _gain.Ki[3]) + 0.7f; // 1.0 avec camera
			if (outputs[3] > 1.0f)
				outputs[3] = 1.0f;
				//outputs[3] = 0.0f;
			else if (outputs[3] < 0.5f)
				outputs[3] = 0.5f;
			//outputs[3] = 0.0f;
		} else {
			//outputs[3] = 0.60f;
            _hover_speed_comm = 0.5f;
            outputs[3] = ((_gain.Kp[3] * _speed_error) + (_speed_derivate * _gain.Kd[3]) + _speed_integrate * _gain.Ki[3]) + 0.7f; // 1.0 avec camera
			if (outputs[3] > 1.0f)
				outputs[3] = 1.0f;
				//outputs[3] = 0.0f;
			else if (outputs[3] < 0.5f)
				outputs[3] = 0.5f;
			//outputs[3] = 0.0f;
		}

    //}

     } else if (_climb) {

        // ******************** CLIMB **************************

        //*** Saturation on ailerons (2018-03-07)
        // Redefined further down as a feedforward command depending on thrust
        if (outputs[0] >= 0.3f){
            outputs[0] = 0.3f;

        }else if (outputs[0] <= -0.3f){
            outputs[0] = -0.3f;
        }


        outputs[0] = outputs[0] - 0.1f;

        //*** Constant command for ailerons in climb mode (2018-03-08)
        //outputs[0] = 0.3f;

        //*********************************//
        // Apply 1.5 additional gain on Kp and Kd for elevator during climb (2018-03-29)
        //outputs[1] = 1.5f*outputs[1];
        //*********************************//

        //*** Saturation on rudder (2018-03-07)
        if (outputs[2] >= 0.15f){ // was 0.15f
            outputs[2] = 0.15f;

        }else if (outputs[2] <= -0.15f){
            outputs[2] = -0.15f;
        }



        if (_climb_take_off) {

              _hover_speed_comm = 0.5f;

              outputs[3] = ((_gain.Kp[3] * _speed_error) + (_speed_derivate * _gain.Kd[3]) + _speed_integrate * _gain.Ki[3]) + 0.65f;

             if (outputs[3] > 1.0f){
                 outputs[3] = 1.0f;
             }else if (outputs[3] < 0.5f){
                 outputs[3] = 0.5f;
             }

             //*** Full throttle all along (2018-04-02)
             if (_climb_or_descent){
                 outputs[3] = 1.0f;
             } else if (!_climb_or_descent){
                 outputs[3] = 0.7f;
             }


             //*** Feedforward command for ailerons in climb mode --> Proportional to thrust command (2018-04-02)
             //outputs[0] = 0.3f*outputs[3];

             //**** Rudder command constant reduction in climb mode
             outputs[2] = outputs[2] - 0.05f; // to take into account constant error on yaw (always drifts away to the right)


         } else if (_climb_hover){

              _hover_speed_comm = 0.15f;

            outputs[3] = ((_gain.Kp[3] * _speed_error) + (_speed_derivate * _gain.Kd[3]) + _speed_integrate * _gain.Ki[3]) + 0.65f;

             if (outputs[3] > 1.0f){
                 outputs[3] = 1.0f;
             }else if (outputs[3] < 0.35f){
                 outputs[3] = 0.35f;
             }




             //*** Throttle approximately enough for thrust to be equal to weight (2018-03-06)
             outputs[3] = 0.65f;

             //*** Feedforward command for ailerons in climb mode --> Proportional to thrust command (2018-03-09)
             //outputs[0] = 0.7f*outputs[3];



             // Full throttle during climb
             //outputs[3] = 1.0f;

             //***************************************************
             // Feedforward ramp-down command for thrust (2018-04-02)
             /*outputs[3] = 1.0f - 0.25f*(_time - 1.5f);

             if (outputs[3] > 1.0f){
                 outputs[3] = 1.0f;
             }else if (outputs[3] < 0.7f){
                 outputs[3] = 0.7f;
             }*/

             //***************************************************

             //outputs[3] = 0.0f;

             //*** Feedforward command for ailerons in climb mode --> Proportional to thrust command (2018-03-09)
             //outputs[0] = 0.8f*outputs[3]; // was 0.7f before.. comment on ***2018-05-04***

        } else {


            //***************************************************
            // Feedforward ramp-down command for thrust (2018-04-02)
            outputs[3] = 0.65f - 0.65f*(_time - (_time_climb + 0.5f));

            if (outputs[3] > 0.7f){
                outputs[3] = 0.7f;
            }else if (outputs[3] < 0.0f){
                outputs[3] = 0.0f;
            }

            //***************************************************


            //*** Feedforward command for ailerons in climb mode --> Proportional to thrust command (2018-03-09)
            outputs[0] = 0.8f*outputs[3]; // was 0.7fbefore
         }



}

	_actuators.control[actuator_controls_s::INDEX_ROLL] = outputs[0];
	_actuators.control[actuator_controls_s::INDEX_PITCH] = outputs[1];
	_actuators.control[actuator_controls_s::INDEX_YAW] = outputs[2];
	_actuators.control[actuator_controls_s::INDEX_THROTTLE] = outputs[3];
    
}

void
PerchingWall::calculation_error(const math::Quaternion & q)
{
	_qm = q.conjugated();
	_qe = _qm * _qd;
	_euler_error = _qe.to_euler();

	/* Speed estimation */
	//AccelZ = -(-PixAccelX.*sin(PixPitch*pi/180) + PixAccelZ.*cos(PixPitch*pi/180) + 9.714);

	//float accelZ = -(-_ctrl_state.x_acc*(float)sin(q.to_euler()(1)) + _ctrl_state.z_acc*(float)cos(q.to_euler()(1)) + 9.714f); // g = 9.714


	float angle2 = q.to_euler()(1);
	float angle3 = q.to_euler()(0);

	float s2 = (float)sin(angle2);
	float c2 = (float)cos(angle2);
	float s3 = (float)sin(angle3);
	float c3 = (float)cos(angle3);

	float ax = _ctrl_state.x_acc;
	float ay = _ctrl_state.y_acc;
	float az = _ctrl_state.z_acc;

	float gi = 9.8f;

	accelZ = -(-ax*s2 + ay*s3*c2 + az*c2*c3 + gi);


	_speed_z_axis_old = _speed_z_axis;
	_speed_z_axis = _speed_z_axis_old + accelZ * _dt; //

	// if (angle2 > 1.39f) {
    _speed_error = _hover_speed_comm - (_speed_z_axis); // float added ro _speed_z_axis as speed calculation bias
	//} else {
	//    _speed_error = _hover_speed_comm - _speed_z_axis;
	// }
}

void
PerchingWall::discrete_derivative()
{
	_derivate = (_euler_error - _euler_error_old) / _dt;
	_euler_error_old = _euler_error;

	_speed_derivate = (_speed_error - _speed_error_old) / _dt;
	_speed_error_old = _speed_error;

}

void
PerchingWall::discrete_integrate()
{
	_integrate = (_euler_error * _dt) + _integrate_old;
	_integrate_old = _integrate;

	_speed_integrate = (_speed_error * _dt) + _speed_integrate_old;
	_speed_integrate_old = _speed_integrate;

}

void
PerchingWall::task_main_trampoline(int argc, char *argv[])
{
	perching_control::g_control->task_main();
}

void
PerchingWall::task_main()
{
	/*
	 * do subscriptions
	 */
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_vcontrol_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_battery_status_sub = orb_subscribe(ORB_ID(battery_status));
	_distance_trone_sub = orb_subscribe_multi(ORB_ID(distance_sensor),0);
    _distance_vl53l0x_sub = orb_subscribe_multi(ORB_ID(distance_sensor),1);

	// Set l'intervale à 200Hz ( boucle de controle )
	orb_set_interval(_ctrl_state_sub, 5);


	parameters_update();

	/* get an initial update for all sensor and status data */
	vehicle_setpoint_poll();
	vehicle_control_mode_poll();
	vehicle_manual_poll();
	vehicle_status_poll();
	vehicle_land_detected_poll();
	battery_status_poll();
	distance_poll();

	/* wakeup source */
	px4_pollfd_struct_t fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _ctrl_state_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {
		static int loop_counter = 0;

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

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		/* only run controller if attitude changed */
		if (fds[1].revents & POLLIN) {
			static uint64_t last_run = 0;
			float deltaT = (hrt_absolute_time() - last_run) / 1000000.0f;
			last_run = hrt_absolute_time();


			/* guard against too large deltaT's */
			if (deltaT > 1.0f) {
				deltaT = 0.01f;
			}

			/* load local copies */
			orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);


			/* get current rotation matrix and euler angles from control state quaternions */
			math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
			_R = q_att.to_dcm();


// CODE INITIAL SUpprimer entre ces deux sections...


			vehicle_status_poll();
			vehicle_manual_poll();
			distance_poll();
#if 0
			if (_landed && (_accel.x*_accel.x + _accel.z*_accel.z) > 500.0f)
                _accel_stop = true;
#endif

            _time_climb = 4.0f;


            if (_take_off || _recovery || _climb) {

				_time += _dt; // deltaT

                // Switch between climb, hover, and thrust relaxation
                if (_climb) {
                   if (_time < _time_climb) {
                       _climb_take_off = true;
                       _climb_hover = false;
                   } else if (_time > _time_climb && _time < (_time_climb + 0.5f)) {
                       _climb_take_off = false;
                       _climb_hover = true;
                   } else {
                       _climb_take_off = false;
                       _climb_hover = false;
                   }
                }
			}

          /*  if ((_ctrl_state.x_acc > 2.00f * _g) || (_time_level > 0)) {

				_time_level += _dt; // deltaT
            }


            // yaw_sp and Vz fixed for feedback at wall detection
            if (_wall_landing) { // ((_ctrl_state.x_acc > (2.05f * _g)) && !_wall_landing && !_landed && !_climb && !_take_off)
				if (!_flag_qd_calculated) {

					_yaw_sp = q_att.to_euler()(2);
                    _speed_z_axis = 0.0f;

					_flag_qd_calculated = true;
				}
			}
            */

            // yaw_sp fixed for feedback at launch
            if ((float (fabs(_ctrl_state.x_acc)) > (2.00f * _g)) && !_wall_landing && !_landed && !_take_off && !_mb_landed && !_recovery) { // ((_ctrl_state.x_acc > (2.05f * _g)) && !_wall_landing && !_landed && !_climb && !_take_off)
                if (!_flag_qd_calculated) {
                    _yaw_sp = q_att.to_euler()(2);
                    _flag_qd_calculated = true;
                }
            }



            // unflag _flag_qd_calculated when landed, to prepare for next perching attempt
            if (_landed) { // ((_ctrl_state.x_acc > (2.05f * _g)) && !_wall_landing && !_landed && !_climb && !_take_off)
                if (_flag_qd_calculated) {
                    _flag_qd_calculated = false;
                }
            }


            //*******************************************//
            // accX and accZ out of control loop calculation
            float angle2 = q_att.to_euler()(1);
            float angle3 = q_att.to_euler()(0);

            //float s2 = (float)sin(angle2);
            float c2 = (float)cos(angle2);
            float c3 = (float)cos(angle3);
           /* float s3 = (float)sin(angle3);

            float ax = _ctrl_state.x_acc;
            float ay = _ctrl_state.y_acc;
            float az = _ctrl_state.z_acc;

            float gi = 9.8f;

           // float accX = ax*c2 + az*s2;
            float accZ = -(-ax*s2 + ay*s3*c2 + az*c2*c3 + gi); */

            // Define shorter name for vl53l0x readings
            float dist_vl53l0x = _distance_vl53l0x.current_distance;

            // Calcul de Xc --> Pour determiner si contact entre griffes et mur
            //float Xbs = 45.0f; // Distance (mm) entre le CM et le point de fixation du Belly Sensor
            //float XpBcm = 20.0f; // Distance (mm) entre le CM et le point de fixation des pattes
            float XpBs = 90.0f; // Distance (mm) entre le point de fixation des pattes et le Belly Sensor
            float qSF = 33.0f*3.1416f/180.0f; // Angle entre patte et fuselage de l'avion
            float c2_XpS = (float)cos(angle2 - 3.1416f/2.0f);
            float Xp = 325.0f; // Longueur des pattes (mm)
            float XpS = Xp*(float)sin(qSF); // Longueur des pattes projetee sur l'axe bz
            float XpL = Xp/(float)cos(qSF); //Longueur des pattes projetee sur l'axe bx

            // *** Detect if pitch above 90 deg, with roll data --> Re-define angle as being larger than 90 deg
            //if ((float (fabs(angle3))) >= 2.0f){ // Might have to change this condition for something else...
            //    angle2 = (180.0f/3.1416f-angle2);
            //}

            //float anglePatte = angle2 - qSF; // Angle entre horizontale et patte
            //float c2Patte = (float)cos(anglePatte); // Cosinus de l'angle entre horizontale et patte
            //float Xc = XpS + (float)pow((pow(XpL,2.0f) - pow(XpL*s2,2.0f)),0.5f)*c3 - XpBs*c3*c2; //  XpL*c3*c2
            float Xc = XpS*c2_XpS + XpL*c3*c2 - XpBs*c3*c2;
            float Xc_m = (float)pow((pow(dist_vl53l0x,2.0f) - pow(dist_vl53l0x*c3*c2,2.0f)),0.5f);





            //*******************************************//



			// MAYBE LANDED
       // OLD - TOTAL ACCEL     if ((q_att.to_euler()(1) > 1.31f && (_wall_landing && (_ctrl_state.x_acc*_ctrl_state.x_acc + _ctrl_state.z_acc*_ctrl_state.z_acc) > 200.0f)) || _mb_landed || (_recovery && (_time >= 3.0f)) ) { // || (_climb && (_time_climb >= 3.0f))) {




          //  if ((q_att.to_euler()(1) > 1.396f && (_distance_vl53l0x.current_distance <= 30.0f/((float)cos(q_att.to_euler()(1)))) && _time_level > 1.3f && (_manual.return_switch != manual_control_setpoint_s::SWITCH_POS_ON)) || _mb_landed || (_recovery && (_time >= 20.0f)) ) { // || (_climb && (_time_climb >= 3.0f))) {

            //if ((_wall_landing && q_att.to_euler()(1) > 1.05f && Xc_m <= Xc && accX < -2.0f) || _mb_landed || (_recovery && (_time >= 20.0f)) ) { // || (_climb && (_time_climb >= 3.0f))) {

            //***********************************//
            // ***Conditions to land***
            bool _wall_landing_land_cond = _wall_landing && q_att.to_euler()(1) > 1.00f && Xc_m <= (Xc + 100.0f); // pitch > 1.396f if accelerometer for detection
            bool _recovery_land_cond = _recovery && (_time >= 20.0f);
            bool _climb_land_cond = _climb && _time >= (_time_climb + 1.5f); // other condition previously used: (_climb && Xc_m <= (Xc - 10.0f) && _time > 2.5f) ||
            //***********************************//

            // **** 1.05f in the future for pitch angle threshold *****
                if (_wall_landing_land_cond || _recovery_land_cond || _climb_land_cond || _mb_landed) { // || (_climb && (_time_climb >= 3.0f))) {

                // ((_wall_landing && q_att.to_euler()(1) > 1.396f && Xc_m <= Xc && (_ctrl_state.x_acc*_ctrl_state.x_acc + _ctrl_state.z_acc*_ctrl_state.z_acc) > 200.0f) || _mb_landed || (_recovery && (_time >= 20.0f)) ) { --> 30-01-2018

                //  OLD - HOR ACCEL    if ((q_att.to_euler()(1) > 1.396f && (_wall_landing && accX < -9.0f) && _time_level > 1.3f && (_manual.return_switch != manual_control_setpoint_s::SWITCH_POS_ON)) || _mb_landed || (_recovery && (_time >= 20.0f)) ) { // || (_climb && (_time_climb >= 3.0f))) {
                //  if ((q_att.to_euler()(1) > 1.34f && ((_wall_landing && (_ctrl_state.x_acc*_ctrl_state.x_acc + _ctrl_state.z_acc*_ctrl_state.z_acc) > 200.0f) || (_take_off && (_ctrl_state.x_acc*_ctrl_state.x_acc + _ctrl_state.z_acc*_ctrl_state.z_acc) > 200.0f)))|| _landed || (_climb && !_climb_take_off && (_ctrl_state.x_acc*_ctrl_state.x_acc + _ctrl_state.z_acc*_ctrl_state.z_acc) > 100000.0f) || (_climb && _time_climb > 100.0f) || (_wall_landing && _manual.acro_switch == manual_control_setpoint_s::SWITCH_POS_ON) ) {
				_wall_landing = false;
				_recovery = false;
				_landed = false;
				_take_off = false;
				_climb = false;
                _man_flight = false;
                //_flag_qd_calculated = false;
				_mb_landed = true;
                //_time_climb = 0; // commented on 2018-05-02 --> this was probably the thing leading bad elevator command, always in _climb_hover and thrust relaxation modes
                _time = 0;

                _actuators.control[actuator_controls_s::INDEX_ROLL] = 0.0f ; // + _parameters.trim_roll;
                _actuators.control[actuator_controls_s::INDEX_PITCH] = 0.0f ; // + _parameters.trim_pitch;
                _actuators.control[actuator_controls_s::INDEX_YAW] = 0.0f ; // + _parameters.trim_yaw;


                // _time is not re-initialized to 0 if we come back to _mb_landed after recovery... to do!
                //if (_time > 0.01f) { // maybe use >= instead of >
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f ; // 0.25f;
                //}
                //else {
                //    _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
                //}


                // PERCHING ****distance_trone <= 5.2f sans camera --> 2017/11/28
            } else if (_wall_landing || (_distance_trone.current_distance <= 5.5f/((float)cos(q_att.to_euler()(1))) && (_distance_trone.current_distance > 0.0f) && _flag_qd_calculated && !_take_off && !_recovery && !_mb_landed && !_landed && !_climb)) { //(_threshold_accel && (_distance.current_distance < 5.2f || _wall_landing))

                   // ***** && (_distance.current_distance > 0.0f) ---> condition necessaire pcq dehors le capteur switch de la mesure reelle au minimum (0.2 m), si trop de lumiere du soleil

                    //_launch = false;
				_level = false;
                _man_flight = false;
				_wall_landing = true;

				_time = 0; // set to zero for start of checks in _mb_landed modes

				// Define quaternion setpoint
				if (q_att.to_euler()(1) > 0.7854f) {
                    _qd.from_euler(0.0f, 1.4835f, _yaw_sp); //83 deg = 1.4486... 85 deg = 1.4835... 87 deg = 1.5184
				} else {
                    _qd.from_euler(0.0f, 1.4835f, _yaw_sp);
				}

				hover_control(q_att);

			}

				// Level flight feedback
            /*else if (_level || ((_time_level >= 0.35f) && (_distance.current_distance > 6.0f/((float)cos(q_att.to_euler()(1)) + 0.0001f)) && !_wall_landing && !_take_off && !_mb_landed && !_landed && !_climb && !_recovery)) {
				//_launch = false;
				_level = true;

				_qd.from_euler(0.0f, 0.175f, _yaw_sp);

				hover_control(q_att);
            }*/

            else {
            //else if (_man_flight || (_take_off && _time >= 3.0f)) {


				// LAUNCH
                _actuators.control[actuator_controls_s::INDEX_ROLL] = 0.0f ; // + _parameters.trim_roll;
                _actuators.control[actuator_controls_s::INDEX_PITCH] = 0.35f ; // + _parameters.trim_pitch;
                _actuators.control[actuator_controls_s::INDEX_YAW] = 0.00f ; // + _parameters.trim_yaw;
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.25f;


                /*
                // VOL MANUEL
                _actuators.control[actuator_controls_s::INDEX_ROLL] = _manual.y + _parameters.trim_roll;
                _actuators.control[actuator_controls_s::INDEX_PITCH] = _manual.x + _parameters.trim_pitch;
                _actuators.control[actuator_controls_s::INDEX_YAW] = _manual.r + _parameters.trim_yaw;
                _actuators.control[actuator_controls_s::INDEX_THROTTLE] = _manual.z;
                */

            }




			// RECOVERY
			//if((_mb_landed && (_time >= 0.2f) && (float (fabs(_ctrl_state.x_acc)) > 0.0f)) || _recovery || (_landed && (float (fabs(_ctrl_state.x_acc)) > 2.0f))) {
            if(_recovery) { // || (_landed && (accelZ < -3.0f))) {

                // _mb_landed && (_time >= 0.20f) && (accZ <= -300.0f) --> 30-01-2018

                // ****Use these conditions for _time variable initialization?...****
                _mb_landed = false;
				_landed = false;
				_recovery = true;
				//_time = 0;

                if (_time < 1.5f) {
                    _qd.from_euler(0.0f, 1.7f, _yaw_sp);
				} else {
                    _qd.from_euler(0.0f, 1.55f, _yaw_sp);
				}

				hover_control(q_att);


            }

            // LANDED
            if((_mb_landed) || _landed || _manual.acro_switch == manual_control_setpoint_s::SWITCH_POS_ON) { // || _manual.acro_switch == manual_control_setpoint_s::SWITCH_POS_ON) {
               // 2018-05-02 --> if((_mb_landed) || _landed || (_take_off && (_ctrl_state.x_acc*_ctrl_state.x_acc + _ctrl_state.z_acc*_ctrl_state.z_acc) > 1000000.0f) || _manual.acro_switch == manual_control_setpoint_s::SWITCH_POS_ON) { // || _manual.acro_switch == manual_control_setpoint_s::SWITCH_POS_ON) {

               // LATEST WITH USE OF RECOVERY, USING TIME CHECK WITH _mb_landed
               // if((_mb_landed && (_time >= 2.0f)) || _landed || (_take_off && (_ctrl_state.x_acc*_ctrl_state.x_acc + _ctrl_state.z_acc*_ctrl_state.z_acc) > 1000000.0f) || _manual.acro_switch == manual_control_setpoint_s::SWITCH_POS_ON) { // || _manual.acro_switch == manual_control_setpoint_s::SWITCH_POS_ON) {


				// if((_mb_landed && (_time >= 2.0f)) || _landed || (_take_off && (_ctrl_state.x_acc*_ctrl_state.x_acc + _ctrl_state.z_acc*_ctrl_state.z_acc) > 200.0f) || _manual.acro_switch == manual_control_setpoint_s::SWITCH_POS_ON) { // || _manual.acro_switch == manual_control_setpoint_s::SWITCH_POS_ON) {

				_wall_landing = false;
				_recovery = false;
				//_threshold_accel = false;
				_take_off = false;
				_climb = false;
				_flag_qd_calculated = false;
				_mb_landed = false;
				_landed = true;

				_time = 0;

				//hover_control(q_att);

				_speed_z_axis = 0.0f;


                _actuators.control[actuator_controls_s::INDEX_ROLL] = 0.0f ; // + _parameters.trim_roll;
                _actuators.control[actuator_controls_s::INDEX_PITCH] = 0.0f ; // + _parameters.trim_pitch;
                _actuators.control[actuator_controls_s::INDEX_YAW] = 0.0f ; // + _parameters.trim_yaw;
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
			}




			// TAKE OFF
        /*    if (_take_off || (_manual.return_switch == manual_control_setpoint_s::SWITCH_POS_ON)) {
				_landed = false;
				_take_off = true;

                _qd.from_euler(0.0f, 1.83f, _yaw_sp); // 1.83f (115 deg) pour les tests dehors,,,

				hover_control(q_att);
			}

        */


            //*******************************************//
            // CLIMB MODE

            // Proportional gain for wall distance feedback loop during climbing
            float Kp_climb = 0.05f; // input: millimeters, output: degrees (ex.: 2 deg command change for 1 cm error)

            if (_climb || (_landed && _manual.return_switch == manual_control_setpoint_s::SWITCH_POS_ON )) {

                if (!_climb && !_climb_or_descent){
                    _climb_or_descent = true; // if first time getting in this _climb condition after perching, and WAS NOT climbing last time it entered climb state --> climb upward
                } else if (!_climb && _climb_or_descent){
                    _climb_or_descent = false; // if first time getting in this _climb condition after perching, and WAS climbing last time it entered climb state --> descend downward
                }

                _mb_landed = false;
                _landed = false;
                _climb = true;


                if (_climb_take_off) {
                    _dist_comm = 300.0f; // fb loop command: horizontal distance between belly sensor and wall --> climb with 5 cm distance between foot tip and wall
                } else {
                    //**** Addition used for temporary testing --> Change for a SUBTRACTION when done!!!!!
                    _dist_comm = 0.0f; // fb loop command: horizontal distance between belly sensor and wall --> climb with 5 cm distance between foot tip and wall
                }

                //Calculate the pitch_err
                float _dist_err = _dist_comm - Xc_m; // error calculation for fb loop
                float _pitch_err = Kp_climb * _dist_err; // proportional gain multiplication
                _pitch_sp = 90.0f + _pitch_err;


                if (!_climb_take_off) {
                    _pitch_sp = 80.0f; // min pitch command if not climb_take_off mode
                }

                // _pitch_sp saturation
                if (_pitch_sp < 80.0f){
                    _pitch_sp = 80.0f;
                }else if (_pitch_sp > 105.0f){
                    _pitch_sp = 105.0f;
                }

                // Degrees to radians
                _pitch_sp = _pitch_sp/180.0f*3.1416f;

                //_pitch_sp = 90.0f/180.0f*3.1416f; // used to determine thrust command to maintain hover

                // Commands for attitude control loop
                _qd.from_euler(0.0f, _pitch_sp, _yaw_sp);

                // ***Run attitude controller***
                hover_control(q_att);
            }
            //*******************************************//



			/* Vérifie que l'autopilot est armee avant d'envoyer la commande au thrust */
			if (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = _actuators.control[actuator_controls_s::INDEX_THROTTLE];
			} else {
				_actuators.control[actuator_controls_s::INDEX_THROTTLE] = 0.0f;
			}
#if 0
			// Détache les griffes
            if (_manual.mode_switch == manual_control_setpoint_s::SWITCH_POS_ON && _counter++ < 1) {
                _actuators.control[4] = 1.0f;
            } else if (_counter >= 1){
                _actuators.control[4] = 0.0f;
            }
#endif

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
}

int
PerchingWall::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("fw_att_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&PerchingWall::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return PX4_OK;
}

int perching_wall_main(int argc, char **argv)
{
	if (argc < 2) {
		warnx("usage: fw_att_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (perching_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		perching_control::g_control = new PerchingWall;

		if (perching_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (PX4_OK != perching_control::g_control->start()) {
			delete perching_control::g_control;
			perching_control::g_control = nullptr;
			warn("start failed");
			return 1;
		}

		/* check if the waiting is necessary at all */
		if (perching_control::g_control == nullptr || !perching_control::g_control->task_running()) {

			/* avoid memory fragmentation by not exiting start handler until the task has fully started */
			while (perching_control::g_control == nullptr || !perching_control::g_control->task_running()) {
				usleep(50000);
				printf(".");
				fflush(stdout);
			}

			printf("\n");
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (perching_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete perching_control::g_control;
		perching_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (perching_control::g_control) {
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
