/****************************************************************************
 *
 *   Copyright (c) 2015, 2016 Airmind Development Team. All rights reserved.
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
 * 3. Neither the name Airmind nor the names of its contributors may be
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
 * @file charging_i2c.cpp
 * @author Gabriel Guilmain
 *
 * Driver for MAX17205 (need custom QgroundControl Builds to monitor battery parameters)
 */

#include <px4_config.h>
#include <px4_defines.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <vector>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <systemlib/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/subsystem_info.h>
#include <uORB/topics/distance_sensor.h>

// on inclu le nouveau UORB topic spécifique pour le driver charging i2c
#include <uORB/topics/charging_info.h>
#include <uORB/topics/charging_info_2.h>

#include <board_config.h>

/* Configuration Constants */

// JASSUME QUE CEST LI2C SUR LE CONECTEUR EXTERNE: À VALIDER !!!
#define SRF02_I2C_BUS 		PX4_I2C_BUS_ONBOARD

// LADRESSE -> VALIDER CE QUE LON VEUT
#define SRF02_I2C_BASEADDR 	0x36 /* vaut 0x6C selon la convention du chip de balancing. ladresse est tassé vers la droite et le dernier bit est sous-entendu */

// LADRESSE -> PATH DU DEVICE DANS LOS NuttX ??? (PAS CERTAIN)
#define SRF02_DEVICE_PATH	"/dev/charging_i2c"

/* MB12xx Registers addresses */

#define SRF02_TAKE_RANGE_REG	0x51		/* Measure range Register */
#define SRF02_SET_ADDRESS_0	0xA0		/* Change address 0 Register */
#define SRF02_SET_ADDRESS_1	0xAA		/* Change address 1 Register */
#define SRF02_SET_ADDRESS_2	0xA5		/* Change address 2 Rsrf02egister */

/* Device limits */
#define SRF02_MIN_DISTANCE 	(0.20f)
#define SRF02_MAX_DISTANCE 	(6.00f)

// le timing entre chaque mesure est 60 ms. la collecte automatique a cette frequence
// est partie dans la fonction ioctl
#define SRF02_CONVERSION_INTERVAL 	100000 /* 60ms for one sonar */
#define TICKS_BETWEEN_SUCCESIVE_FIRES 	100000 /* 30ms between each sonar measurement (watch out for interference!) */

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class CHARGING_I2C : public device::I2C
{
public:
	CHARGING_I2C(int bus = SRF02_I2C_BUS, int address = SRF02_I2C_BASEADDR);
	// la declaration de la fonction definit deja le BUS et lADRESS

	virtual ~CHARGING_I2C();

	virtual int 		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();

protected:
	virtual int			probe();

private:

	///////////////////////////////////////////////////////////////////////////////////
	// TEST: UNE COPY LOCALE DE PARAMETRES CUSTOM POUR LE DRIVER CHARGING_I2C

	
	struct {

		float is_new_bat;
		float bat_cap;
		float rsense;
		float balance_threshold;

	}	_parameters;			
	

	struct {

        param_t is_new_bat;
        param_t bat_cap;
        param_t rsense;
        param_t balance_threshold;

	}	_parameter_handles;		/**< handles for interesting parameters */
	///////////////////////////////////////////////////////////////////////////////////

	float				_min_distance;
	float				_max_distance;
	work_s				_work;
	ringbuffer::RingBuffer		*_reports;
	bool				_sensor_ok;
	int					_measure_ticks;
	bool				_collect_phase;
	int					_class_instance;
	int					_orb_class_instance;

	orb_advert_t		_distance_sensor_topic;
	// test topic custom pour charging i2c
	orb_advert_t     	_charging_info_topic;

	// pour subsrcibe au topic du charging_info (il y a des info que lon veut recevoir)
	int			_charging_info_sub_2;
	struct charging_info_2_s		_charging_2;

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	uint8_t				_cycle_counter;	/* counter in cycle to change i2c adresses */
	int				_cycling_rate;	/* */
	uint8_t				_index_counter;	/* temporary sonar i2c address */
	std::vector<uint8_t>	addr_ind; 	/* temp sonar i2c address vector */
	std::vector<float>
	_latest_sonar_measurements; /* vector to store latest sonar measurements in before writing to report */

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();


	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return			True if the device is present.
	*/
	int					probe_address(uint8_t address);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	* Set the min and max distance thresholds if you want the end points of the sensors
	* range to be brought in at all, otherwise it will use the defaults SRF02_MIN_DISTANCE
	* and SRF02_MAX_DISTANCE
	*/
	void				set_minimum_distance(float min);
	void				set_maximum_distance(float max);
	float				get_minimum_distance();
	float				get_maximum_distance();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				cycle();
	int					measure();
	int					collect();
	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg		Instance pointer for the driver that is polling.
	*/
	static void			cycle_trampoline(void *arg);


};

/*
 * Driver 'main' command.
 */
extern "C" { __EXPORT int charging_i2c_main(int argc, char *argv[]);}

CHARGING_I2C::CHARGING_I2C(int bus, int address) :
	I2C("MB12xx", SRF02_DEVICE_PATH, bus, address, 100000), // _address est private on appelle donc le constructor de la classe mere
								// la frequence du bus est à 100 khz ***
	_min_distance(SRF02_MIN_DISTANCE),
	_max_distance(SRF02_MAX_DISTANCE),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_distance_sensor_topic(nullptr),
	_charging_info_topic(nullptr),	// charging_i2c custom topic
	_sample_perf(perf_alloc(PC_ELAPSED, "srf02_i2c_read")),
	_comms_errors(perf_alloc(PC_COUNT, "srf02_i2c_comms_errors")),
	_cycle_counter(0),	/* initialising counter for cycling function to zero */
	_cycling_rate(0),	/* initialising cycling rate (which can differ depending on one sonar or multiple) */
	_index_counter(0) 	/* initialising temp sonar i2c address to zero */

{
	_parameter_handles.is_new_bat = param_find("IS_NEW_BATTERY");
	_parameter_handles.bat_cap = param_find("BATT_CAPA_MAH");
	_parameter_handles.rsense = param_find("RSENSE_MOHM");
	_parameter_handles.balance_threshold = param_find("BALANCE_THRESH");

  	parameters_update();

	/* enable debug() calls */
	_debug_enabled = false;

	/* work_cancel in the dtor will explode if we don't do this... */
	memset(&_work, 0, sizeof(_work));
}

// constructor utilse initialisation list. cest plus performant...

CHARGING_I2C::~CHARGING_I2C()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

//********************************************//
// PARAMETER UPDATE                         ////
//********************************************//
int
CHARGING_I2C::parameters_update()
{

	param_get(_parameter_handles.is_new_bat, &_parameters.is_new_bat);
	param_get(_parameter_handles.bat_cap, &_parameters.bat_cap);
	param_get(_parameter_handles.rsense, &_parameters.rsense);
	param_get(_parameter_handles.balance_threshold, &_parameters.balance_threshold);

        return OK;
}

int
CHARGING_I2C::init()
{
	int ret = PX4_ERROR; // -1 -> valeur standard

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		warn("I2C init failed (debug)");
		return ret;
	}

	/* allocate basic report buffers */
	//_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));
	_reports = new ringbuffer::RingBuffer(2, sizeof(charging_info_s));

	// TODO: a quoi ca sert?
	_index_counter = SRF02_I2C_BASEADDR;	/* set temp sonar i2c address to base adress */
	set_address(_index_counter);		/* set I2c port to temp sonar i2c adress */

	if (_reports == nullptr) {
		return ret;
	}

	// TODO: a quoi ca sert?
	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	/* get a publish handle on the range finder topic */
	//struct distance_sensor_s ds_report = {};
	struct charging_info_s ds_report = {};

	// TODO: changer de topic pour le nouveau
	//_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
	//			 &_orb_class_instance, ORB_PRIO_LOW);

	_charging_info_topic = orb_advertise_multi(ORB_ID(charging_info), &ds_report,
				 &_orb_class_instance, ORB_PRIO_LOW);

	if (_charging_info_topic == nullptr) {
		DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
	}

	// XXX we should find out why we need to wait 200 ms here
	usleep(200000);

	/* check for connected rangefinders on each i2c port:
	   We start from i2c base address (0x70 = 112) and count downwards
	   So second iteration it uses i2c address 111, third iteration 110 and so on*/
	// ON NA PAS BESOIN DE FAIRE CELA PUISQUE LON CONNAIT NOTRE ADRESSE ???
	//TODO: A TESTER!

	// on enleve MB12XX_MAX_RANGEFINDERS comme condition de la boucle suivante
	// permettait de faire un scroll de ladresse avec plusieurs essaie mais on en a pas besoin
	for (unsigned counter = 0; counter <= 0; counter++) {
		_index_counter = SRF02_I2C_BASEADDR + counter * 2;	/* set temp sonar i2c address to base adress - counter */
		set_address(_index_counter);				/* set I2c port to temp sonar i2c adress */
		int ret2 = measure();

		if (ret2 == 0) { /* sonar is present -> store address_index in array */
			addr_ind.push_back(_index_counter);
			DEVICE_DEBUG("sonar added");
			_latest_sonar_measurements.push_back(200);
		}

		// pour debug notre device repond a quel adresse (en base 10)
		warn("ladresse est (BASE 10): %i", _index_counter);
	}

	_index_counter = SRF02_I2C_BASEADDR;
	set_address(_index_counter); /* set i2c port back to base adress for rest of driver */

	// cycling rate utilisé (dans notre cas, il y a un seul capteur detecté)
	/* if only one sonar detected, no special timing is required between firing, so use default */
	if (addr_ind.size() == 1) {
		_cycling_rate = SRF02_CONVERSION_INTERVAL;

	} else {
		_cycling_rate = TICKS_BETWEEN_SUCCESIVE_FIRES;
	}

	/* show the connected sonars in terminal */
	for (unsigned i = 0; i < addr_ind.size(); i++) {
		DEVICE_LOG("sonar %d with address %d added", (i + 1), addr_ind[i]);
	}

	DEVICE_DEBUG("Number of sonars connected: %d", addr_ind.size());

	ret = OK;
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;

	return ret;
}

int
CHARGING_I2C::probe()
{
	warnx("probe class charging i2c (debug)"); // debug erreur
	// measure() dans la classe charging I2C
	//return measure();
	return OK;
}

void
CHARGING_I2C::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
CHARGING_I2C::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
CHARGING_I2C::get_minimum_distance()
{
	return _min_distance;
}

float
CHARGING_I2C::get_maximum_distance()
{
	return _max_distance;
}

int
CHARGING_I2C::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:

			////////////////////////////////////////////////////////////////
			// CEST LE CASE OU ON VA DANS LES CONDITIONS DE BASE 
			// SANS RIEN MODIFIER DU CODE -> ON LAIISERA CA AINSI
			////////////////////////////////////////////////////////////////	
					
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(_cycling_rate);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();

					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					int ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(_cycling_rate)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
		return _reports->size();

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	case RANGEFINDERIOCSETMINIUMDISTANCE: {
			set_minimum_distance(*(float *)arg);
			return 0;
		}
		break;

	case RANGEFINDERIOCSETMAXIUMDISTANCE: {
			set_maximum_distance(*(float *)arg);
			return 0;
		}
		break;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
CHARGING_I2C::read(struct file *filp, char *buffer, size_t buflen)
{

	unsigned count = buflen / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(_cycling_rate * 2);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

int
CHARGING_I2C::measure()
{

	int ret;

	/*
	 * Send the command to begin a measurement.
	 */

	//uint8_t cmd[2];

	// TEST: on get les valeurs du topic charging_info_2 oour obtenir des donnée dans les deux sens
	///////////////////////////////////////////////////////////////////////////
	//orb_copy(ORB_ID(charging_info_2), _charging_info_sub_2, &_charging_2);

	///////////////////////////////////////////////////////////////////////////

	//cmd[0] = (uint8_t)_charging_2.manual_test;
	//cmd[1] = 0x00;

	//ret = transfer(cmd, 2, nullptr, 0);

	// pour debug
	//warn("valeur OK %d et valeur ret %d",OK,ret);

	/*
	if (OK != ret) {
		perf_count(_comms_errors);
		DEVICE_DEBUG("i2c::transfer returned %d", ret);
		return ret;
	}
	*/

	ret = OK;

	return ret;
}

int
CHARGING_I2C::collect()
{
	//la fonction transfer fonctionne ainsi (VOIR I2C_NUTTX.H)
	/**
	 * Perform an I2C transaction to the device.
	 *
	 * At least one of send_len and recv_len must be non-zero.
	 *
	 * @param send		Pointer to bytes to send.
	 * @param send_len	Number of bytes to send.
	 * @param recv		Pointer to buffer for bytes received.
	 * @param recv_len	Number of bytes to receive.
	 * @return		OK if the transfer was successful, -errno
	 *			otherwise.
	 **/

	int ret = -EIO;

	struct charging_info_s report;

	static float BAT_CAP_REG;

	static uint8_t BAT_CAP_REG_LSB;
	static uint8_t BAT_CAP_REG_MSB;

	static uint8_t BAT_FULL_CAP_REG_LSB;
	static uint8_t BAT_FULL_CAP_REG_MSB;

	static uint8_t RSENSE_REG_LSB;
	static uint8_t RSENSE_REG_MSB;

	//static uint8_t NEW_nNVCfG0_LSB;
	//static uint8_t NEW_nNVCfG0_MSB;	

	static uint16_t Current_PackCfg; // to store current configuration and modify BALCFG bits to change balancing threshold
	static uint8_t NEW_PackCfg_LSB;
	static uint8_t NEW_PackCfg_MSB;

	// update parameters for config or reading of MAX17205
	parameters_update();

	/////////////////////////////////////////////////////////////////////////////////////////
	// LECTURE EN BOUCLE DES REGISTRES PERTINENT DU MAX17205 (CHIP DE BALANCING DES CELLULES)
	/////////////////////////////////////////////////////////////////////////////////////////
	if((int)(_parameters.is_new_bat) == 0)
	{
		// on met lasresse pour faire un read des parametres (0x6C selon la chip -> 0x36 selon la nomenclature du driver)
		set_address(0x36);

		// commandes pour lire les valeurs pertinentes sur le MAX17205 (voir plus bas pour les valeurs lues)
		uint8_t cmd[11] = {0x19,0xD2,0xD3,0xD4,0x05,0x0A,0x0B,0x11,0x20,0x06,0xB8};

		// valeurs à lire sur le MAX17205
		uint8_t val1[2] = {0,0};
		uint8_t val2[2] = {0,0};
		uint8_t val3[2] = {0,0};
		uint8_t val4[2] = {0,0};
		uint8_t val5[2] = {0,0};
		uint8_t val6[2] = {0,0};
		uint8_t val7[2] = {0,0};
		uint8_t val8[2] = {0,0};
		uint8_t val9[2] = {0,0};
		uint8_t val10[2] = {0,0};

		perf_begin(_sample_perf);
		
		// lectures des registres
		ret = transfer(&cmd[0], 1, val1, 2); // envoie la commande au slave pour faire un read AvgVCell
		ret = transfer(&cmd[1], 1, val2, 2); // envoie la commande au slave pour faire un read AvgCell3
		ret = transfer(&cmd[2], 1, val3, 2); // envoie la commande au slave pour faire un read AvgCell2
		ret = transfer(&cmd[3], 1, val4, 2); // envoie la commande au slave pour faire un read AvgCell1
		ret = transfer(&cmd[4], 1, val5, 2); // envoie la commande au slave pour faire un read RepCap
		ret = transfer(&cmd[5], 1, val6, 2); // envoie la commande au slave pour faire un read Current
		ret = transfer(&cmd[6], 1, val7, 2); // envoie la commande au slave pour faire un read AvgCurrent
		ret = transfer(&cmd[7], 1, val8, 2); // envoie la commande au slave pour faire un read TTE (Time To Empty)
		ret = transfer(&cmd[8], 1, val9, 2); // envoie la commande au slave pour faire un read TTF (Time To Full)
		ret = transfer(&cmd[9], 1, val10, 2); // envoie la commande au slave pour faire un read	RepSOC

		//ret = OK;
		
		if (ret < 0) {
			DEVICE_DEBUG("error reading from sensor: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return ret;
		}

		// publish des valeurs lu sur le MAX17205 (et conditionnement des valeurs) (VOIR TABLEAU 1 PAGE 26 DE LA DATASHEET)
		report.AvgVCell = (float)(((int)val1[1] * 256) | (int)val1[0])/12800.0f;	// average cell voltage (V)
		report.AvgCell3 = (float)(((int)val2[1] * 256) | (int)val2[0])/12800.0f;	// cell 3 voltage (V)
		report.AvgCell2 = (float)(((int)val3[1] * 256) | (int)val3[0])/12800.0f;	// cell 2 voltage (V)
		report.AvgCell1 = (float)(((int)val4[1] * 256) | (int)val4[0])/12800.0f;	// cell 1 voltage (V)

		report.RepCap = (float)(((int)val5[1] * 256) | (int)val5[0])*((5.0f/1000.0f)/(_parameters.rsense / 1000.0f));		// reported capacity (mAh)
	
		report.Current = (float)(int16_t((val6[1] * 256) | val6[0]))*((1.5625f/1000.0f)/(_parameters.rsense / 1000.0f));	// current (mA)
		report.AvgCurrent = (float)(int16_t((val7[1] * 256) | val7[0]))*((1.5625f/1000.0f)/(_parameters.rsense / 1000.0f));		// average current (mA)

		report.TTE = (float)(((int)val8[1] * 256) | (int)val8[0])/640.0f;		// time to empty (hr)
		report.TTF = (float)(((int)val9[1] * 256) | (int)val9[0])/640.0f;		// time to full (hr)
		report.RepSOC = (float)(((int)val10[1] * 256) | (int)val10[0])/256.0f;		// reported state of charge (%)

	}
	/////////////////////////////////////////////////////////////////////////////////////////
	// CONFIGURATION DU MAX17205 SI LE PARAMETRE is_new_bat EST A 1
	/////////////////////////////////////////////////////////////////////////////////////////
	else if((int)(_parameters.is_new_bat) == 1)
	{
		// on met ladresse pour faire un read des parametres (0x16 selon la chip -> 0x0B selon la nomenclature du driver)

		BAT_CAP_REG = _parameters.bat_cap * 1.0f/(0.005f/(_parameters.rsense / 1000.0f));

		// on calcul les LSB et MSB a programmer selon la capacité désirée
	        BAT_CAP_REG_LSB = (uint8_t)((uint16_t)(BAT_CAP_REG) & 0x00FF);
	        BAT_CAP_REG_MSB = (uint8_t)(((uint16_t)(BAT_CAP_REG) & 0xFF00)/256.0f);

	        BAT_FULL_CAP_REG_LSB = (uint8_t)((uint16_t)(BAT_CAP_REG+0.16f*BAT_CAP_REG) & 0x00FF);
	        BAT_FULL_CAP_REG_MSB = (uint8_t)(((uint16_t)(BAT_CAP_REG+0.16f*BAT_CAP_REG) & 0xFF00)/256.0f);

	        // on calcul les LSB et MSB a programmer selon la résistance pour mesurer le courant que lon choisie
	        RSENSE_REG_LSB = (uint8_t)((uint16_t)(_parameters.rsense*100.0f) & 0x00FF);
	        RSENSE_REG_MSB = (uint8_t)(((uint16_t)(_parameters.rsense*100.0f) & 0xFF00)/256.0f);


		static int compt = 0;

		// TODO: cleaner le code
		// pour reset du fuel gauge (algorithme de la chip)
		uint8_t reset_addr2 = 0xBB;
		uint8_t reset_reg2[2] = {0x01,0x00};

		// TODO: cleaner le code
		// adresses de registre et valeurs correspondantes pour configurer le MAX17205 (3S 1000 mAh LiPo)
		uint8_t charging_reg[15] = {0xA0,0xA1,0xA2,0xA3,0xA5,0xA8,0xA9,0xB3,0xB5,0xB7,0xB8,0xB9,0xCF,0xB8,0XC8};
		// config par défault a mettre dans les registre (8 bits LSB then 8 bits MSB)
		uint8_t charging_config[26] = {0x3C,0x00,0x1B,0x80,0x0B,0x04,0x08,0x85,0x02,0x44,0xFE,0x0C,0x01,0xF4,0x01,0xF4,0x0E,0xA3,0x22,0x41,0x01,0x00,0x00,0x06,0x00,0xC8};

		perf_begin(_sample_perf);

		// premiere étape de programmation: setter chaque registre
		if(compt == 0)
		{
			// TODO: cleaner le code
			set_address(0x0B); // slave adress -> 0x16

			uint8_t val11[2] = {0,0};

			// pour configurer Cgain à la valeur désiré.... (FONCTION ENLEVÉE)
			//////////////////////////////////////////////////////////////
			/*
			ret = transfer((uint8_t*)0xB8, 1, val11, 2); // envoie la commande au slave pour faire un read-> read à nNVCfg0
			nNVCfG0 = (uint16_t)((val11[1] * 256) | val11[0]);

			nNVCfG0 = nNVCfG0 | 0x0040;

	        	NEW_nNVCfG0_LSB = (uint8_t)((uint16_t)(nNVCfG0) & 0x00FF);
	        	NEW_nNVCfG0_MSB = (uint8_t)(((uint16_t)(nNVCfG0) & 0xFF00)/256.0f); 
			*/
			//////////////////////////////////////////////////////////////

			// pour configurer le balancing threshold à la valeur désirée
			//////////////////////////////////////////////////////////////
			// 0 - 000 -> no balance
			// 1 - 001 -> 2.5 mV
			// 2 - 010 -> 5.0 mV
			// 3 - 011 -> 10.0 mV
			// 4 - 100 -> 20.0 mV
			// 5 - 101 -> 40.0 mV
			// 6 - 110 -> 80.0 mV
			// 7 - 111 -> 160.0 mV
			//////////////////////////////////////////////////////////////

			ret = transfer(&charging_reg[8], 1, val11, 2); // envoie la commande au slave pour faire un read-> read à nNVCfg0
			Current_PackCfg = (uint16_t)((val11[1] * 256) | val11[0]);

			Current_PackCfg = Current_PackCfg & 0b1111111100011111;

			if((int)(_parameters.balance_threshold) == 0)
				Current_PackCfg = Current_PackCfg | 0b0000000000000000;
			else if((int)(_parameters.balance_threshold) == 1)
				Current_PackCfg = Current_PackCfg | 0b0000000000100000;
			else if((int)(_parameters.balance_threshold) == 2)
				Current_PackCfg = Current_PackCfg | 0b0000000001000000;
			else if((int)(_parameters.balance_threshold) == 3)
				Current_PackCfg = Current_PackCfg | 0b0000000001100000;
			else if((int)(_parameters.balance_threshold) == 4)
				Current_PackCfg = Current_PackCfg | 0b0000000010000000;
			else if((int)(_parameters.balance_threshold) == 5)
				Current_PackCfg = Current_PackCfg | 0b0000000010100000;
			else if((int)(_parameters.balance_threshold) == 6)
				Current_PackCfg = Current_PackCfg | 0b0000000011000000;
			else if((int)(_parameters.balance_threshold) == 7)
				Current_PackCfg = Current_PackCfg | 0b0000000011100000;
			else
				Current_PackCfg = Current_PackCfg | 0b0000000010000000; // default blancing threshold to 20 mV if user value is not valid

	        	NEW_PackCfg_LSB = (uint8_t)((uint16_t)(Current_PackCfg) & 0x00FF);
	        	NEW_PackCfg_MSB = (uint8_t)(((uint16_t)(Current_PackCfg) & 0xFF00)/256.0f);

			//////////////////////////////////////////////////////////////

			uint8_t test11[3] = {charging_reg[0],charging_config[1],charging_config[0]};
			uint8_t test12[3] = {charging_reg[1],charging_config[3],charging_config[2]};
			uint8_t test13[3] = {charging_reg[2],charging_config[5],charging_config[4]};
			uint8_t test14[3] = {charging_reg[3],charging_config[7],charging_config[6]};
			uint8_t test15[3] = {charging_reg[4],BAT_FULL_CAP_REG_LSB,BAT_FULL_CAP_REG_MSB};
			uint8_t test16[3] = {charging_reg[5],charging_config[11],charging_config[10]};
			uint8_t test17[3] = {charging_reg[6],BAT_CAP_REG_LSB,BAT_CAP_REG_MSB};
			uint8_t test18[3] = {charging_reg[7],BAT_CAP_REG_LSB,BAT_CAP_REG_MSB};
			uint8_t test19[3] = {charging_reg[8],charging_config[17],charging_config[16]};
			uint8_t test110[3] = {charging_reg[9],charging_config[19],charging_config[18]};
			uint8_t test111[3] = {charging_reg[10],charging_config[21],charging_config[20]};
			uint8_t test112[3] = {charging_reg[11],charging_config[23],charging_config[22]};
			uint8_t test113[3] = {charging_reg[12],RSENSE_REG_LSB,RSENSE_REG_MSB};
			//uint8_t test114[3] = {charging_reg[13],NEW_nNVCfG0_LSB,NEW_nNVCfG0_MSB};
			//uint8_t test115[3] = {charging_reg[14],0x00,0x20};
			uint8_t test116[3] = {charging_reg[8],NEW_PackCfg_LSB,NEW_PackCfg_MSB};


			// configure registre pour la capacité de la battery
			ret = transfer(test11, 3, nullptr, 0); // envoie l'adresse du registre à configurer
			ret = transfer(test12, 3, nullptr, 0); // envoie l'adresse du registre à configurer
			ret = transfer(test13, 3, nullptr, 0); // envoie l'adresse du registre à configurer
			ret = transfer(test14, 3, nullptr, 0); // envoie l'adresse du registre à configurer
			ret = transfer(test15, 3, nullptr, 0); // envoie l'adresse du registre à configurer
			ret = transfer(test16, 3, nullptr, 0); // envoie l'adresse du registre à configurer
			ret = transfer(test17, 3, nullptr, 0); // envoie l'adresse du registre à configurer
			ret = transfer(test18, 3, nullptr, 0); // envoie l'adresse du registre à configurer
			ret = transfer(test19, 3, nullptr, 0); // envoie l'adresse du registre à configurer
			ret = transfer(test110, 3, nullptr, 0); // envoie l'adresse du registre à configurer
			ret = transfer(test111, 3, nullptr, 0); // envoie l'adresse du registre à configurer
			ret = transfer(test112, 3, nullptr, 0); // envoie l'adresse du registre à configurer
			ret = transfer(test113, 3, nullptr, 0); // envoie l'adresse du registre à configurer
			//ret = transfer(test115, 3, nullptr, 0); // envoie l'adresse du registre à configurer
			//ret = transfer(test114, 3, nullptr, 0); // envoie l'adresse du registre à configurer
			ret = transfer(test116, 3, nullptr, 0); // envoie l'adresse du registre à configurer
		}		

		// deuxieme étape de programmation: reset model gauge (so change take effect)
		else if(compt == 4) // ON SET LE COMPTEUR POUR QUE LES REGISTRES SOIENT CORRECTEMENT TRANSFÉRÉ AVANT DE RESETTER LE CHIP
		{
			set_address(0x36); //slave adress -> 0x6C

			uint8_t test33[3] = {reset_addr2,reset_reg2[0],reset_reg2[1]};

            if(test33[2]==0)
            {
                test33[2]=test33[2];
            }
			// fuel gauge model reset
			ret = transfer(test33, 3, nullptr, 0); // envoie l'adresse du registre à configurer
		}

		if(compt++ > 4)
		{
			compt = 0;
            //float value = 0.0;
            //param_set(_parameters.is_new_bat,(void*)&value);
		}

		if (ret < 0) {
			DEVICE_DEBUG("error reading from sensor: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return ret;
		}

		// publish des valeurs lu sur le MAX17205 (et conditionnement des valeurs) (VOIR TABLEAU 1 PAGE 26 DE LA DATASHEET)
		//struct charging_info_s report;

		report.AvgVCell = 1;
		report.AvgCell3 = 1;
		report.AvgCell2 = 1;
		report.AvgCell1 = 1;
		report.RepCap = 1;
		report.Current = 1;
		report.AvgCurrent = 1;
		report.TTE = 1;
		report.TTF = 1;
		report.RepSOC = 1;

	}
	/////////////////////////////////////////////////////////////////////////////////////////
	// PARAMETRE is_new_batt NON VALIDE (on)
	/////////////////////////////////////////////////////////////////////////////////////////
	else
	{
		perf_begin(_sample_perf);

		// publish des valeurs lu sur le MAX17205 (et conditionnement des valeurs) (VOIR TABLEAU 1 PAGE 26 DE LA DATASHEET)
		//struct charging_info_s report;
		report.AvgVCell = 0;
		report.AvgCell3 = 0;
		report.AvgCell2 = 0;
		report.AvgCell1 = 0;
		report.RepCap = 0;
		report.Current = 0;
		report.AvgCurrent = 0;
		report.TTE = 0;
		report.TTF = 0;
		report.RepSOC = 0;
	}
											
	/* publish it, if we are the primary */
	if (_charging_info_topic != nullptr) {
		orb_publish(ORB_ID(charging_info), _charging_info_topic, &report);
	}

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
CHARGING_I2C::start()
{

	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	// tente de subsriber au topic du charging_info car des info sont envoyer dans 2 sens
	// TODO: tester cela et si marche pas, créer un nouveau topic pour recevoir des infos et non les envoyer
	_charging_info_sub_2 = orb_subscribe(ORB_ID(charging_info_2));

	warn("fonction start work_queue");

	/* schedule a cycle to start things */
	//
	work_queue(HPWORK, &_work, (worker_t)&CHARGING_I2C::cycle_trampoline, this, 5);

	/* notify about state change */
	struct subsystem_info_s info = {};
	info.present = true;
	info.enabled = true;
	info.ok = true;
	info.subsystem_type = subsystem_info_s::SUBSYSTEM_TYPE_RANGEFINDER;

	static orb_advert_t pub = nullptr;

	if (pub != nullptr) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);

	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);

	}
}

void
CHARGING_I2C::stop()
{
	work_cancel(HPWORK, &_work);
}

void
CHARGING_I2C::cycle_trampoline(void *arg)
{

	warn("fonction cycle trampoline");
	CHARGING_I2C *dev = (CHARGING_I2C *)arg;

	dev->cycle();

}

void
CHARGING_I2C::cycle()
{

	if (_collect_phase) {
		_index_counter = addr_ind[_cycle_counter]; /*sonar from previous iteration collect is now read out */
		set_address(_index_counter);

		/* perform collection */
		if (OK != collect()) {
			DEVICE_DEBUG("collection error");
			/* if error restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = false;

		/* change i2c adress to next sonar */
		_cycle_counter = _cycle_counter + 1;

		if (_cycle_counter >= addr_ind.size()) {
			_cycle_counter = 0;
		}

		/* Is there a collect->measure gap? Yes, and the timing is set equal to the cycling_rate
		   Otherwise the next sonar would fire without the first one having received its reflected sonar pulse */

		if (_measure_ticks > USEC2TICK(_cycling_rate)) {

			// test debug
			//warn("condition 1? (debug)");
			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK,
				   &_work,
				   (worker_t)&CHARGING_I2C::cycle_trampoline,
				   this,
				   _measure_ticks - USEC2TICK(_cycling_rate));
			return;
		}
	}

	/* Measurement (firing) phase */

	/* ensure sonar i2c adress is still correct */
	_index_counter = addr_ind[_cycle_counter];
	set_address(_index_counter);

	/* Perform measurement */
	if (OK != measure()) {
		DEVICE_DEBUG("measure error sonar adress %d", _index_counter);
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	// test debug
	//warn("condition 2? (debug)");
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&CHARGING_I2C::cycle_trampoline,
		   this,
		   USEC2TICK(_cycling_rate));

}

void
CHARGING_I2C::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");

    //uint8_t charging_reg[15] = {0xA0,0xA1,0xA2,0xA3,0xA5,0xA8,0xA9,0xB3,0xB5,0xB7,0xB8,0xB9,0xCF,0xB8,0XC8};
    int ret;
    set_address(0x0B); // slave adress -> 0x16
    uint8_t nQRTable00 = 0xA0;
    uint8_t nQRTable10 = 0xA1;
    uint8_t nQRTable20 = 0xA2;
    uint8_t nQRTable30 = 0xA3;
    uint8_t nFullCapNom = 0xA5;
    uint8_t nlAvgEmpty = 0xA8;
    uint8_t nFullCapRep = 0xA9;
    uint8_t nDesignCap = 0xB3;
    uint8_t nPackCfg = 0xB5;
    uint8_t nConvgCfg = 0xB7;
    uint8_t nNVCfg0 = 0xB8;
    uint8_t AgeForecast = 0xB9;
    uint8_t nRSense = 0xCF;

    uint8_t val_nQRTable00[2] = {0,0};
    uint8_t val_nQRTable10[2] = {0,0};
    uint8_t val_nQRTable20[2] = {0,0};
    uint8_t val_nQRTable30[2] = {0,0};
    uint8_t val_nFullCapNom[2] = {0,0};
    uint8_t val_nlAvgEmpty[2] = {0,0};
    uint8_t val_nFullCapRep[2] = {0,0};
    uint8_t val_nDesignCap[2] = {0,0};
    uint8_t val_nPackCfg[2] = {0,0};
    uint8_t val_nConvgCfg[2] = {0,0};
    uint8_t val_nNVCfg0[2] = {0,0};
    uint8_t val_AgeForecast[2] ={0,0};
    uint8_t val_nRSense[2] = {0,0};

    ret = transfer(&nPackCfg, 1, val_nPackCfg, 2);
    ret = transfer(&nQRTable00, 1, val_nQRTable00, 2);
    ret = transfer(&nQRTable10, 1, val_nQRTable10, 2);
    ret = transfer(&nQRTable20, 1, val_nQRTable20, 2);
    ret = transfer(&nQRTable30, 1, val_nQRTable30, 2);
    ret = transfer(&nFullCapNom, 1, val_nFullCapNom, 2);
    ret = transfer(&nlAvgEmpty, 1, val_nlAvgEmpty, 2);
    ret = transfer(&nFullCapRep, 1, val_nFullCapRep, 2);
    ret = transfer(&nDesignCap, 1, val_nDesignCap, 2);
    ret = transfer(&nPackCfg, 1, val_nPackCfg, 2);
    ret = transfer(&nConvgCfg, 1, val_nConvgCfg, 2);
    ret = transfer(&nNVCfg0, 1, val_nNVCfg0, 2);
    ret = transfer(&AgeForecast, 1, val_AgeForecast, 2);
    ret = transfer(&nRSense, 1, val_nRSense, 2);
    if (ret < 0) {
        errx(1, "timed out waiting for sensor data");
    }

    printf("nPackCfg value: %x %x \n", val_nPackCfg[1], val_nPackCfg[0]);
    printf("nQRTable00 value: %x %x \n", val_nQRTable00[1], val_nQRTable00[0]);
    printf("val_nQRTable10 value: %x %x \n", val_nQRTable10[1], val_nQRTable10[0]);
    printf("val_nQRTable20 value: %x %x \n", val_nQRTable20[1], val_nQRTable20[0]);
    printf("val_nQRTable30 value: %x %x \n", val_nQRTable30[1], val_nQRTable30[0]);
    printf("val_nFullCapNom value: %x %x \n", val_nFullCapNom[1], val_nFullCapNom[0]);
    printf("val_nlAvgEmpty value: %x %x \n", val_nlAvgEmpty[1], val_nlAvgEmpty[0]);
    printf("val_nFullCapRep value: %x %x \n", val_nFullCapRep[1], val_nFullCapRep[0]);
    printf("val_nDesignCap value: %x %x \n", val_nDesignCap[1], val_nDesignCap[0]);
    printf("val_nPackCfg value: %x %x \n", val_nPackCfg[1], val_nPackCfg[0]);
    printf("val_nConvgCfg value: %x %x \n", val_nConvgCfg[1], val_nConvgCfg[0]);
    printf("val_nNVCfg0 value: %x %x \n", val_nNVCfg0[1], val_nNVCfg0[0]);
    printf("val_AgeForecast value: %x %x \n", val_AgeForecast[1], val_AgeForecast[0]);
    printf("val_nNVCfg0 value: %x %x \n", val_nRSense[1], val_nRSense[0]);


}

/**
 * Local functions in support of the shell command
 * Pour appeler les fonctions on écrit:
 * charging_i2c start
 * charging_i2c stop
 * charging_i2c test
 * charging_i2c reset
 * charging_i2c info
  */
namespace  charging_i2c
{

CHARGING_I2C	*g_dev;

void	start();
void	stop();
void	test();
void	reset();
void	info();

/**
 * Start the driver.
 */
void
start()
{
	int fd;

	if (g_dev != nullptr) {
		errx(1, "already started");
	}

	/////////////////////////////////////////////////////////////////////////////////////
	/* create the driver */
	// constructor de la classe fille appelle le contructor de la fonction mere
	/////////////////////////////////////////////////////////////////////////////////////	
	g_dev = new CHARGING_I2C(SRF02_I2C_BUS);

	if (g_dev == nullptr) {
		
		warnx("1"); // debug erreur
		goto fail;
	}

	/////////////////////////////////////////////////////////////////////////////////////	
	// fonction init dans la classe fille (charging_i2c)
	// init fait appel aussi a init() de la classe mere. on set plusieurs chose:
	// - vitesse de bus
	// - probe() pour tester la presence du slave
	// - pour le capteur sonar a partir duquelon developpe le driver, fait un scan dadresse pour detecter plusieur capteurs
	// - initialise le UORB (a changer pour utiliser notre topic custom)
	// - prend la frequence de poll spécifique pour le capteur (sur loscilloscope semble etre 10 Hz)
	// TODO: modifier cette fonction pour effectuer la configuration des registres dans le chip de balancing
	/////////////////////////////////////////////////////////////////////////////////////	
	if (OK != g_dev->init()) {
		
		warnx("2"); // debug erreur		
		goto fail;
	}

	/////////////////////////////////////////////////////////////////////////////////////
	/* set the poll rate to default, starts automatic data collection */
	// ne comprend pas trop: si on va dans nsh SRF02_DEVICE_PATH (quon a setter a dev/charging_i2c )
	// apparait lorsque lon fait charging_i2c start. on uvre donc le peripherique qui fait mainteant partie de la liste
	// de tout les peripherique connecté au pixhawk, mais est-ce vraiment utile ?? -> a voir
	/////////////////////////////////////////////////////////////////////////////////////	
	fd = open(SRF02_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		
		warnx("3"); // debug erreur		
		goto fail;
	}	
	
	/////////////////////////////////////////////////////////////////////////////////////
	// fonction qui permet deffectuer lechantillonnage du capteur a la frequence desirée
	// TODO: determiner comment cette fonction permet dappeler les fonction de mesure en boucle
	// et changer ces fonction pour notre application
	// NOTE: selon les tests avec arduino et oscilloscope on semble appeler ::collect et apres :: measure
	// appelle la fonction charging_i2c::start selon certain case lié au parametres 
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		
		warnx("4"); // debug erreur		
		goto fail;
	}

	warn("function start executed completely");
	exit(0);

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	errx(1, "driver start failed");
}

/**
 * Stop the driver
 */
void stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		errx(1, "driver not running");
	}

	exit(0);
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
void
test()
{
	struct distance_sensor_s report;
	ssize_t sz;
	int ret;

	int fd = open(SRF02_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'srf02_i2c start' if the driver is not running", SRF02_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("measurement: %0.2f m", (double)report.current_distance);
	warnx("time:        %llu", report.timestamp);

	/* start the sensor polling at 2Hz */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		errx(1, "failed to set 2Hz poll rate");
	}

	/* read the sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;

		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 2000);

		if (ret != 1) {
			errx(1, "timed out waiting for sensor data");
		}

		/* now go get it */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			err(1, "periodic read failed");
		}

		warnx("periodic read %u", i);
		warnx("valid %u", (float)report.current_distance > report.min_distance
		      && (float)report.current_distance < report.max_distance ? 1 : 0);
		warnx("measurement: %0.3f", (double)report.current_distance);
		warnx("time:        %llu", report.timestamp);
	}

	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		errx(1, "failed to set default poll rate");
	}

	errx(0, "PASS");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(SRF02_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "failed ");
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		err(1, "driver reset failed");
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		err(1, "driver poll restart failed");
	}

	exit(0);
}

/**
 * Print a little info about the driver.
 */
void
info()
{
	if (g_dev == nullptr) {
		errx(1, "driver not running");
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();


	exit(0);
}

} /* namespace */

int
charging_i2c_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		charging_i2c::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		charging_i2c::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		charging_i2c::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		charging_i2c::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		charging_i2c::info();
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
