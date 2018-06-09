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
 * @file srf02_i2c.cpp
 * @author Greg Hulands
 * @author Jon Verbeke <jon.verbeke@kuleuven.be>
 *
 * Driver for the Maxbotix sonar range finders connected via I2C.
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
//#include <uORB/topics/charging_info.h>
//#include <uORB/topics/charging_info_2.h>
#include <uORB/topics/wake_up_slave_info.h>
#include <uORB/topics/wake_up_slave_info_2.h>

#include <board_config.h>

/* Configuration Constants */

// JASSUME QUE CEST LI2C SUR LE CONECTEUR EXTERNE: À VALIDER !!!
#define SRF02_I2C_BUS 		PX4_I2C_BUS_ONBOARD

// LADRESSE -> VALIDER CE QUE LON VEUT
#define SRF02_I2C_BASEADDR 	0x12 /* 7-bit address. 8-bit address is 0xE0 */

// LADRESSE -> PATH DU DEVICE DANS LOS NuttX ??? (PAS CERTAIN)
#define SRF02_DEVICE_PATH	"/dev/wake_up_i2c_slave"

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

class WAKE_UP_I2C_SLAVE : public device::I2C
{
public:
	WAKE_UP_I2C_SLAVE(int bus = SRF02_I2C_BUS, int address = SRF02_I2C_BASEADDR);
	// la declaration de la fonction definit deja le BUS et lADRESS

	virtual ~WAKE_UP_I2C_SLAVE();

	virtual int 		init();

	virtual ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	virtual int			ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				print_info();
    void                transferI2C(const uint8_t* sendData,uint8_t sendDataSize, uint8_t* receivedData, uint8_t receivedDataSize);

protected:
	virtual int			probe();

private:

	///////////////////////////////////////////////////////////////////////////////////
	// TEST: UNE COPY LOCALE DE PARAMETRES CUSTOM POUR LE DRIVER CHARGING_I2C

	
	struct {

		float nb_sec_dodo;
		float test_dodo;

	}	_parameters;			
	

	struct {

        	param_t nb_sec_dodo;
        	param_t test_dodo;

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
	orb_advert_t     	_wake_up_slave_info_topic;

	// pour subsrcibe au topic du charging_info (il y a des info que lon veut recevoir)
	int			_wake_up_salve_info_sub_2;
	struct wake_up_slave_info_2_s		_wake_up_slave_2;

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
extern "C" { __EXPORT int wake_up_i2c_slave_main(int argc, char *argv[]);}

WAKE_UP_I2C_SLAVE::WAKE_UP_I2C_SLAVE(int bus, int address) :
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
	_wake_up_slave_info_topic(nullptr),	// charging_i2c custom topic
	_sample_perf(perf_alloc(PC_ELAPSED, "srf02_i2c_read")),
	_comms_errors(perf_alloc(PC_COUNT, "srf02_i2c_comms_errors")),
	_cycle_counter(0),	/* initialising counter for cycling function to zero */
	_cycling_rate(0),	/* initialising cycling rate (which can differ depending on one sonar or multiple) */
	_index_counter(0) 	/* initialising temp sonar i2c address to zero */

{
	_parameter_handles.nb_sec_dodo = param_find("TEMPS_DODO_SEC");
	_parameter_handles.test_dodo = param_find("TEST_MODE_DODO");

  	parameters_update();
	/* enable debug() calls */
	_debug_enabled = false;

	/* work_cancel in the dtor will explode if we don't do this... */
	memset(&_work, 0, sizeof(_work));
}

// constructor utilse initialisation list. cest plus performant...

WAKE_UP_I2C_SLAVE::~WAKE_UP_I2C_SLAVE()
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
WAKE_UP_I2C_SLAVE::parameters_update()
{

	param_get(_parameter_handles.nb_sec_dodo, &_parameters.nb_sec_dodo);
	param_get(_parameter_handles.test_dodo, &_parameters.test_dodo);

        return OK;
}

int
WAKE_UP_I2C_SLAVE::init()
{
	int ret = PX4_ERROR; // -1 -> valeur standard

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		warn("I2C init failed (debug)");
		return ret;
	}

	/* allocate basic report buffers */
	//_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));
	//_reports = new ringbuffer::RingBuffer(2, sizeof(charging_info_s));
	_reports = new ringbuffer::RingBuffer(2, sizeof(wake_up_slave_info_s));

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
	//struct charging_info_s ds_report = {};
	struct wake_up_slave_info_s ds_report = {};

	// TODO: changer de topic pour le nouveau
	//_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
	//			 &_orb_class_instance, ORB_PRIO_LOW);

	//_charging_info_topic = orb_advertise_multi(ORB_ID(charging_info), &ds_report,
	//			 &_orb_class_instance, ORB_PRIO_LOW);

	_wake_up_slave_info_topic = orb_advertise_multi(ORB_ID(wake_up_slave_info), &ds_report,
				 &_orb_class_instance, ORB_PRIO_LOW);

	if (_wake_up_slave_info_topic == nullptr) {
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
WAKE_UP_I2C_SLAVE::probe()
{
	warnx("probe class charging i2c (debug)"); // debug erreur
	// measure() dans la classe charging I2C
	return measure();
}

void
WAKE_UP_I2C_SLAVE::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
WAKE_UP_I2C_SLAVE::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
WAKE_UP_I2C_SLAVE::get_minimum_distance()
{
	return _min_distance;
}

float
WAKE_UP_I2C_SLAVE::get_maximum_distance()
{
	return _max_distance;
}

int
WAKE_UP_I2C_SLAVE::ioctl(struct file *filp, int cmd, unsigned long arg)
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
WAKE_UP_I2C_SLAVE::read(struct file *filp, char *buffer, size_t buflen)
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
WAKE_UP_I2C_SLAVE::measure()
{


	int ret;
/*
	uint8_t cmd[2];
	

	// TEST: on get les valeurs du topic charging_info_2 oour obtenir des donnée dans les deux sens
	///////////////////////////////////////////////////////////////////////////
	orb_copy(ORB_ID(wake_up_slave_info_2), _wake_up_salve_info_sub_2, &_wake_up_slave_2);

	///////////////////////////////////////////////////////////////////////////

	

	cmd[0] = (uint8_t)_wake_up_slave_2.manual_test;
	cmd[1] = 0x00;

	ret = transfer(cmd, 2, nullptr, 0);

	// pour debug
	warn("valeur OK %d et valeur ret %d",OK,ret);

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
WAKE_UP_I2C_SLAVE::collect()
{
	int	ret = -EIO;

	static uint8_t temps_dodo_MSB_01 = 0;
	static uint8_t temps_dodo_MSB_02 = 0;
	static uint8_t temps_dodo_LSB_01 = 0;
	static uint8_t temps_dodo_LSB_02 = 0;

	static int flag_dodo = 0;

	// update parameters for config or reading of MAX17205
	parameters_update();

	if((int)_parameters.test_dodo == 1 && flag_dodo == 0)
	{

		flag_dodo = 1;
		_parameters.test_dodo = 0.0f;
		param_set(_parameter_handles.test_dodo,&_parameters.test_dodo);

		temps_dodo_MSB_01 = (uint8_t)(((uint32_t)_parameters.nb_sec_dodo / 16777216) & 0x000000FF); // shift a droite de 24 bit
		temps_dodo_MSB_02 = (uint8_t)(((uint32_t)_parameters.nb_sec_dodo / 65536) & 0x000000FF); // shift a droite de 24 bit
		temps_dodo_LSB_01 = (uint8_t)(((uint32_t)_parameters.nb_sec_dodo / 256) & 0x000000FF); // shift a droite de 24 bit
		temps_dodo_LSB_02 = (uint8_t)(((uint32_t)_parameters.nb_sec_dodo / 1) & 0x000000FF); // shift a droite de 24 bit

		/* read from the sensor */
		uint8_t test1[5] = {0x31,temps_dodo_MSB_01,temps_dodo_MSB_02,temps_dodo_LSB_01,temps_dodo_LSB_02};

		perf_begin(_sample_perf);

		ret = transfer(test1, 5, nullptr, 0); // envoie l'adresse du registre à configurer

		if (ret < 0) {
			DEVICE_DEBUG("error reading from sensor: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return ret;
		}
	}
	else if((int)_parameters.test_dodo == 0 && flag_dodo == 1)
	{
		flag_dodo = 0;
	}
	else
	{

	}
	
	/*
	struct distance_sensor_s report;
	report.timestamp = hrt_absolute_time();
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
	report.orientation = 8;
	report.current_distance = distance_m;
	report.min_distance = get_minimum_distance();
	report.max_distance = get_maximum_distance();
	report.covariance = 0.0f;
	 //TODO: set proper ID 
	report.id = 0;
	*/

	// TEST: on get les valeurs du topic charging_i2c pour obtenir des donnée dans les deux sens
	///////////////////////////////////////////////////////////////////////////

	///////////////////////////////////////////////////////////////////////////

	/*
	// test de publish de valeur custom
	struct wake_up_slave_info_s report;
	report.var_test1 = (int)val[0];
	report.var_test2 = (int)val[1];
	report.timestamp = hrt_absolute_time();
	//report.manual_test = report.manual_test;

	//TODO: publish pour notre topic lier au module de balancement
	// publish it, if we are the primary 
	if (_wake_up_slave_info_topic != nullptr) {
		orb_publish(ORB_ID(wake_up_slave_info), _wake_up_slave_info_topic, &report);
	}
	*/

	//_reports->force(&report);

	/* notify anyone waiting for data */
	//poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
WAKE_UP_I2C_SLAVE::start()
{

	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	// tente de subsriber au topic du charging_info car des info sont envoyer dans 2 sens
	// TODO: tester cela et si marche pas, créer un nouveau topic pour recevoir des infos et non les envoyer
	_wake_up_salve_info_sub_2 = orb_subscribe(ORB_ID(wake_up_slave_info_2));

	warn("fonction start work_queue");

	/* schedule a cycle to start things */
	//
	work_queue(HPWORK, &_work, (worker_t)&WAKE_UP_I2C_SLAVE::cycle_trampoline, this, 5);

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
WAKE_UP_I2C_SLAVE::stop()
{
	work_cancel(HPWORK, &_work);
}

void
WAKE_UP_I2C_SLAVE::cycle_trampoline(void *arg)
{

	warn("fonction cycle trampoline");
	WAKE_UP_I2C_SLAVE *dev = (WAKE_UP_I2C_SLAVE *)arg;

	dev->cycle();

}

void
WAKE_UP_I2C_SLAVE::cycle()
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
				   (worker_t)&WAKE_UP_I2C_SLAVE::cycle_trampoline,
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
		   (worker_t)&WAKE_UP_I2C_SLAVE::cycle_trampoline,
		   this,
		   USEC2TICK(_cycling_rate));

}

void
WAKE_UP_I2C_SLAVE::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

void
WAKE_UP_I2C_SLAVE::transferI2C(const uint8_t* sendData,uint8_t sendDataSize, uint8_t* receivedData, uint8_t receivedDataSize)
{
    transfer(sendData,sendDataSize,receivedData,receivedDataSize);
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
namespace  wake_up_i2c_slave
{

WAKE_UP_I2C_SLAVE	*g_dev;

void	start();
void	stop();
void	test();
void	reset();
void	info();
void    sleep(uint32_t time);

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
	g_dev = new WAKE_UP_I2C_SLAVE(SRF02_I2C_BUS);

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

void
sleep(uint32_t time)
{
    uint8_t temps_dodo_MSB_01 = 0;
    uint8_t temps_dodo_MSB_02 = 0;
    uint8_t temps_dodo_LSB_01 = 0;
    uint8_t temps_dodo_LSB_02 = 0;

    temps_dodo_MSB_01 = (uint8_t)((time / 16777216) & 0x000000FF); // shift a droite de 24 bit
    temps_dodo_MSB_02 = (uint8_t)((time / 65536) & 0x000000FF); // shift a droite de 24 bit
    temps_dodo_LSB_01 = (uint8_t)((time / 256) & 0x000000FF); // shift a droite de 24 bit
    temps_dodo_LSB_02 = (uint8_t)((time / 1) & 0x000000FF); // shift a droite de 24 bit

    /* read from the sensor */
    uint8_t test1[5] = {0x31,temps_dodo_MSB_01,temps_dodo_MSB_02,temps_dodo_LSB_01,temps_dodo_LSB_02};


    g_dev->transferI2C(test1, 5, nullptr, 0); // envoie l'adresse du registre à configurer

    exit(0);

}

} /* namespace */

int
wake_up_i2c_slave_main(int argc, char *argv[])
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		wake_up_i2c_slave::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		wake_up_i2c_slave::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		wake_up_i2c_slave::test();
	}

    if (!strcmp(argv[1], "sleep")) {
        wake_up_i2c_slave::sleep(atoi(argv[2]));
    }

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		wake_up_i2c_slave::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		wake_up_i2c_slave::info();
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset', 'sleep' or 'info'");
}
