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

#include <perf/perf_counter.h>
#include <systemlib/err.h>

#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/go_to_sleep.h>

#include <board_config.h>

/* Configuration Constants */

#define GO_SLEEP_I2C_BUS 		PX4_I2C_BUS_ONBOARD
#define GO_SLEEP_I2C_BASEADDR 	0x12 /* 7-bit address. 8-bit address is 0xE0 */
#define GO_SLEEP_MESS_OVHD		0x31 // Overhead code for I2C slave device
#define GO_SLEEP_DEVICE_PATH	"/dev/wake_up_i2c_slave"


// le timing entre chaque mesure est 60 ms. la collecte automatique a cette frequence
// est partie dans la fonction ioctl
#define GO_SLEEP_CONVERSION_INTERVAL 	100000 /* 60ms for one sonar */
#define TICKS_BETWEEN_SUCCESIVE_FIRES 	100000 /* 30ms between each sonar measurement (watch out for interference!) */

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class WAKE_UP_I2C_SLAVE : public device::I2C
{
public:
	WAKE_UP_I2C_SLAVE(int bus = GO_SLEEP_I2C_BUS, int address = GO_SLEEP_I2C_BASEADDR);
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
	
	struct {

		int32_t nb_sec_sleep;
		float test_sleep;

	}	_parameters;			
	

	struct {

		param_t nb_sec_sleep;
		param_t test_sleep;

	}	_parameter_handles;		

	struct go_to_sleep_s _go_sleep_s  {};
	
	///////////////////////////////////////////////////////////////////////////////////

	work_s				_work;
	ringbuffer::RingBuffer		*_reports;
	bool				_sensor_ok;
	int					_measure_ticks;
	bool				_collect_phase;
	int					_class_instance;
	int					_orb_class_instance;
	int 				_sub_go_sleep;
	

	// pour subsrcibe au topic du charging_info (il y a des info que lon veut recevoir)

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	uint8_t				_cycle_counter;	/* counter in cycle to change i2c adresses */
	int					_cycling_rate;	/* */

	/**
	 * Update our local parameter cache.
	 */
	int					parameters_update();


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
	I2C("MB12xx", GO_SLEEP_DEVICE_PATH, bus, address, 100000), // _address est private on appelle donc le constructor de la classe mere
								// la frequence du bus est à 100 khz ***
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "go_sleep_i2c_read")),
	_comms_errors(perf_alloc(PC_COUNT, "go_sleep_i2c_comms_errors")),
	_cycle_counter(0),	/* initialising counter for cycling function to zero */
	_cycling_rate(0)	/* initialising cycling rate (which can differ depending on one sonar or multiple) */

{
	_parameter_handles.nb_sec_sleep = param_find("TEMPS_DODO_SEC");
	_parameter_handles.test_sleep = param_find("TEST_MODE_DODO");

	_sub_go_sleep = orb_subscribe(ORB_ID(go_to_sleep));

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
		unregister_class_devname(GO_SLEEP_DEVICE_PATH, _class_instance);
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

	param_get(_parameter_handles.nb_sec_sleep, &_parameters.nb_sec_sleep);
	param_get(_parameter_handles.test_sleep, &_parameters.test_sleep);

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
	_reports = new ringbuffer::RingBuffer(2, sizeof(go_to_sleep_s));

	// TODO: a quoi ca sert?
	set_device_address(GO_SLEEP_I2C_BASEADDR);		/* set I2c port to temp sonar i2c adress */

	if (_reports == nullptr) {
		return ret;
	}

	// TODO: a quoi ca sert?
	_class_instance = register_class_devname(GO_SLEEP_DEVICE_PATH);

	// XXX we should find out why we need to wait 200 ms here
	usleep(200000);

	set_device_address(GO_SLEEP_I2C_BASEADDR); /* set i2c port back to base adress for rest of driver */

	// cycling rate utilisé (dans notre cas, il y a un seul capteur detecté)
	/* if only one sonar detected, no special timing is required between firing, so use default */

	_cycling_rate = GO_SLEEP_CONVERSION_INTERVAL;

	DEVICE_DEBUG("Wake up I2c slave drivers connected");

	ret = OK;
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


	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
WAKE_UP_I2C_SLAVE::read(struct file *filp, char *buffer, size_t buflen)
{
	int ret = OK;

	return ret;
}

int
WAKE_UP_I2C_SLAVE::measure()
{
	int ret = OK;

	return ret;
}

int
WAKE_UP_I2C_SLAVE::collect()
{
	int	ret = -EIO;

	static uint8_t sleep_MSB_01 = 0;
	static uint8_t sleep_MSB_02 = 0;
	static uint8_t sleep_LSB_01 = 0;
	static uint8_t sleep_LSB_02 = 0;

	// update parameters for config or reading
	parameters_update();

	// the value can be changed in mission.cpp
	memset(&_go_sleep_s, 0, sizeof(_go_sleep_s));
	orb_copy(ORB_ID(go_to_sleep), _sub_go_sleep, &_go_sleep_s);

	// Sleep request coming from a parameter change
	if((int)_parameters.test_sleep == 1)
	{
		_parameters.test_sleep = 0.0f;
		param_set(_parameter_handles.test_sleep,&_parameters.test_sleep);
		sleep(1);

		sleep_MSB_01 = (uint8_t)((_parameters.nb_sec_sleep >> 24) & 0x000000FF); 
		sleep_MSB_02 = (uint8_t)((_parameters.nb_sec_sleep >> 16) & 0x000000FF); 
		sleep_LSB_01 = (uint8_t)((_parameters.nb_sec_sleep >> 8) & 0x000000FF); 
		sleep_LSB_02 = (uint8_t)(_parameters.nb_sec_sleep & 0x000000FF); 


		uint8_t test1[5] = {GO_SLEEP_MESS_OVHD,sleep_MSB_01,sleep_MSB_02,sleep_LSB_01,sleep_LSB_02};

		perf_begin(_sample_perf);

		ret = transfer(test1, 5, nullptr, 0); // envoie l'adresse du registre à configurer

		if (ret < 0) {
			DEVICE_DEBUG("error reading from sensor: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return ret;
		}
	}
	// Sleep request coming from a uorb message
	else if(_go_sleep_s.sleep)
	{
		sleep_MSB_01 = (uint8_t)(((uint32_t)_go_sleep_s.sleep_time_ms >> 24) & 0x000000FF); 
		sleep_MSB_02 = (uint8_t)(((uint32_t)_go_sleep_s.sleep_time_ms >> 16) & 0x000000FF); 
		sleep_LSB_01 = (uint8_t)(((uint32_t)_go_sleep_s.sleep_time_ms >> 8) & 0x000000FF); 
		sleep_LSB_02 = (uint8_t)(((uint32_t)_go_sleep_s.sleep_time_ms) & 0x000000FF); 

		uint8_t test1[5] = {GO_SLEEP_MESS_OVHD,sleep_MSB_01,sleep_MSB_02,sleep_LSB_01,sleep_LSB_02};

		perf_begin(_sample_perf);

		ret = transfer(test1, 5, nullptr, 0); // envoie l'adresse du registre à configurer

		if (ret < 0) {
			DEVICE_DEBUG("error reading from sensor: %d", ret);
			perf_count(_comms_errors);
			perf_end(_sample_perf);
			return ret;
		}
	}
	
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

	_sub_go_sleep = orb_subscribe(ORB_ID(go_to_sleep));

	warn("fonction start work_queue");

	/* schedule a cycle to start things */
	//
	work_queue(HPWORK, &_work, (worker_t)&WAKE_UP_I2C_SLAVE::cycle_trampoline, this, 5);

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
		set_device_address(GO_SLEEP_I2C_BASEADDR);

		/* perform collection */
		if (OK != collect()) {
			DEVICE_DEBUG("collection error");
			/* if error restart the measurement state machine */
			start();
			return;
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


	set_device_address(GO_SLEEP_I2C_BASEADDR);

	/* Perform measurement */
	if (OK != measure()) {
		DEVICE_DEBUG("measure error");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
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

	g_dev = new WAKE_UP_I2C_SLAVE(GO_SLEEP_I2C_BUS);

	if (g_dev == nullptr) {
		
		warnx("1"); // debug erreur
		goto fail;
	}
	if (OK != g_dev->init()) {
		
		warnx("2"); // debug erreur		
		goto fail;
	}

	fd = open(GO_SLEEP_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {		
		warnx("3"); // debug erreur		
		goto fail;
	}	
	
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
	warnx("No test available for this driver");
}

/**
 * Reset the driver.
 */
void
reset()
{
	int fd = open(GO_SLEEP_DEVICE_PATH, O_RDONLY);

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
    uint8_t sleep_MSB_01 = 0;
    uint8_t sleep_MSB_02 = 0;
    uint8_t sleep_LSB_01 = 0;
    uint8_t sleep_LSB_02 = 0;

    sleep_MSB_01 = (uint8_t)((time >> 24) & 0x000000FF); 
    sleep_MSB_02 = (uint8_t)((time >> 16) & 0x000000FF); 
    sleep_LSB_01 = (uint8_t)((time >> 8) & 0x000000FF); 
    sleep_LSB_02 = (uint8_t)((time ) & 0x000000FF); 

    /* read from the sensor */
    uint8_t test1[5] = {GO_SLEEP_MESS_OVHD,sleep_MSB_01,sleep_MSB_02,sleep_LSB_01,sleep_LSB_02};


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
		wake_up_i2c_slave::info();
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
