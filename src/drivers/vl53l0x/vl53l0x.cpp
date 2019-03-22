/****************************************************************************
 *
 *   Copyright (c) 2015-2017 Createk Design Lab. All rights reserved.
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
 * 3. Neither the name Createk nor the names of its contributors may be
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
 * @file vl53l0x.cpp
 * @author Thomas Courteau <thomas.robichaud.courteau@gmail.com>
 *
 * Driver for the VL53L0X range finders connected via I2C.
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

#include <board_config.h>
#include <drivers/vl53l0x/Api/platform/inc/vl53l0x_platform.h>
#include <drivers/vl53l0x/Api/core/inc/vl53l0x_def.h>

#include "vl53l0x_api.h"

/* Configuration Constants */
#define VL53L0X_I2C_BUS 		PX4_I2C_BUS_EXPANSION
#define VL53L0X_I2C_BASEADDR 	0x29 /* 7-bit address. 8-bit address is 0x52 */
#define VL53L0X_DEVICE_PATH	"/dev/vl53l0x"

/* VL53L0X Registers addresses */

#define VL53L0X_TAKE_RANGE_REG	0x51		/* Measure range Register */
#define VL53L0X_SET_ADDRESS_0	0xA0		/* Change address 0 Register */
#define VL53L0X_SET_ADDRESS_1	0xAA		/* Change address 1 Register */
#define VL53L0X_SET_ADDRESS_2	0xA5		/* Change address 2 Register */

/* Device limits */
#define VL53L0X_MIN_DISTANCE 	(0.20f)
#define VL53L0X_MAX_DISTANCE 	(6.00f)

#define VL53L0X_CONVERSION_INTERVAL 	100000 /* 60ms for one sonar */
#define TICKS_BETWEEN_SUCCESIVE_FIRES 	100000 /* 30ms between each sonar measurement (watch out for interference!) */

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class VL53L0X : public device::I2C
{
public:
	VL53L0X(int bus = VL53L0X_I2C_BUS, int address = VL53L0X_I2C_BASEADDR);
	virtual ~VL53L0X();

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

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	uint8_t				_cycle_counter;	/* counter in cycle to change i2c adresses */
	int					_cycling_rate;	/* */
	uint8_t				_index_counter;	/* temporary sonar i2c address */
	std::vector<uint8_t>	addr_ind; 	/* temp sonar i2c address vector */
	std::vector<float>
	_latest_sonar_measurements; /* vector to store latest sonar measurements in before writing to report */

    VL53L0X_Dev_t           MyDevice;
    VL53L0X_Dev_t           * const pMyDevice = &MyDevice;
    VL53L0X_Version_t       Version;
    VL53L0X_Version_t       * pVersion = &Version;
    VL53L0X_DeviceInfo_t    DeviceInfo;


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
	* range to be brought in at all, otherwise it will use the defaults VL53L0X_MIN_DISTANCE
	* and VL53L0X_MAX_DISTANCE
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
extern "C" { __EXPORT int vl53l0x_main(int argc, char **argv);}

VL53L0X::VL53L0X(int bus, int address) :
	I2C("VL53L0X", VL53L0X_DEVICE_PATH, bus, address, 100000),
	_min_distance(VL53L0X_MIN_DISTANCE),
	_max_distance(VL53L0X_MAX_DISTANCE),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	_collect_phase(false),
	_class_instance(-1),
	_orb_class_instance(-1),
	_distance_sensor_topic(nullptr),
	_sample_perf(perf_alloc(PC_ELAPSED, "vl53l0x_i2c_read")),
	_comms_errors(perf_alloc(PC_COUNT, "vl53l0x_i2c_comms_errors")),
	_cycle_counter(0),	/* initialising counter for cycling function to zero */
	_cycling_rate(0),	/* initialising cycling rate (which can differ depending on one sonar or multiple) */
	_index_counter(0) 	/* initialising temp sonar i2c address to zero */

{
	/* enable debug() calls */
	_debug_enabled = true;

	/* work_cancel in the dtor will explode if we don't do this... */
	memset(&_work, 0, sizeof(_work));
}

VL53L0X::~VL53L0X()
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

int
VL53L0X::init()
{


	int ret = PX4_ERROR;
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t RangingMeasurementData;

    uint32_t  refSpadCount;
    uint8_t   isApertureSpads;
    uint8_t   VhvSettings;
    uint8_t   PhaseCal;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
        return ret;
    }

    MyDevice.I2cDevAddr = VL53L0X_I2C_BASEADDR;
    MyDevice.comms_type = 1;
    MyDevice.comms_speed_khz = 100;

    MyDevice.device_i2c_px4 = _dev;

    status = VL53L0X_DataInit(&MyDevice);
	if (status != VL53L0X_ERROR_NONE) {
		DEVICE_DEBUG("DataInit() failed");
		return status;
	}

    status = VL53L0X_GetDeviceInfo(&MyDevice, &DeviceInfo);
    if (status == VL53L0X_ERROR_NONE) {
        warn("VL53L0X infos ::");
        warn("Device name : %s", DeviceInfo.Name);
        warn("Type: %s", DeviceInfo.Type);
        warn("ID: %s", DeviceInfo.ProductId);
        warn("Rev Major: %i", DeviceInfo.ProductRevisionMajor);
        warn("Minor: %i", DeviceInfo.ProductRevisionMinor);
    }

    status = VL53L0X_StaticInit(pMyDevice);

    if (status != VL53L0X_ERROR_NONE) {
        DEVICE_DEBUG("VL53L0X staticInit() failed");
        return status;
    }

    status = VL53L0X_PerformRefSpadManagement(pMyDevice, &refSpadCount, &isApertureSpads);
    DEVICE_DEBUG("refSpadCourt : %i\tisApertureSpad : %i", refSpadCount, isApertureSpads);

    if ( status == VL53L0X_ERROR_NONE ) {
        status = VL53L0X_PerformRefCalibration(pMyDevice, &VhvSettings, &PhaseCal);
    } else {
        DEVICE_DEBUG("RefSpadManagement failed");
        status = VL53L0X_ERROR_NONE;
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_SetDeviceMode( pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING );
    } else {
        DEVICE_DEBUG("RefCalibration failed");
        status = VL53L0X_ERROR_NONE;
    }

    if( status == VL53L0X_ERROR_NONE ) {
        status = VL53L0X_SetLimitCheckEnable( pMyDevice, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1 );
    } else {
        DEVICE_DEBUG("SetDeviceMode failed");
        status = VL53L0X_ERROR_NONE;
    }

    if( status == VL53L0X_ERROR_NONE ) {
        status = VL53L0X_SetLimitCheckEnable( pMyDevice, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1 );
    } else {
        DEVICE_DEBUG("set4 fail");
        status = VL53L0X_ERROR_NONE;
    }

    if( status == VL53L0X_ERROR_NONE ) {
        status = VL53L0X_SetLimitCheckEnable( pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1 );
    } else {
        DEVICE_DEBUG("set5 fail");
    }

    if( status == VL53L0X_ERROR_NONE ) {
        status = VL53L0X_SetLimitCheckValue( pMyDevice, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, (FixPoint1616_t)( 1.5 * 0.023 * 65536 ) );
    } else {
        DEVICE_DEBUG("set6 fail");
    }

    if (status != VL53L0X_ERROR_NONE) {
        DEVICE_DEBUG("set failed");
        return status;
    }

    VL53L0X_PerformSingleRangingMeasurement( pMyDevice, &RangingMeasurementData );
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetRangeStatusString(RangingMeasurementData.RangeStatus, buf);
    DEVICE_DEBUG("Range status : %i\t%s", RangingMeasurementData.RangeStatus, buf);
    DEVICE_DEBUG("Measure mm : %i", RangingMeasurementData.RangeMilliMeter);

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

	_index_counter = VL53L0X_I2C_BASEADDR;	/* set temp sonar i2c address to base adress */
	set_address(_index_counter);		/* set I2c port to temp sonar i2c adress */

	if (_reports == nullptr) {
		return ret;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	/* get a publish handle on the range finder topic */
	struct distance_sensor_s ds_report = {};

	_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
				 &_orb_class_instance, ORB_PRIO_LOW);

	if (_distance_sensor_topic == nullptr) {
		DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
	}
#if 0
	// XXX we should find out why we need to wait 200 ms here
	usleep(200000);

	/* check for connected rangefinders on each i2c port:
	   We start from i2c base address (0x70 = 112) and count downwards
	   So second iteration it uses i2c address 111, third iteration 110 and so on*/
	for (unsigned counter = 0; counter <= MB12XX_MAX_RANGEFINDERS; counter++) {
		_index_counter = VL53L0X_I2C_BASEADDR + counter * 2;	/* set temp sonar i2c address to base adress - counter */
		set_address(_index_counter);			/* set I2c port to temp sonar i2c adress */
		int ret2 = measure();

		if (ret2 == 0) { /* sonar is present -> store address_index in array */
			addr_ind.push_back(_index_counter);
			DEVICE_DEBUG("sonar added");
			_latest_sonar_measurements.push_back(200);
		}
	}
#endif
	_index_counter = VL53L0X_I2C_BASEADDR;
	set_address(_index_counter); /* set i2c port back to base adress for rest of driver */

	/* if only one sonar detected, no special timing is required between firing, so use default */
	if (addr_ind.size() == 1) {
		_cycling_rate = VL53L0X_CONVERSION_INTERVAL;

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
VL53L0X::probe()
{
	return OK;
}

void
VL53L0X::set_minimum_distance(float min)
{
	_min_distance = min;
}

void
VL53L0X::set_maximum_distance(float max)
{
	_max_distance = max;
}

float
VL53L0X::get_minimum_distance()
{
	return _min_distance;
}

float
VL53L0X::get_maximum_distance()
{
	return _max_distance;
}

int
VL53L0X::ioctl(struct file *filp, int cmd, unsigned long arg)
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
VL53L0X::read(struct file *filp, char *buffer, size_t buflen)
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
#if 0
		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}
#endif
		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

int
VL53L0X::measure()
{
    int	ret = -EIO;

    VL53L0X_RangingMeasurementData_t RangingMeasurementData;
    VL53L0X_PerformSingleRangingMeasurement( pMyDevice, &RangingMeasurementData );
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetRangeStatusString(RangingMeasurementData.RangeStatus, buf);
    DEVICE_DEBUG("Range status : %i\t%s", RangingMeasurementData.RangeStatus, buf);
    DEVICE_DEBUG("Measure mm : %i", RangingMeasurementData.RangeMilliMeter);

    struct distance_sensor_s report;
    report.timestamp = hrt_absolute_time();
    report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
    report.orientation = 8;
    report.current_distance = RangingMeasurementData.RangeMilliMeter;
    report.min_distance = get_minimum_distance();
    report.max_distance = get_maximum_distance();
    report.covariance = 0.0f;
    report.id = RangingMeasurementData.ZoneId;

    /* publish it, if we are the primary */
    if (_distance_sensor_topic != nullptr) {
        orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
    }

    _reports->force(&report);

    /* notify anyone waiting for data */
    poll_notify(POLLIN);

    ret = OK;

    perf_end(_sample_perf);
    return ret;
}

int
VL53L0X::collect()
{
	int	ret = -EIO;

	/* read from the sensor */
	uint8_t val[2] = {0, 0};
	uint8_t cmd = 0x02;
	perf_begin(_sample_perf);

	ret = transfer(&cmd, 1, nullptr, 0);
	ret = transfer(nullptr, 0, &val[0], 2);

	if (ret < 0) {
		DEVICE_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}

	uint16_t distance_cm = val[0] << 8 | val[1];
	float distance_m = float(distance_cm) * 1e-2f;

	struct distance_sensor_s report;
	report.timestamp = hrt_absolute_time();
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_ULTRASOUND;
	report.orientation = 8;
	report.current_distance = distance_m;
	report.min_distance = get_minimum_distance();
	report.max_distance = get_maximum_distance();
	report.covariance = 0.0f;
	/* TODO: set proper ID */
	report.id = 0;

	/* publish it, if we are the primary */
	if (_distance_sensor_topic != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &report);
	}

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;

	perf_end(_sample_perf);
	return ret;
}

void
VL53L0X::start()
{

	/* reset the report ring and state machine */
	_collect_phase = false;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&VL53L0X::cycle_trampoline, this, 5);

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
VL53L0X::stop()
{
	work_cancel(HPWORK, &_work);
}

void
VL53L0X::cycle_trampoline(void *arg)
{

	VL53L0X *dev = (VL53L0X *)arg;

	dev->cycle();

}

void
VL53L0X::cycle()
{
	/* Measurement (firing) phase */

	/* ensure sonar i2c adress is still correct */
	_index_counter = addr_ind[_cycle_counter];
	set_address(_index_counter);

	/* Perform measurement */
	if (OK != measure()) {
		DEVICE_DEBUG("measure error sonar adress %d", _index_counter);
	}
	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&VL53L0X::cycle_trampoline,
		   this,
		   USEC2TICK(_cycling_rate));

}

void
VL53L0X::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

/**
 * Local functions in support of the shell command.
 */
namespace  vl53l0x
{

VL53L0X	*g_dev;

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

	/* create the driver */
	g_dev = new VL53L0X(VL53L0X_I2C_BUS);

	if (g_dev == nullptr) {
		goto fail;
	}

	if (OK != g_dev->init()) {
		goto fail;
	}

	/* set the poll rate to default, starts automatic data collection */
	fd = open(VL53L0X_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

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

	int fd = open(VL53L0X_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		err(1, "%s open failed (try 'vl53l0x start' if the driver is not running", VL53L0X_DEVICE_PATH);
	}

	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));

	if (sz != sizeof(report)) {
		err(1, "immediate read failed");
	}

	warnx("single read");
	warnx("measurement: %0.2f mm", (double)report.current_distance);
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
	int fd = open(VL53L0X_DEVICE_PATH, O_RDONLY);

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
vl53l0x_main(int argc, char **argv)
{
	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[1], "start")) {
		vl53l0x::start();
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[1], "stop")) {
		vl53l0x::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		vl53l0x::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		vl53l0x::reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[1], "info") || !strcmp(argv[1], "status")) {
		vl53l0x::info();
	}

	errx(1, "unrecognized command, try 'start', 'test', 'reset' or 'info'");
}
