/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file ak09916.cpp
 * Driver for the Bosch BMM 150 MEMS magnetometer connected via I2C.
 */

#include "ak09916.hpp"
#include <px4_getopt.h>

/** driver 'main' command */
extern "C" { __EXPORT int ak09916_main(int argc, char *argv[]); }


namespace ak09916
{

AK09916 *g_dev_ext;
AK09916 *g_dev_int;

void start(bool, enum Rotation);
void test(bool);
void reset(bool);
void info(bool);
void regdump(bool external_bus);
void usage();


/**
 * Start the driver.
 *
 * This function only returns if the driver is up and running
 * or failed to detect the sensor.
 */
void start(bool external_bus, enum Rotation rotation)
{
	int fd;
	AK09916 **g_dev_ptr = (external_bus ? &g_dev_ext : &g_dev_int);
	const char *path = (external_bus ? AK09916_DEVICE_PATH_MAG_EXT : AK09916_DEVICE_PATH_MAG);

	if (*g_dev_ptr != nullptr)
		/* if already started, the still command succeeded */
	{
		PX4_ERR("already started");
		exit(0);
	}

	/* create the driver */
	if (external_bus) {
#if defined(PX4_I2C_BUS_EXPANSION)
		*g_dev_ptr = new AK09916(PX4_I2C_BUS_EXPANSION, path, rotation);
#else
		PX4_ERR("External I2C not available");
		exit(0);
#endif

	} else {
		*g_dev_ptr = new AK09916(PX4_I2C_BUS_ONBOARD, path, rotation);
	}

	if (*g_dev_ptr == nullptr) {
		goto fail;
	}


	if (OK != (*g_dev_ptr)->init()) {
		goto fail;
	}
	/* set the poll rate to default, starts automatic data collection */
	fd = open(path, O_RDONLY);


	if (fd < 0) {
		goto fail;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		goto fail;
	}

	close(fd);

	exit(0);
fail:

	if (*g_dev_ptr != nullptr) {
		delete (*g_dev_ptr);
		*g_dev_ptr = nullptr;
	}

	PX4_ERR("driver start failed");
	exit(1);

}


void test(bool external_bus)
{
	int fd = -1;
	const char *path = (external_bus ? AK09916_DEVICE_PATH_MAG_EXT : AK09916_DEVICE_PATH_MAG);
	struct mag_report m_report;
	ssize_t sz;


	/* get the driver */
	fd = open(path, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("%s open failed (try 'ak09916 start' if the driver is not running)", path);
		exit(1);
	}

	/* reset to default polling rate*/
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("reset to Max polling rate");
		exit(1);
	}

	/* do a simple demand read */
	sz = read(fd, &m_report, sizeof(m_report));

	if (sz != sizeof(m_report)) {
		PX4_ERR("immediate mag read failed");
		exit(1);
	}

	PX4_WARN("single read");
	PX4_WARN("time:     %lld", m_report.timestamp);
	PX4_WARN("mag  x:  \t%8.4f\t", (double)m_report.x);
	PX4_WARN("mag  y:  \t%8.4f\t", (double)m_report.y);
	PX4_WARN("mag  z:  \t%8.4f\t", (double)m_report.z);
	PX4_WARN("mag  x:  \t%d\traw 0x%0x", (short)m_report.x_raw, (unsigned short)m_report.x_raw);
	PX4_WARN("mag  y:  \t%d\traw 0x%0x", (short)m_report.y_raw, (unsigned short)m_report.y_raw);
	PX4_WARN("mag  z:  \t%d\traw 0x%0x", (short)m_report.z_raw, (unsigned short)m_report.z_raw);

	PX4_ERR("PASS");
	exit(0);

}


void
reset(bool external_bus)
{
	const char *path = external_bus ? AK09916_DEVICE_PATH_MAG_EXT : AK09916_DEVICE_PATH_MAG;
	int fd = open(path, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("failed");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		exit(1);
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		exit(1);
	}

	exit(0);

}

void
info(bool external_bus)
{
	AK09916 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		PX4_ERR("driver not running");
		exit(1);
	}

	printf("state @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_info();

	exit(0);

}

/**
 * Dump the register information
 */
void
regdump(bool external_bus)
{
	AK09916 **g_dev_ptr = external_bus ? &g_dev_ext : &g_dev_int;

	if (*g_dev_ptr == nullptr) {
		PX4_ERR("driver not running");
		exit(1);
	}

	printf("regdump @ %p\n", *g_dev_ptr);
	(*g_dev_ptr)->print_registers();

	exit(0);
}

void
usage()
{
	PX4_WARN("missing command: try 'start', 'info', 'test', 'stop',\n'reset', 'regdump'");
	PX4_WARN("options:");
	PX4_WARN("    -X    (external bus)");

}

} // namespace ak09916


AK09916 :: AK09916(int bus, const char *path, enum Rotation rotation) :
	I2C("AK09916", path, bus, AK09916_SLAVE_ADDRESS, AK09916_BUS_SPEED),
	_running(false),
	_call_interval(0),
	_reports(nullptr),
	_collect_phase(false),
	_scale{},
	_range_scale(0.01f), // uT to gauss
	_topic(nullptr),
	_orb_class_instance(-1),
	_class_instance(-1),
	_calibrated(false),
	_sample_perf(perf_alloc(PC_ELAPSED, "ak09916_read")),
	_bad_transfers(perf_alloc(PC_COUNT, "ak09916_bad_transfers")),
	_good_transfers(perf_alloc(PC_COUNT, "ak09916_good_transfers")),
	_measure_perf(perf_alloc(PC_ELAPSED, "bmp280_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "bmp280_comms_errors")),
	_duplicates(perf_alloc(PC_COUNT, "ak09916_duplicates")),
	_rotation(rotation)
	{
	_device_id.devid_s.devtype = DRV_MAG_DEVTYPE_AK09916;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));

	// default scaling
	_scale.x_offset = 0;
	_scale.x_scale = 1.0f;
	_scale.y_offset = 0;
	_scale.y_scale = 1.0f;
	_scale.z_offset = 0;
	_scale.z_scale = 1.0f;
}

AK09916 :: ~AK09916()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}


	if (_class_instance != -1) {
		unregister_class_devname(MAG_BASE_DEVICE_PATH, _class_instance);
	}

	if (_topic != nullptr) {
		orb_unadvertise(_topic);
	}


	/* delete the perf counter */
	perf_free(_sample_perf);
	perf_free(_bad_transfers);
	perf_free(_good_transfers);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
	perf_free(_duplicates);

}

int AK09916::init()
{
	int ret = OK;

	/* do I2C init (and probe) first */
	ret = I2C::init();

	/* if probe/setup failed, bail now */
	if (ret != OK) {
		DEVICE_DEBUG("I2C setup failed");
		return ret;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(mag_report));

	if (_reports == nullptr) {
		goto out;
	}

	/* Bring the device from sleep mode */
	write_reg(AK09916_CNTL2_REG, AK09916_CONTINUOUS_MODE_4);
	up_udelay(10000);


	/* check  id*/
	if (read_reg(AK09916_COMPANY_ID_REG) != AK09916_COMPANY_ID) {
		PX4_WARN("company id of magnetometer is not: 0x%02x", AK09916_COMPANY_ID);
		return -EIO;
	}

	/* check  id*/
	if (read_reg(AK09916_DEVICE_ID_REG) != AK09916_DEVICE_ID) {
		PX4_WARN("device id of magnetometer is not: 0x%02x", AK09916_DEVICE_ID);
		return -EIO;
	}

	if (reset() != OK) {
		goto out;
	}

	_class_instance = register_class_devname(MAG_BASE_DEVICE_PATH);

	if (measure()) {
		return -EIO;
	}

	up_udelay(10000);

	if (collect()) {
		return -EIO;
	}

	/* advertise sensor topic, measure manually to initialize valid report */
	struct mag_report mrb;
	_reports->get(&mrb);

	/* measurement will have generated a report, publish */
	_topic = orb_advertise_multi(ORB_ID(sensor_mag), &mrb,
				     &_orb_class_instance, (external()) ? ORB_PRIO_MAX-1 : ORB_PRIO_HIGH-1);

	if (_topic == nullptr) {
		PX4_WARN("ADVERT FAIL");
	}

out:
	return ret;
}

int
AK09916::probe()
{
	/* During I2C Initialization, sensor is in suspend mode
	 * and chip Id will return 0x00, hence returning OK. After
	 * I2C initialization, sensor is brought to sleep mode
	 * and Chip Id is verified. In sleep mode, we can read
	 * chip id. */

	/* @Note: Please read AK09916 Datasheet for more details */
	return OK;
}


void
AK09916::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_running = true;
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&AK09916::cycle_trampoline, this, 1);

}

void
AK09916::stop()
{
	_running = false;
	work_cancel(HPWORK, &_work);

}

ssize_t
AK09916::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(mag_report);
	struct mag_report *mag_buf = reinterpret_cast<struct mag_report *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_call_interval > 0) {
		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(mag_buf)) {
				ret += sizeof(struct mag_report);
				mag_buf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	/* XXX really it'd be nice to lock against other readers here */
	do {
		_reports->flush();

		/* trigger a measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(AK09916_CONVERSION_INTERVAL);

		/* run the collection phase */
		if (OK != collect()) {
			ret = -EIO;
			break;
		}


		if (_reports->get(mag_buf)) {
			ret = sizeof(struct mag_report);
		}
	} while (0);

	/* return the number of bytes transferred */
	return ret;

}


void
AK09916::cycle_trampoline(void *arg)
{
	AK09916 *dev = reinterpret_cast<AK09916 *>(arg);

	/* make measurement */
	dev->cycle();
}

void
AK09916::cycle()
{
	if (_collect_phase) {
		collect();
		unsigned wait_gap = _call_interval - USEC2TICK(AK09916_CONVERSION_INTERVAL);

		if ((wait_gap != 0) && (_running)) {
			work_queue(HPWORK, &_work, (worker_t)&AK09916::cycle_trampoline, this,
				   wait_gap); //need to wait some time before new measurement
			return;
		}

	}

	measure();

	if ((_running)) {
		/* schedule a fresh cycle call when the measurement is done */
		work_queue(HPWORK,
			   &_work,
			   (worker_t)&AK09916::cycle_trampoline,
			   this,
			   USEC2TICK(AK09916_CONVERSION_INTERVAL));
	}


}

int
AK09916::measure()
{
	_collect_phase = true;

	perf_begin(_measure_perf);


	perf_end(_measure_perf);

	return OK;
}



int
AK09916::collect()
{

	_collect_phase = false;

	bool mag_notify = true;
	uint8_t mag_data[8];
	uint16_t lsb, msb, msblsb;
	mag_report  mrb;


	/* start collecting data */
	perf_begin(_sample_perf);

	/* Read Magnetometer data*/
	if (OK != get_data(AK09916_DATA_X_LSB_REG, mag_data, sizeof(mag_data))) {
		return -EIO;
	}

	/* Array holding the mag XYZ data
	mag_data[0] - X LSB
	mag_data[1] - X MSB
	mag_data[2] - Y LSB
	mag_data[3] - Y MSB
	mag_data[4] - Z LSB
	mag_data[5] - Z MSB
	mag_data[6] - Reserved
	mag_data[7] - Status 2
	*/

	/* Extract X axis data */
	lsb = mag_data[0];
	msb = (((int8_t)mag_data[1]) << 8);
	msblsb = (msb | lsb);
	mrb.x_raw = (int16_t)msblsb;

	/* Extract Y axis data */
	lsb = mag_data[2];
	msb = (((int8_t)mag_data[3]) << 8);
	msblsb = (msb | lsb);
	mrb.y_raw = (int16_t)msblsb;

	/* Extract Z axis data */
	lsb = mag_data[4];
	msb = (((int8_t)mag_data[5]) << 8);
	msblsb = (msb | lsb);
	mrb.z_raw = (int16_t)msblsb;

	if (mrb.x_raw == 0 &&
	    mrb.y_raw == 0 &&
	    mrb.z_raw == 0) {
		// all zero data - probably a I2C bus error
		perf_count(_comms_errors);
		perf_count(_bad_transfers);
		perf_end(_sample_perf);
		PX4_WARN("All raw data are equal to 0");
		return -EIO;
	}

	if (mag_data[7] & AK09916_SENSOR_OVERFLOW) {
	// all zero data - probably a I2C bus error
	perf_count(_comms_errors);
	perf_count(_bad_transfers);
	perf_end(_sample_perf);
	PX4_WARN("Sensor overflow detected");
	return -EIO;
	}

	perf_count(_good_transfers);

	mrb.timestamp = hrt_absolute_time();
	mrb.is_external = external();

	// report the error count as the number of bad transfers.
	// This allows the higher level code to decide if it
	// should use this sensor based on whether it has had failures
	mrb.error_count = perf_event_count(_bad_transfers);
	
	mrb.x = mrb.x_raw * AK09916_CONVERSION_SCALE;
	mrb.y = mrb.y_raw * AK09916_CONVERSION_SCALE;
	mrb.z = mrb.z_raw * AK09916_CONVERSION_SCALE;

	// apply user specified rotation
	rotate_3f(_rotation, mrb.x, mrb.y, mrb.z);


	/* Scaling the data */
	mrb.x = ((mrb.x * _range_scale) - _scale.x_offset) * _scale.x_scale;
	mrb.y = ((mrb.y * _range_scale) - _scale.y_offset) * _scale.y_scale;
	mrb.z = ((mrb.z * _range_scale) - _scale.z_offset) * _scale.z_scale;

	mrb.scaling = _range_scale;
	mrb.device_id = _device_id.devid;

	_last_report.x = mrb.x;
	_last_report.y = mrb.y;
	_last_report.z = mrb.z;

	_reports->force(&mrb);

	/* notify anyone waiting for data */
	if (mag_notify) {
		poll_notify(POLLIN);
	}

	if (mag_notify && !(_pub_blocked)) {
		/* publish it */
		orb_publish(ORB_ID(sensor_mag), _topic, &mrb);
	}

	perf_end(_sample_perf);
	return OK;
}

int
AK09916::ioctl(struct file *filp, int cmd, unsigned long arg)
{

	switch (cmd) {
	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default polling rate */
			case SENSOR_POLLRATE_DEFAULT:
				return ioctl(filp, SENSORIOCSPOLLRATE, AK09916_MAX_DATA_RATE);

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_call_interval == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(AK09916_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_call_interval = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCRESET:
		return reset();

	case MAGIOCSSCALE:
		return OK;

	case MAGIOCGSCALE:
		return OK;

	case MAGIOCCALIBRATE:
		return OK;

	case MAGIOCEXSTRAP:
		return OK;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}


int AK09916::reset()
{
	/* Soft-reset */
	write_reg(AK09916_CNTL3_REG, AK09916_SELF_RESET);
	up_udelay(10000);

	/* Enable Magnetometer in normal mode */
	write_reg(AK09916_CNTL2_REG, AK09916_CONTINUOUS_MODE_4);
	up_udelay(10000);

	return OK;
}


int
AK09916::self_test()
{
	if (perf_event_count(_sample_perf) == 0) {
		collect();
	}

	/* return 0 on success, 1 else */
	return (perf_event_count(_sample_perf) > 0) ? 0 : 1;
}

uint8_t
AK09916::read_reg(uint8_t reg)
{
	uint8_t cmd = reg;
	uint8_t ret;
	transfer(&cmd, 1, &ret, 1);

	return ret;
}

int
AK09916::write_reg(uint8_t reg, uint8_t value)
{
	const uint8_t cmd[2] = { reg, value};

	return transfer(cmd, 2, nullptr, 0);
}

int
AK09916::get_data(uint8_t reg, uint8_t *data, unsigned len)
{
	const uint8_t cmd = reg;

	return transfer(&cmd, 1, data, len);
}

void
AK09916::modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits)
{
	uint8_t val;

	val = read_reg(reg);
	val &= ~clearbits;
	val |= setbits;
	write_reg(reg, val);
}

void
AK09916::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_bad_transfers);
	perf_print_counter(_good_transfers);
	_reports->print_info("mag queue");

	printf("output  (%.2f %.2f %.2f)\n", (double)_last_report.x, (double)_last_report.y, (double)_last_report.z);
	printf("offsets (%.2f %.2f %.2f)\n", (double)_scale.x_offset, (double)_scale.y_offset, (double)_scale.z_offset);
	printf("scaling (%.2f %.2f %.2f) 1/range_scale %.2f ",
	       (double)_scale.x_scale, (double)_scale.y_scale, (double)_scale.z_scale,
	       (double)(1.0f / _range_scale));
	printf("\n");

}

void
AK09916::print_registers()
{
	printf("AK09916 registers\n");

	uint8_t reg = AK09916_COMPANY_ID_REG;
	uint8_t v = read_reg(reg);
	printf("Company Id: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = AK09916_DEVICE_ID_REG;
	v = read_reg(reg);
	printf("Device Id: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

	reg = AK09916_CNTL2_REG;
	v = read_reg(reg);
	printf("Int sett Cntl2 reg: %02x:%02x ", (unsigned)reg, (unsigned)v);
	printf("\n");

}

int
ak09916_main(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	bool external_bus = false;
	enum Rotation rotation = ROTATION_NONE;

	while ((ch = px4_getopt(argc, argv, "XR:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'X':
			external_bus = true;
			break;

		case 'R':
			rotation = (enum Rotation)atoi(myoptarg);
			break;

		default:
			ak09916::usage();
			return 0;
		}
	}

	if (myoptind >= argc) {
		ak09916::usage();
		return -1;
	}

	const char *verb = argv[myoptind];

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(verb, "start")) {
		ak09916::start(external_bus, rotation);
	}


	/*
	 * Test the driver/device.
	 */
	if (!strcmp(verb, "test")) {
		ak09916::test(external_bus);
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(verb, "reset")) {
		ak09916::reset(external_bus);
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(verb, "info")) {
		ak09916::info(external_bus);
	}

	/*
	 * Print register information.
	 */
	if (!strcmp(verb, "regdump")) {
		ak09916::regdump(external_bus);
	}


	ak09916::usage();
	return -1;
}
