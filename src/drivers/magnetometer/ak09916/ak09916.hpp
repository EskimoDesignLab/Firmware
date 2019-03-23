#ifndef AK09916_HPP_
#define AK09916_HPP_

#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <px4_log.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <nuttx/wqueue.h>
#include <systemlib/conversions.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

#include <board_config.h>
#include <drivers/drv_hrt.h>

#include <drivers/device/ringbuffer.h>
#include <drivers/device/integrator.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_mag.h>
#include <mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/conversion/rotation.h>


#define AK09916_DEVICE_PATH_MAG              "/dev/ak09916_i2c_int"

#define AK09916_DEVICE_PATH_MAG_EXT          "/dev/ak09916_i2c_ext"

#define AK09916_SLAVE_ADDRESS                 0x0C

#define AK09916_BUS_SPEED                     1000*100

/* Chip Who Am I */
#define AK09916_COMPANY_ID_REG               	0x00
#define AK09916_DEVICE_ID_REG              		0x01
#define AK09916_COMPANY_ID										0x48
#define AK09916_DEVICE_ID											0x09

/* Data Registers */
#define AK09916_DATA_X_LSB_REG                0x11
#define AK09916_DATA_X_MSB_REG                0x12
#define AK09916_DATA_Y_LSB_REG                0x13
#define AK09916_DATA_Y_MSB_REG                0x14
#define AK09916_DATA_Z_LSB_REG                0x15
#define AK09916_DATA_Z_MSB_REG                0x16

/* Status registers */
#define AK09916_STATUS1_REG               	 	0x10
#define AK09916_STATUS2_REG               	 	0x18
#define AK09916_SENSOR_OVERFLOW								0x08

/* Control Registers */
#define AK09916_CNTL1_REG                     0x30
#define AK09916_CNTL2_REG                     0x31
#define AK09916_CNTL3_REG                     0x32

/* This value is set based on Max output data rate value */
#define AK09916_CONVERSION_INTERVAL          (1000000 / 100) /* microseconds */
#define AK09916_MAX_DATA_RATE          				100

/* Operation mode value */
#define AK09916_POWER_DOWN										0x00
#define AK09916_SINGLE_MEASURE								0x01
#define AK09916_CONTINUOUS_MODE_1							0x02
#define AK09916_CONTINUOUS_MODE_2							0x04
#define AK09916_CONTINUOUS_MODE_3							0x06
#define AK09916_CONTINUOUS_MODE_4							0x08
#define AK09916_SELF_TEST											0x10

/* Self reset value */
#define AK09916_SELF_RESET										0x01

/* Conversion from raw data to uT */
#define AK09916_CONVERSION_SCALE							(4912.0f/32752.0f) 


struct ak09916_data {
	int16_t x;
	int16_t y;
	int16_t z;
};


class AK09916 : public device::I2C
{
public:
	AK09916(int bus, const char *path, enum Rotation rotation);
	virtual ~AK09916();

	virtual int             init();
	virtual ssize_t       read(struct file *filp, char *buffer, size_t buflen);
	virtual int       ioctl(struct file *filp, int cmd, unsigned long arg);

	/**
	 * Stop automatic measurement.
	 */
	void            stop();

	/**
	  * Diagnostics - print some basic information about the driver.
	  */
	void            print_info();

	void        print_registers();

protected:
	virtual int       probe();

private:
	work_s            _work{};

	bool _running;

	/* altitude conversion calibration */
	unsigned        _call_interval;


	mag_report _report {};
	ringbuffer::RingBuffer  *_reports;

	bool            _collect_phase;

	struct mag_calibration_s    _scale;
	float           _range_scale;

	orb_advert_t        _topic;
	int         _orb_class_instance;
	int         _class_instance;
	bool        _calibrated;        /**< the calibration is valid */

	perf_counter_t      _sample_perf;
	perf_counter_t      _bad_transfers;
	perf_counter_t      _good_transfers;
	perf_counter_t      _measure_perf;
	perf_counter_t      _comms_errors;
	perf_counter_t      _duplicates;

	enum Rotation       _rotation;

	mag_report   _last_report {};          /**< used for info() */

	int             init_trim_registers(void);

	/**
	 * Start automatic measurement.
	 */
	void            start();

	int     measure(); //start measure
	int     collect(); //get results and publish

	static void     cycle_trampoline(void *arg);
	void            cycle(); //main execution

	/**
	 * Read the specified number of bytes from AK09916.
	 *
	 * @param reg       The register to read.
	 * @param data      Pointer to buffer for bytes read.
	 * @param len       Number of bytes to read
	 * @return          OK if the transfer was successful, -errno otherwise.
	 */
	int             get_data(uint8_t reg, uint8_t *data, unsigned len);

	/**
	 * Resets the chip.
	 */
	int             reset();

	/**
	 * Measurement self test
	 *
	 * @return 0 on success, 1 on failure
	 */
	int             self_test();

	/**
	 * Read a register from the AK09916
	 *
	 * @param reg     The register to read.
	 * @return        The value that was read.
	 */
	uint8_t         read_reg(uint8_t reg);

	/**
	 * Write a register in the AK09916
	 *
	 * @param reg       The register to write.
	 * @param value     The new value to write.
	 * @return          OK if the transfer was successful, -errno otherwise.
	 */
	int             write_reg(uint8_t reg, uint8_t value);

	/**
	 * Modify a register in the AK09916
	 *
	 * Bits are cleared before bits are set.
	 *
	 * @param reg       The register to modify.
	 * @param clearbits Bits in the register to clear.
	 * @param setbits   Bits in the register to set.
	 */
	void            modify_reg(unsigned reg, uint8_t clearbits, uint8_t setbits);

	/* do not allow to copy this class due to pointer data members */
	AK09916(const AK09916 &);
	AK09916 operator=(const AK09916 &);

};



#endif /* AK09916_HPP_ */
