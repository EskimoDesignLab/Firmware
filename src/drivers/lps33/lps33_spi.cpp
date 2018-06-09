/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file LPS33_SPI.cpp
 *
 * SPI interface for LPS33
 */

/* XXX trim includes */
#include <px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <unistd.h>

#include <arch/board/board.h>

#include <drivers/device/spi.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_device.h>

#include "lps33.h"
#include <board_config.h>

#if PX4_SPIDEV_BARO
#ifdef PX4_SPI_BUS_EXT
#define EXTERNAL_BUS PX4_SPI_BUS_EXT
#else
#define EXTERNAL_BUS 0
#endif


/* SPI protocol address bits */
#define DIR_READ			(1<<7)
#define DIR_WRITE			(0<<7)
//#define ADDR_INCREMENT			(1<<6)

device::Device *LPS33_SPI_interface(int bus); //bool external bus

class LPS33_SPI : public device::SPI
{
public:
    LPS33_SPI(int bus, spi_dev_e device);
    virtual ~LPS33_SPI();

	virtual int	init();
	virtual int	read(unsigned address, void *data, unsigned count);
	virtual int	write(unsigned address, void *data, unsigned count);

	virtual int	ioctl(unsigned operation, unsigned &arg);

};

device::Device *
LPS33_SPI_interface(int bus) //bool external bus
{
    /*
    spi_dev_e cs = SPIDEV_NONE;
    device::Device *interface = nullptr;

    if (1) {
#if defined(PX4_SPI_BUS_EXT) && defined(PX4_SPIDEV_EXT0)
        cs = (spi_dev_e) PX4_SPIDEV_EXT0;
#else
        errx(0, "External SPI not available");
#endif

    } else {
        cs = (spi_dev_e) PX4_SPIDEV_BARO;
        warnx("test");

    }

    if (cs != SPIDEV_NONE) {
        interface = new LPS33_SPI(bus, cs);
    }

    return interface;
    */

        return new LPS33_SPI(bus, (spi_dev_e)PX4_SPIDEV_EXT1);

}

LPS33_SPI::LPS33_SPI(int bus, spi_dev_e device) :
    SPI("LPS33_SPI", nullptr, bus, device, SPIDEV_MODE3, 1000 * 1000 /* originalement 11*1000*1000 pour 10.4 Mhz */)
{
    _device_id.devid_s.devtype = DRV_BARO_DEVTYPE_LPS33;
}

LPS33_SPI::~LPS33_SPI()
{
}

int
LPS33_SPI::init()
{
	int ret;

	ret = SPI::init();

	if (ret != OK) {
		DEVICE_DEBUG("SPI init failed");
		return -EIO;
	}
    
    warnx("LPS33_SPI INITIALIZATION BEGIN");
    

    // read WHO_AM_I value
    uint8_t id;
    if (read(WHO_AM_I, &id, 1)) {
        DEVICE_DEBUG("read_reg fail");
        return -EIO;
    }

    uint8_t ID_WHO_AM_I = 0b10110001;

    if (id != ID_WHO_AM_I) {
        DEVICE_DEBUG("ID byte mismatch (%02x != %02x)", ID_WHO_AM_I, id);
        return -EIO;
    }
    
    // write Ctrl_Reg_1 value
    uint8_t tempData = 0b01000000;
    if (write(CTRL_REG1, &tempData, 1)) {
        DEVICE_DEBUG("read_reg fail");
        return -EIO;
    }

    // write Ctrl_Reg_2 value
    uint8_t tempData1 = 0b00000000;
    if (write(CTRL_REG2, &tempData1, 1)) {
        DEVICE_DEBUG("read_reg fail");
        return -EIO;
    }

    // AUTOZERO
    uint8_t tempData2 = 0b00000000;
    if (write(INTERRUPT_CFG, &tempData2, 1)) {
        DEVICE_DEBUG("read_reg fail");
        return -EIO;
    }

    return OK;


}

int
LPS33_SPI::ioctl(unsigned operation, unsigned &arg)
{
	int ret;

	switch (operation) {

	case DEVIOCGDEVICEID:
		return CDev::ioctl(nullptr, operation, arg);

	default: {
			ret = -EINVAL;
		}
	}

	return ret;
}

int
LPS33_SPI::write(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

	buf[0] = address | DIR_WRITE;
    buf[1] = *(uint8_t *)data;
	return transfer(&buf[0], &buf[0], count + 1);
}

int
LPS33_SPI::read(unsigned address, void *data, unsigned count)
{
	uint8_t buf[32];

	if (sizeof(buf) < (count + 1)) {
		return -EIO;
	}

    //buf[0] = address | DIR_READ | ADDR_INCREMENT;
    buf[0] = address | DIR_READ;

	int ret = transfer(&buf[0], &buf[0], count + 1);
	memcpy(data, &buf[1], count);
	return ret;
}

#endif /* PX4_SPIDEV_HMC */
