/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file lps33.h
 *
 * Shared defines for the lps33 driver.
 */

#pragma once

//REGISTRES SPI

#define INTERRUPT_CFG	0x0B	//Interrupt COnfiguration
#define THIS_P_L		0x0C	//Least significant bits of the threshold value for pressure interrupt generation
#define THIS_P_H		0x0D	//Most significant bits of the threshold value for pressure interrupt generation
#define WHO_AM_I		0x0F	//Device Who am i
#define CTRL_REG1		0x10	//CONTROL Register 1
#define CTRL_REG2		0x11	//CONTROL Register 2
#define CTRL_REG3		0x12	//CONTROL Register 3
#define FIFO_CTRL		0x14	//FIFO control register
#define REF_P_XL		0x15	//Reference pressure (LSB)
#define REF_P_L			0x16	//Reference pressure (middle)
#define REF_P_H			0x17	//Reference pressure (MSB)
#define RPDS_L			0x18	//Pressure Offset (LSB)
#define RPDS_H			0x19	//Pressure Offset (MSB)
#define RES_CONF		0x1A	//Low power mode config
#define INT_SOURCE		0x25	//Interrupt source
#define FIFO_STATUS		0x26	//FIFO status
#define STATUS			0x27	//STATUS REGISTER
#define PRESS_OUT_XL	0x28	//Pressure output value (LSB)
#define PRESS_OUT_L		0x29	//Pressure Output value (middle part)
#define PRESS_OUT_H		0x2A	//Pressure output value (MSB)
#define TEMP_OUT_L		0x2B	//Temp. value out (LSB)
#define TEMP_OUT_H		0x2C	//Temp. value out (MSB)
#define LPFP_RES		0x33	//Low Pass filter reset register

//PARAMETRES

#define CTRL_REG2_ONE_SHOT	(1 << 0)
#define CTRL_REG2_SWRESET	(1 << 2)
#define CTRL_REG2_BOOT		(1 << 7)


/* interface factories */
extern device::Device *LPS33_SPI_interface(int bus); //bool external bus
typedef device::Device *(*LPS33_constructor)(int);
