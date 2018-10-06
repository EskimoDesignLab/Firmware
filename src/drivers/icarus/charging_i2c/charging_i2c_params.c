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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file perching_wall_params.c
 *
 * Parameters defined by the perching_wall control task.
 *
 * @author Thomas Courteau <thomas.robichaud.courteau@gmail.com>
 */

#include <parameters/param.h>

/*
 * Controller parameters, accessible via MAVLink
 *
 */

/**
 * -> 0 if monitoring / 1 if configuring / other to desactivate the charging driver
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 1.0
 * @reboot_required true
 * @group PID
 */
PARAM_DEFINE_FLOAT(IS_NEW_BATTERY, 0.0f);

/**
 * Battery capacity (mAh)
 *
 * @min 0.0
 * @max 20000.0
 * @decimal 2
 * @increment 1.0
 * @reboot_required true
 * @group PID
 */
PARAM_DEFINE_FLOAT(BATT_CAPA_MAH, 1000.0f);

/**
 * Resistor for current sense on MAX17205 (mOhm)
 *
 * @min 0.0
 * @max 50.0
 * @decimal 2
 * @increment 0.1
 * @reboot_required true
 * @group PID
 */
PARAM_DEFINE_FLOAT(RSENSE_MOHM, 2.8f);

/**
 * Resistor for current sense on MAX17205 (mOhm)
 *
 * @min 0.0
 * @max 7.0
 * @decimal 2
 * @increment 1.0
 * @reboot_required true
 * @group PID
 */
PARAM_DEFINE_FLOAT(BALANCE_THRESH, 2.0f);



