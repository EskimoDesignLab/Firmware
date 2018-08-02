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
 * @file drone_aqua_test.c
 *
 * Parameters defined by the perching_wall control task.
 *
 * @author Thomas Courteau <thomas.robichaud.courteau@gmail.com>
 */

#include <systemlib/param/param.h>

/*
 * Controller parameters, accessible via MAVLink
 *
 */

/**
 * Custom takeoff timing: timing 11
 *
 *
 * @unit s
 * @min -10.0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(TK_HOR_POS, 0.0f);

/**
 * Custom takeoff timing: timing 11
 *
 *
 * @unit s
 * @min -10.0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(TK_UP_POS, 1.0f);

/**
 * Custom takeoff timing: timing 11
 *
 *
 * @unit s
 * @min -10.0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(TK_DN_POS, -1.0f);

/**
 * Custom takeoff timing: timing 01
 *
 *
 * @unit us
 * @min 0.0
 * @max 10000000.0
 * @decimal 2
 * @increment 0.05
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(TK_WAIT_TIME, 2000000.0f);

/**
 * Custom takeoff timing: timing 08
 *
 *
 * @unit us
 * @min 0.0
 * @max 10000000.0
 * @decimal 2
 * @increment 0.05
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(TK_IDLE_UP_TIME, 2000000.0f);

/**
 * Custom takeoff timing: timing 09
 *
 *
 * @unit us
 * @min 0.0
 * @max 10000000.0
 * @decimal 2
 * @increment 0.05
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(TK_FULL_UP_TIME, 230000.0f);

/**
 * Custom takeoff timing: timing 11
 *
 *
 * @unit us
 * @min 0.0
 * @max 10000000.0
 * @decimal 2
 * @increment 0.05
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(TK_FULL_DN_TIME, 2000000.0f);

/**
 * Custom takeoff pitch controler : Gain Kp
 *
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(TK_CON_KP, 0.20f);

/**
 * Custom takeoff timing: controller timing
 *
 * @unit us
 * @min 0.0
 * @max 1000000000.0
 * @decimal 2
 * @increment 0.05
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(TK_CON_TIME, 5000000.0f);

/**
* Custom takeoff pitch controler : Gain Kd
*
*
* @min 0.0
* @max 1.0
* @decimal 3
* @increment 0.001
* @group FW Attitude Control
*/
PARAM_DEFINE_FLOAT(TK_CON_KD, 0.01f);

/**
 * Custom takeoff pitch angle pendant la manoeuvre
 *
 *
 * @unit rad
 * @min 0.0
 * @max 10000000.0
 * @decimal 2
 * @increment 0.001
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(TK_CUSTM_PITCH, 1.571f);

/**
 * Custom takeoff yaw controler : Gain Kp
 *
 *
 * @min -10.0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group FW Attitude Control
 */
PARAM_DEFINE_FLOAT(TK_YAW_KP, 0.20f);

/**
* Custom takeoff yaw controler : Gain Kd
*
*
* @min -10.0
* @max 10.0
* @decimal 3
* @increment 0.001
* @group FW Attitude Control
*/
PARAM_DEFINE_FLOAT(TK_YAW_KD, 0.01f);








