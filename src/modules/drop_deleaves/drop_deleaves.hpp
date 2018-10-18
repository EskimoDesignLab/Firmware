/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#pragma once

#include <lib/mixer/mixer.h>
#include <matrix/matrix/math.hpp>
#include <perf/perf_counter.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <drivers/drv_hrt.h>





#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>
#include <math.h>

#include <uORB/uORB.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/rc_channels.h>

extern "C" __EXPORT int drop_deleaves_main(int argc, char **argv);

class DropDeLeaves : public ModuleBase<DropDeLeaves>, public ModuleParams
{
public:
    DropDeLeaves();

    virtual ~DropDeLeaves() = default;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static DropDeLeaves *instantiate(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::run() */
    void run() override;

    /** @see ModuleBase::print_status() */
    int print_status() override;

private:
    /**
    * Check for parameter update and handle it.
    */
    void		rc_channels_poll();
    int         actuators_publish();

    void        drop_switch_state();
    void        check_2_drop();
    void        drop();


    hrt_abstime now;

    int _rc_channels_sub{-1};

    struct rc_channels_s _rc_channels{};

    struct actuator_controls_s      _actuators;    /**< actuator controls */

    bool _curr_drop_switch_state = 0;
    bool _last_drop_switch_state = 0;

    bool _flg_drop1 = 0;
    bool _flg_drop2 = 0;
    bool _flg_drop3 = 0;
    bool _flg_drop4 = 0;
    bool _flg_drop5 = 0;
    hrt_abstime	_check_2_drop_time;

    orb_id_t _actuator_id{nullptr}; /**< pointer to correct actuator controls0 uORB metadata structure */

    orb_advert_t	_actuator_pub{nullptr}; /**< attitude actuator controls publication */

    perf_counter_t	_loop_perf;			/**< loop performance counter */
};

