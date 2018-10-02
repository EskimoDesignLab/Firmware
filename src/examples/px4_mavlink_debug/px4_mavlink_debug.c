/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file px4_mavlink_debug.c
 * Debug application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>
 */

#include <px4_config.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>

#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/debug_key_value.h>

//#include <v2.0/mavlink_types.h>
//#include <v2.0/common/mavlink.h>
// #include <v2.0/mavlink_types.h>
// #include <v2.0/standard/mavlink.h>

#include <px4_tasks.h>
#include <px4_posix.h>

#include <sys/ioctl.h>
#include <drivers/drv_hrt.h>
#include <termios.h>

__EXPORT int px4_mavlink_debug_main		(int argc, char *argv[]);

int 	mav_enable_flow_control			(bool enabled,int _uart_fd);
int 	init_mavlink_uart				(char *uart_name);
int 	mav_set_baud					(int _uart_fd, char *_device);
void 	test1							(void);
void 	test2							(void);


int init_mavlink_uart(char *uart_name)
{
	// assuming NuttShell is on UART1 (/dev/ttyS0) /
	int _uart_fd = open(uart_name, O_RDWR | O_NONBLOCK | O_NOCTTY); //

	if (_uart_fd < 0) {
		printf("ERROR opening UART %s, aborting..\n", uart_name);
		return _uart_fd;

	} else {
		printf("Writing to UART %s\n", uart_name);
	}

    _uart_fd = mav_set_baud(_uart_fd,uart_name);
    if (_uart_fd < 0) {
        return _uart_fd;
    }


    return _uart_fd;
}


int mav_set_baud(int _uart_fd, char *_device)
{
	printf("mav_set_baud\n");
    // set baud rate
    int speed = B2000000;
    struct termios uart_config;
    tcgetattr(_uart_fd, &uart_config);
    // clear ONLCR flag (which appends a CR for every LF)
    uart_config.c_oflag &= ~ONLCR;

    /* Set baud rate */
    if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
        warnx("ERR SET BAUD %s\n", _device);
        close(_uart_fd);
        return -1;
    }

    if ((tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
        PX4_WARN("ERR SET CONF %s\n", _device);
        px4_close(_uart_fd);
        return -1;
    }

    /* setup output flow control */
    if (mav_enable_flow_control(false,_uart_fd)) {
        PX4_WARN("hardware flow disable failed");
    }

    return _uart_fd;
}

int mav_enable_flow_control(bool enabled,int _uart_fd)
{
	printf("mav_enable_flow_control\n");
    struct termios uart_config;

    int ret = tcgetattr(_uart_fd, &uart_config);

    if (enabled) {
        uart_config.c_cflag |= CRTSCTS;

    } else {
        uart_config.c_cflag &= ~CRTSCTS;
    }

    ret = tcsetattr(_uart_fd, TCSANOW, &uart_config);

    return ret;
}


void test1(void)
{
	printf("Hello Mavlink Debug!\n");

	// int test_uart = init_mavlink_uart("/dev/ttyACM0");

	// mavlink_status_t serial_status;
	// mavlink_message_t msg;

	// int chan = 0;
	// int bytesAvailable = 0;
	// uint8_t buffer[500] = "";
	// int count = 100;

	// msg.msgid = 0;
	// msg.seq = 0;
	// msg.compid = 0; 
	// msg.sysid = 0;

	// //bool MessageRecieved = false;

	// //char ch = 0;

	// //while(/*ch != '\n' || ch != '\r' ||*/ ch != ' ')
	// //while(!MessageRecieved)
	// while(count>0)
	// {
	// 	bytesAvailable = read(test_uart, buffer, 500);

	// 	if(bytesAvailable>0){
	// 		printf("bytesAvailable: %d\n",bytesAvailable);
	// 		for(int i = 0;i<bytesAvailable;i++){
	// 			printf("%d ",buffer[i]);

	// 			if (mavlink_parse_char(chan, buffer[i], &msg, &serial_status))
	// 			{
	// 				count--;
	// 				//MessageRecieved = true;
	// 				printf("\nmessage ID: %d, sequence: %d component id: %d system id %d", msg.msgid, msg.seq, msg.compid, msg.sysid);
	// 				//ch = getchar();
	// 				//printf("no wait\n");
	// 			}
	// 		}
	// 		printf("\n\n");
	// 	}
	// }

	// printf("done!\n");
	// close(test_uart);
}


void test2(void)
{
	printf("commander...sleep!\n");
}




int px4_mavlink_debug_main(int argc, char *argv[])
{
	printf("testes available:\n");
	printf("\ttest1: test mavlink message reception\n");
	printf("\ttest2: test navigator avec dataman\n");

	if(argc > 1){
		// if (!strcmp(argv[1], "test1")) {
		// 	test1();
		// }
		// else if(!strcmp(argv[1], "test2")){
		// 	test2();
		// }
	}
	
	return 0;
}


// int px4_mavlink_debug_main(int argc, char *argv[])
// {
// 	printf("Hello Debug!\n");

// 	/* advertise named debug value */
// 	struct debug_key_value_s dbg_key = { .key = "velx", .value = 0.0f };
// 	orb_advert_t pub_dbg_key = orb_advertise(ORB_ID(debug_key_value), &dbg_key);

// 	/* advertise indexed debug value */
// 	struct debug_value_s dbg_ind = { .ind = 42, .value = 0.5f };
// 	orb_advert_t pub_dbg_ind = orb_advertise(ORB_ID(debug_value), &dbg_ind);

// 	/* advertise debug vect */
// 	struct debug_vect_s dbg_vect = { .name = "vel3D", .x = 1.0f, .y = 2.0f, .z = 3.0f };
// 	orb_advert_t pub_dbg_vect = orb_advertise(ORB_ID(debug_vect), &dbg_vect);

// 	int value_counter = 0;

// 	while (value_counter < 100) {
// 		uint64_t timestamp_us = hrt_absolute_time();
// 		uint32_t timestamp_ms = timestamp_us / 1000;

// 		/* send one named value */
// 		dbg_key.value = value_counter;
// 		dbg_key.timestamp = timestamp_ms * 1e3f;
// 		orb_publish(ORB_ID(debug_key_value), pub_dbg_key, &dbg_key);

// 		/* send one indexed value */
// 		dbg_ind.value = 0.5f * value_counter;
// 		dbg_ind.timestamp = timestamp_ms * 1e3f;
// 		orb_publish(ORB_ID(debug_value), pub_dbg_ind, &dbg_ind);

// 		/* send one vector */
// 		dbg_vect.x = 1.0f * value_counter;
// 		dbg_vect.y = 2.0f * value_counter;
// 		dbg_vect.z = 3.0f * value_counter;
// 		dbg_vect.timestamp = timestamp_us;
// 		orb_publish(ORB_ID(debug_vect), pub_dbg_vect, &dbg_vect);

// 		warnx("sent one more value..");

// 		value_counter++;
// 		usleep(500000);
// 	}

// 	return 0;
// }