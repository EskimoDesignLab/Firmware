/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file run_cam.c
 * Control for RunCam Split
 *
 * @author Adriatik Sermaxhaj <adri.sermax@gmail.com>
 * 
 */
#include <strings.h>
#include <px4_config.h>
#include <px4_workqueue.h>
#include <sys/types.h>

#include <px4_posix.h>
#include <drivers/drv_hrt.h>
#include <termios.h>

#include <uORB/uORB.h>
#include <uORB/topics/camera_trigger.h>

static const int RCSPLIT_PACKET_HEADER      = 0x55;
static const int RCSPLIT_PACKET_CMD_CTRL    = 0x01;
static const int RCSPLIT_PACKET_TAIL        = 0xaa;


class RC_Spit
{
    public:

    RC_Spit();
	~RC_Spit();


    int                     rcSplitInit         (char *uart_name);
    int                     RunCamStateMachine  (char* cmd);



    private:

    enum rcsplitState_e {
        RCSPLIT_STATE_UNKNOWN = 0,
        RCSPLIT_STATE_INITIALIZING,
        RCSPLIT_STATE_IS_READY,
        RCSPLIT_STATE_VIDEO_STOPPED,
        RCSPLIT_STATE_VIDEO_STARTED,
        RCSPLIT_STATE_PHOTO,
        RCSPLIT_STATE_SETUP,
    };

    // the commands of RunCam Split serial protocol
    enum rcsplit_ctrl_argument_e {
        RCSPLIT_CTRL_ARGU_INVALID = 0x0,
        RCSPLIT_CTRL_ARGU_WIFI_BTN = 0x1,
        RCSPLIT_CTRL_ARGU_POWER_BTN = 0x2,
        RCSPLIT_CTRL_ARGU_CHANGE_MODE = 0x3,
        RCSPLIT_CTRL_ARGU_WHO_ARE_YOU = 0xFF,
    };

    bool            topic_initialized;
    rcsplitState_e  _oldState;
    int             _uart_fd;
    work_s		    _work;
    int             camera_trigger_sub_fd;
    bool            new_data_camera_trigger;

    struct camera_trigger_s _camera_trigger_s {};

    void                    sendCtrlCommand     (int port,rcsplit_ctrl_argument_e argument);
    uint8_t                 crc_high_first      (uint8_t *ptr, uint8_t len);
    int                     enable_flow_control (bool enabled,int uart_fd);
    int                     set_baud            (int uart_fd, char *device);
    static void             usage               (void);
    static void             RC_trampoline       (void *arg);
    void                    RC_task             (void);

};

RC_Spit::
RC_Spit()
{
    _uart_fd = -1;
	_oldState = RCSPLIT_STATE_VIDEO_STARTED;
    topic_initialized = false;
    camera_trigger_sub_fd = -1;

    memset(&_camera_trigger_s, 0, sizeof(_camera_trigger_s));

    memset(&_work, 0, sizeof(_work));
}

RC_Spit::
~RC_Spit()
{
	close(_uart_fd);
}

uint8_t 
RC_Spit::crc_high_first(uint8_t *ptr, uint8_t len)
{
    uint8_t i;
    uint8_t crc=0x00;
    while (len--) {
        crc ^= *ptr++;
        for (i=8; i>0; --i) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x31;
            else
                crc = (crc << 1);
        }
    }
    return (crc);
}

int 
RC_Spit::RunCamStateMachine(char* cmd){

    rcsplit_ctrl_argument_e split_cmd = RCSPLIT_CTRL_ARGU_WHO_ARE_YOU;
    rcsplitState_e newState = _oldState;


    if (_uart_fd < 0) {
        printf("ERROR opening UART!\n");
        return -1;
    }

    if (!strcmp(cmd, "rec")){
        newState = RCSPLIT_STATE_VIDEO_STARTED;
        printf("cmd:rec\n");
    }else if(!strcmp(cmd, "nrec")){
        newState = RCSPLIT_STATE_VIDEO_STOPPED;
        printf("cmd:stop\n");
    }else if(!strcmp(cmd, "snap")){
        newState = RCSPLIT_STATE_PHOTO;
        printf("cmd:snap\n");
    }else if (!strcmp(cmd, "wifi")) {
        sendCtrlCommand(_uart_fd,RCSPLIT_CTRL_ARGU_WIFI_BTN);

    }else if (!strcmp(cmd, "mode")) {
        sendCtrlCommand(_uart_fd,RCSPLIT_CTRL_ARGU_CHANGE_MODE);

    }else if (!strcmp(cmd, "who")) {
        sendCtrlCommand(_uart_fd,RCSPLIT_CTRL_ARGU_WHO_ARE_YOU);

    }else if (!strcmp(cmd, "info")) {
        usage();
        printf("camera_trigger.seq = %d \n",_camera_trigger_s.seq);
        return 0;
    }else{
        return 0;
    }


    if(newState == RCSPLIT_STATE_VIDEO_STOPPED && _oldState == RCSPLIT_STATE_VIDEO_STARTED){

        split_cmd = RCSPLIT_CTRL_ARGU_POWER_BTN;
        sendCtrlCommand(_uart_fd,split_cmd);
        sleep(5);

    }else if(newState == RCSPLIT_STATE_PHOTO && _oldState == RCSPLIT_STATE_VIDEO_STARTED){

        split_cmd = RCSPLIT_CTRL_ARGU_POWER_BTN;
        sendCtrlCommand(_uart_fd,split_cmd);
        sleep(5);

        split_cmd = RCSPLIT_CTRL_ARGU_CHANGE_MODE;
        sendCtrlCommand(_uart_fd,split_cmd);

        //split_cmd = RCSPLIT_CTRL_ARGU_POWER_BTN;
        //sendCtrlCommand(_uart_fd,split_cmd);
    }else if(newState == RCSPLIT_STATE_VIDEO_STARTED && _oldState == RCSPLIT_STATE_VIDEO_STOPPED){

        split_cmd = RCSPLIT_CTRL_ARGU_POWER_BTN;
        sendCtrlCommand(_uart_fd,split_cmd);

    }else if(newState == RCSPLIT_STATE_PHOTO && _oldState == RCSPLIT_STATE_VIDEO_STOPPED){

        split_cmd = RCSPLIT_CTRL_ARGU_CHANGE_MODE;
        sendCtrlCommand(_uart_fd,split_cmd);

        //split_cmd = RCSPLIT_CTRL_ARGU_POWER_BTN;
        //sendCtrlCommand(_uart_fd,split_cmd);
    }else if(newState == RCSPLIT_STATE_VIDEO_STARTED && _oldState == RCSPLIT_STATE_PHOTO){

        split_cmd = RCSPLIT_CTRL_ARGU_CHANGE_MODE;
        sendCtrlCommand(_uart_fd,split_cmd);

        split_cmd = RCSPLIT_CTRL_ARGU_CHANGE_MODE;
        sendCtrlCommand(_uart_fd,split_cmd);

        split_cmd = RCSPLIT_CTRL_ARGU_POWER_BTN;
        sendCtrlCommand(_uart_fd,split_cmd);

    }else if(newState == RCSPLIT_STATE_VIDEO_STOPPED && _oldState == RCSPLIT_STATE_PHOTO){

        split_cmd = RCSPLIT_CTRL_ARGU_CHANGE_MODE;
        sendCtrlCommand(_uart_fd,split_cmd);

        split_cmd = RCSPLIT_CTRL_ARGU_CHANGE_MODE;
        sendCtrlCommand(_uart_fd,split_cmd);

    }else if(newState == RCSPLIT_STATE_PHOTO && _oldState == RCSPLIT_STATE_PHOTO){
        split_cmd = RCSPLIT_CTRL_ARGU_POWER_BTN;
        sendCtrlCommand(_uart_fd,split_cmd);
    }

    _oldState = newState;

    if(newState == RCSPLIT_STATE_PHOTO){
        printf("State: SNAP\n");
    }else if(newState == RCSPLIT_STATE_VIDEO_STOPPED){
        printf("State: STOPPED\n");
    }else if(newState == RCSPLIT_STATE_VIDEO_STARTED){
        printf("State: STARTED\n");
    }

    return _uart_fd;
}


void 
RC_Spit::sendCtrlCommand(int port,rcsplit_ctrl_argument_e argument)
{
    if (!port)
        return ;

    uint8_t uart_buffer[5] = {0};
    uint8_t crc = 0;

    uart_buffer[0] = RCSPLIT_PACKET_HEADER;
    uart_buffer[1] = RCSPLIT_PACKET_CMD_CTRL;
    uart_buffer[2] = argument;
    uart_buffer[3] = RCSPLIT_PACKET_TAIL;
    crc = crc_high_first(uart_buffer, 4);

    // build up a full request [header]+[command]+[argument]+[crc]+[tail]
    uart_buffer[3] = crc;
    uart_buffer[4] = RCSPLIT_PACKET_TAIL;


    if(argument == RCSPLIT_CTRL_ARGU_CHANGE_MODE){
        printf("commande envoye: MODE\n");
    }else if(argument == RCSPLIT_CTRL_ARGU_POWER_BTN){
        printf("commande envoye: POWER\n");
    }

    // write to device
    write(port, uart_buffer, 5);

    sleep(1);
}


int 
RC_Spit::rcSplitInit(char *uart_name)
{
	// assuming NuttShell is on UART1 (/dev/ttyS0) /
	_uart_fd = open(uart_name, O_RDWR | O_NONBLOCK | O_NOCTTY); //

	if (_uart_fd < 0) {
		printf("ERROR opening UART %s, aborting..\n", uart_name);
		return _uart_fd;

	} else {
		printf("Writing to UART %s\n", uart_name);
	}

    _uart_fd = set_baud(_uart_fd,uart_name);
    if (_uart_fd < 0) {
        return _uart_fd;
    }

    //cameraState = RCSPLIT_STATE_IS_READY;

    work_queue(LPWORK, &_work, (worker_t)&RC_trampoline, this, 100);

    return _uart_fd;
}


int 
RC_Spit::set_baud(int uart_fd, char *_device)
{
    // set baud rate
    int speed = B115200;
    struct termios uart_config;
    tcgetattr(uart_fd, &uart_config);
    // clear ONLCR flag (which appends a CR for every LF)
    uart_config.c_oflag &= ~ONLCR;

    /* Set baud rate */
    if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
        warnx("ERR SET BAUD %s\n", _device);
        close(uart_fd);
        return -1;
    }

    if ((tcsetattr(uart_fd, TCSANOW, &uart_config)) < 0) {
        PX4_WARN("ERR SET CONF %s\n", _device);
        px4_close(uart_fd);
        return -1;
    }

    /* setup output flow control */
    if (enable_flow_control(false,uart_fd)) {
        PX4_WARN("hardware flow disable failed");
    }

    return uart_fd;
}

int 
RC_Spit::enable_flow_control(bool enabled,int uart_fd)
{
    struct termios uart_config;

    int ret = tcgetattr(uart_fd, &uart_config);

    if (enabled) {
        uart_config.c_cflag |= CRTSCTS;

    } else {
        uart_config.c_cflag &= ~CRTSCTS;
    }

    ret = tcsetattr(uart_fd, TCSANOW, &uart_config);

    /*
    if (!ret) {
        _flow_control_enabled = enabled;
    }
    */

    return ret;
}

void
RC_Spit::RC_trampoline(void *arg)
{
	RC_Spit *rc = (RC_Spit *)arg;

	rc->RC_task();
}

void
RC_Spit::RC_task(void)
{
    if (!topic_initialized) {
		camera_trigger_sub_fd = orb_subscribe(ORB_ID(camera_trigger));
		orb_set_interval(camera_trigger_sub_fd, 100);

		topic_initialized = true;
	}


    orb_check(camera_trigger_sub_fd, &new_data_camera_trigger);

    if (new_data_camera_trigger) {
        orb_copy(ORB_ID(camera_trigger), camera_trigger_sub_fd, &_camera_trigger_s);
        char cmd[] = "mode"; // for test only: should be "snap": 
        RunCamStateMachine(cmd);
    } 
}


static void 
RC_Spit::usage(void)
{
	printf("usage: run_cam {start|stop|rec|nrec|snap|power|mode|who}\n");
    printf("usage: start : start a task for trigger");
    printf("usage: stop  : stop the task for trigger\n");
    printf("usage: rec   : start recording\n");
    printf("usage: nrec  : stop recording\n");
    printf("usage: wifi  : start/stop wifi\n");
    printf("usage: snap  : take a photo\n");
    printf("usage: mode  : change mode (photo/video/settings)\n");
    printf("usage: who   : return uart feedback\n\n");
    printf("usage: the second argument is used for uart device name \n");
    printf("Example: run_cam start /dev/ttyS6 \n");
}


namespace
{
    RC_Spit *p_rcsplit;
}

extern "C" __EXPORT int run_cam_main(int argc, char *argv[]);

/*
run_cam who
run_cam wifi
run_cam power
run_cam mode
*/

int run_cam_main(int argc, char *argv[])
{
    if(argc>1){
        if (!strcmp(argv[1], "start")) {
            if(p_rcsplit == nullptr){
                p_rcsplit = new RC_Spit();

                if (p_rcsplit == nullptr) {
                    printf("new failed\n");
                    return 1;
                }
            }else{
                printf("Already started!\n");
            }
        }else if(!strcmp(argv[1], "stop")){
            if(p_rcsplit != nullptr){
                delete p_rcsplit;
                p_rcsplit = nullptr;
            }else{
                printf("Run_cam is not started!\n");
            }
        }else{
            if(p_rcsplit != nullptr){
                int device_uart = -1;

                if(argv[2] == nullptr){
                    char default_dev[] = "/dev/ttyS6";
                    device_uart = p_rcsplit->rcSplitInit(default_dev);
                }else{
                    device_uart = p_rcsplit->rcSplitInit(argv[2]);
                }

                if(device_uart<0){
                    PX4_INFO("problem with uart initialisation!\n");
                }else{
                    device_uart = p_rcsplit->RunCamStateMachine(argv[1]);
                }
            }else{
            printf("run_cam is not started!\n"); 
            }
        }
    }else{
        RC_Spit::usage();
    }

	return 0;
}