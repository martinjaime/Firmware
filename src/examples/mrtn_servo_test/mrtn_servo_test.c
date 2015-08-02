/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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
 * @file mrtn_servo_test.c
 * 
 * App to test main pwm outputs. 
 *
 * @author Martin Jaime <jaimem5@unlv.nevada.edu>
 */

#include <px4_config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h> // for getopt()
#include <fcntl.h>
#include <poll.h>
#include <sys/mount.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

//#include <nuttx/mtd.h>
//#include <nuttx/fs/nxffs.h>
#include <nuttx/fs/ioctl.h>

#include <arch/board/board.h>

#include "systemlib/systemlib.h"
#include "systemlib/err.h"
#include "drivers/drv_pwm_output.h"

//static void	usage(const char *reason);
__EXPORT int	mrtn_servo_test_main(int argc, char *argv[]);

int mrtn_servo_test_main(int argc, char *argv[])
{
    const char *dev = PWM_OUTPUT0_DEVICE_PATH;
    char *endptr;
    //char c;               // Input for exiting program. 
    int dev_fd;             // file descriptor for device.
	unsigned servo_count;
    int ret;                // Hold ret value from ioctl.
    unsigned pwm_value;
    unsigned channel = 7;
    unsigned i;             // Loop counter. 

    //if argc < 
    channel = strtoul(argv[1], &endptr, 0) - 1;
    pwm_value = argc > 1 ? strtoul(argv[2], &endptr, 0) : 1500;
    printf("Output to channel %u at %u\n", channel, pwm_value);

    dev_fd = open(dev, 0);
    if (dev_fd < 0) 
        err(1, "Opening %s failed.", dev);

	/* get the number of servo channels */
	ret = ioctl(dev_fd, PWM_SERVO_GET_COUNT, (unsigned long)&servo_count);
	if (ret != OK) {
		err(1, "PWM_SERVO_GET_COUNT");
	}

    /* 
     * BEGIN BY ARMING
     */ 

    /* Tell safety that its okay to disable it with the switch */ 
    ret = ioctl(dev_fd, PWM_SERVO_SET_ARM_OK, 0); 
    if (ret != OK) {
        err(1, "PWM_SERVO_SET_ARM_OK command failed");
    }

    /* Tell I/O that the system is armed */ 
    ret = ioctl(dev_fd, PWM_SERVO_ARM, 0);
    if (ret != OK) {
        err(1, "PWM_SERVO_ARM command failed");
    }
    warnx("Outputs armed");

    /* Get current servo values */ 
    struct pwm_output_values last_spos;
    for ( i = 0; i < servo_count; i++) { 
        ret = ioctl(dev_fd, PWM_SERVO_GET(i), (unsigned long)&last_spos.values[i]);
        if (ret != OK) {
            err(1, "PWM_SERVO_GET(%d) command failed.", i);
        }
    }

    /* 
     * PERFORM OUTPUT
     */

    /* Open console to directly grab CTRL-C signal */ 
    struct pollfd input_fds; 
    input_fds.fd = 0; // stdin  
    input_fds.events = POLLIN;

    // warnx("Press CTRL-C to abort.");
    printf("Enter a PWM value within 1k - 2k. Out of range to quit:\n"); 

    //while(pwm_value >= 1000 && pwm_value <= 2000) {
        /* Set PWM value on selected channel */
        ret = ioctl(dev_fd, PWM_SERVO_SET(channel), pwm_value);
        if (ret != OK) {
            err(1, "PWM_SERVO_SET(%u) command failed", channel);
        }

        //printf("pwm_value is %u at end of loop.\n", pwm_value);

        /* Abort on user request */
        ret = poll(&input_fds, 1, -1); 
        //if (ret > 0) { 
        //    read(0, &c, sizeof(c));
        //}//c == 0x03 || c == 0x63 || c == 'q' <- condition

        //usleep(2000);
        //pwm_value++; // DEBUGGING
    //}

    printf("pwm_value is %u at exit.\n", pwm_value);
    /* 
     * DISARM AND RESTORE
     */ 
    sleep(2);
    warnx("User abort");

    /* Restore pwm values. */ 
    printf("Reseting to %u\n", last_spos.values[channel]);
    ret = ioctl(dev_fd, PWM_SERVO_SET(channel), last_spos.values[channel]);
    if (ret != OK) {
        err(1, "PWM_SERVO_SET(%d) command failed at restore", channel);
    }
    
    /* disarm, but do not revoke the SET_ARM_OK flag */
    ret = ioctl(dev_fd, PWM_SERVO_DISARM, 0); 
    if (ret != OK) {
        err(1, "PWM_SERVO_DISARM command failed");
    }
    warnx("Outputs disarmed.");

    return 0; 
}
