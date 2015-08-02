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
 * @file px4_daemon_app.c
 * daemon application example for PX4 autopilot
 *
 * @author Martin Jaime <jaimem5@unlv.nevada.edu>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <poll.h>

#include <px4_config.h>
#include <nuttx/sched.h>
#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

static bool thread_should_exit = false; /* daemon exit flag */ 
static bool thread_running = false;     /* daemon status flag */ 
static bool should_poll = false;        /* whether user requests poll */
static int daemon_task;                 /* handle of daemon task/thread */ 

/* Daemon management function. */ 
__EXPORT int mrtn_px4_daemon_app_main(int argc, char *argv[]);

/* Main loop of daemon. */ 
int px4_daemon_thread_main(int argc, char *argv[]);

/* To print correct usage */ 
static void usage(const char *reason)
{
    if (reason) {
        warnx("%s", reason);
    }
    
    warnx("usage: daemon {start|stop|status|poll} [-p <additional params>]\n");
}

int mrtn_px4_daemon_app_main(int argc, char *argv[]) 
{
    if (argc < 2) {
        usage("missing command\n");
        return 1;
    }

    if (!strcmp(argv[1], "start"))
    {
        if (thread_running)
        {
            warnx("Daemon already running\n");
            exit(0); /* This is not an error */ 
        } 

        thread_should_exit = false; 
        daemon_task = px4_task_spawn_cmd("daemon", 
            SCHED_DEFAULT,
            SCHED_PRIORITY_DEFAULT,
            2000,
            px4_daemon_thread_main,
            (argv) ? (char * const *)&argv[2] : (char * const *)NULL);

        return 0;
    }

    if (!strcmp(argv[1], "stop")) 
    {
        thread_should_exit = true;
        return 0; 
    }

    if (!strcmp(argv[1], "status")) 
    {
        if (thread_running)
            warnx("\trunning\n");
        else 
            warnx("\tnot started\n");
        return 0;
    }

    if (!strcmp(argv[1], "poll"))
    {
        should_poll = true;
        return 0;
    }

    usage("Unrecognized command\n");

    return 1; 
}

int px4_daemon_thread_main(int argc, char *argv[])
{
    int llsensor_sub_fd;
    int poll_ret;
    int error_count = 0;
    int sleep_time = 8;
    struct distance_sensor_s raw;
    warnx("[sample daemon] starting\n");

    thread_running = true; 

    /* Subscribe to distance_sensor topic */ 
    llsensor_sub_fd = orb_subscribe(ORB_ID(distance_sensor));
    // orb_set_interval(llsensor_sub_fd, 1000); /* for a second */ 

    /* Wait for topic. */ 
    struct pollfd fds[] = {
        {.fd = llsensor_sub_fd, .events = POLLIN},
    };

    while(!thread_should_exit) {
        if (should_poll)
        {
            poll_ret = poll(fds, 1, 1000); /* poll for a second */ 

            /* Handling the return of poll() */ 
            if (poll_ret == 0)  
                printf("No data from providers within a second\n");
            else if (poll_ret < 0) {
                if (error_count < 10 || error_count % 50 == 0) 
                    /* To avoid flooding the console with error messages */ 
                    printf("ERROR returned from poll(): %d\n", poll_ret);
                error_count++;
            } else {
                if (fds[0].revents & POLLIN) {
                    /* Obtained data! */ 
                    orb_copy(ORB_ID(distance_sensor), llsensor_sub_fd, &raw);
                    printf("[mrtn_px4_daemon_app] Distance read: %.3f\n",
                            (double)raw.current_distance);
                }
                /* 
                 * More file descriptor checks can be added here
                 * if (fds[1..n].revents & POLLIN) {}
                 */
            }
            should_poll = false;
        }
        printf("Sleeping for %d seconds\n", sleep_time);
        sleep(sleep_time);
    }

    warnx("[sample daemon] exiting.\n");
    thread_running = false;

    return 0; 
}
