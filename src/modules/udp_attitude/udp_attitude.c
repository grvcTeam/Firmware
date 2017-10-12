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
 * @file udp_attitude.c
 * Application to send topcis data through UDP
 *
 * @author Manuel J. Fernadez <manfergonz@gmail.com>
 */

#include <px4_posix.h>
#include <poll.h>

#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/att_control.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <px4_config.h>
#include <px4_tasks.h>

#include <time.h>
#include <uORB/uORB.h>
#include <drivers/drv_hrt.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
//----------------
#include<arpa/inet.h>
#include<sys/socket.h>
#include <unistd.h> // for close
#include "param.h" // struct and connection data

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */
//static struct param_handles ph;

/**
 * management function.
 */
__EXPORT int udp_attitude_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int udp_attitude_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

void udp_die(char *s);

static void usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

void udp_die(char *s)
{
    perror(s);
    exit(1);
}

/**
 *
 */
int udp_attitude_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {
			warnx("daemon already running\n");
			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("daemon",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
						 udp_attitude_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");

		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int udp_attitude_thread_main(int argc, char *argv[])
{

	warnx("[daemon] starting\n");

  struct sockaddr_in si_other;
  int s, slen=sizeof(si_other);

	attitudeValues msg;

	// Create a UDP socket
	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
	{
			udp_die("socket");
	}

	// zero out the structure
	memset((char *) &si_other, 0, sizeof(si_other));

	si_other.sin_family = AF_INET;
	si_other.sin_port = htons(PORT);

  if (inet_aton(SERVER , &si_other.sin_addr) == 0)
  {
      fprintf(stderr, "inet_aton() failed\n");
      exit(1);
  }
  // -------------------
  /* subscribe to vehicle_attitude topic */
  int vehicle_attitude_sub_fd = orb_subscribe(ORB_ID(vehicle_attitude));
	int att_control_sub_fd = orb_subscribe(ORB_ID(att_control));

  /********************************************************************
   ********************************************************************
   *************erase next line to eliminate the limit rate************
   ********************************************************************
   ********************************************************************/
  //orb_set_interval(vehicle_attitude_sub_fd, 200);

  /* one could wait for multiple topics with this technique, just using one here */
  px4_pollfd_struct_t fds[] = {
    { .fd = vehicle_attitude_sub_fd,   .events = POLLIN },
		{ .fd = att_control_sub_fd,   .events = POLLIN },
        /* there could be more file descriptors here, in the form like:
         * { .fd = other_sub_fd,   .events = POLLIN },
         */
  };

  int error_counter = 0;

	thread_running = true;

	while (!thread_should_exit) {
    /* wait for sensor update of 2 file descriptors for 4 ms (250Hz) */
    int poll_ret = px4_poll(fds, 2, 4);

    /* handle the poll result */
    if (poll_ret == 0) {
        /* this means none of our providers is giving us data */
        //PX4_ERR("Got no data within a second");

    } else if (poll_ret < 0) {
        /* this is seriously bad - should be an emergency */
        if (error_counter < 10 || error_counter % 50 == 0) {
            /* use a counter to prevent flooding (and slowing us down) */
            PX4_ERR("ERROR return value from poll(): %d", poll_ret);
        }

        error_counter++;

    } else {
				// fds[1] to give priority to att_control update
        if ((fds[0].revents || fds[1].revents) & POLLIN) {
            /* obtained data for the first file descriptor */
            struct vehicle_attitude_s v_attitude;
						struct att_control_s att_control;
            /* copy sensors raw data into local buffer */
            orb_copy(ORB_ID(vehicle_attitude), vehicle_attitude_sub_fd, &v_attitude);
						orb_copy(ORB_ID(att_control), att_control_sub_fd, &att_control);
            /*PX4_INFO("Vehicle_attitude:\t%8.4f\t%8.4f\t%8.4f",
                 (double)v_attitude.rollspeed,
                 (double)v_attitude.pitchspeed,
                 (double)v_attitude.yawspeed);*/
            msg.roll_s = v_attitude.rollspeed;
            msg.pitch_s = v_attitude.pitchspeed;
            msg.yaw_s = v_attitude.yawspeed;
            for(int k=0;k<4;k++)
							msg.quat[k] = v_attitude.q[k];

						msg.att_control[0] = att_control.roll;
						msg.att_control[1] = att_control.pitch;
						msg.att_control[2] = att_control.yaw;

            if (sendto(s, &msg, sizeof(attitudeValues) , 0 , (struct sockaddr *) &si_other, slen)==-1)
            {
              udp_die("sendto()");
            }
        }
    }
	}

	warnx("[daemon] exiting.\n");
	close(s);
	thread_running = false;

	return 0;
}
