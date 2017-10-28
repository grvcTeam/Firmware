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
 * @file control_correction.c
 * Application to update topic control_correction with data received through UDP
 *
 * @author Manuel J. Fernadez <manfergonz@gmail.com>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <px4_config.h>
#include <px4_tasks.h>

#include <time.h>
#include <uORB/uORB.h>
#include <uORB/topics/control_correction.h>
#include <drivers/drv_hrt.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>
//----------------
#include<arpa/inet.h>
#include<sys/socket.h>
#include <unistd.h> // for close
#include "attitude.h" // struct and connection data

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */
//static struct param_handles ph;

/**
 * management function.
 */
__EXPORT int control_correction_main(int argc, char *argv[]);

// struct param_handles {
// 	float data; //typedef uint16_t	param_t;
// };

/**
 * Initialize all parameter handles and values
 *
 */
//int att_parameters_init(struct param_handles *h);

/**
 * Mainloop of daemon.
 */
int control_correction_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

void att_die(char *s);

void att_die(char *s)
{
    perror(s);
    exit(1);
}

// int att_parameters_init(struct param_handles *h)
// {
// 	/* PID parameters */
// 	h->data 	=	(float)1;
//
// 	return OK;
// }

static void usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

/**
 *
 */
int control_correction_main(int argc, char *argv[])
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
						 control_correction_thread_main,
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

int control_correction_thread_main(int argc, char *argv[])
{

	warnx("[daemon] starting\n");

	//att_parameters_init(&ph);

	struct sockaddr_in si_me, si_other;

	int s = sizeof(si_other);
	int recv_len;
	socklen_t slen;

	correctionValues msg;



	//create a UDP socket
	if ((s=socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)
	{
			att_die("socket");
	}

	// zero out the structure
	memset((char *) &si_me, 0, sizeof(si_me));

	si_me.sin_family = AF_INET;
	si_me.sin_port = htons(PORT);
	si_me.sin_addr.s_addr = htonl(INADDR_ANY);

	//bind socket to port
	if( bind(s , (struct sockaddr*)&si_me, sizeof(si_me) ) == -1)
	{
			att_die("bind");
	}

	int control_sub = orb_subscribe(ORB_ID(control_correction));

	struct control_correction_s correction;
	memset(&correction, 0, sizeof(correction));
	orb_advert_t correction_pub = orb_advertise(ORB_ID(control_correction), &correction);

	thread_running = true;

		while (!thread_should_exit) {
			struct control_correction_s raw;
			orb_copy(ORB_ID(control_correction), control_sub, &raw);

			warnx("Waiting for data...");
			fflush(stdout);

			//try to receive some data, this is a blocking call
			if ((recv_len = recvfrom(s, &msg, sizeof(correctionValues), 0, (struct sockaddr *) &si_other, &slen)) == -1)
			{
					att_die("recvfrom()");
			}

			//print details of the client/peer and the data received
			//warnx("Received packet from %s:%d\n", inet_ntoa(si_other.sin_addr), ntohs(si_other.sin_port));
		  //warnx("Data: %f %f %f %f\n" , (double)msg.roll, (double)msg.pitch,(double)msg.yaw,(double)msg.thrust);

      correction.rollCorrection = msg.roll;
		  correction.pitchCorrection = msg.pitch;
      correction.yawCorrection = msg.yaw;
      correction.thrustCorrection = msg.thrust;
			correction.timestamp = hrt_absolute_time();

			orb_publish(ORB_ID(control_correction), correction_pub, &correction);
		}

	warnx("[daemon] exiting.\n");
	close(s);
	thread_running = false;

	return 0;
}
