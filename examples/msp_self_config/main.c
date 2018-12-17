/*
 * Copyright (C) 2018 Robert Hartung <hartung@ibr.cs.tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */
#define CALIBRATION_SLEEP_TIME 1

#include <stdio.h>
#include <string.h>

#include "board.h"
#include "msg.h"
#include "thread.h"
#include "shell.h"
#include "shell_commands.h"
#include "xtimer.h"
#include "net/gnrc.h"
#include "net/gnrc/netreg.h"
#include "net/gnrc/netif/ieee802154.h"

#include "stdio_uart.h"

extern void msp430_init_dco(void);

char calib_thread_stack[512+256];

void *calib_thread(void *arg)
{
	int *intPoint = (int*)arg;
	printf("Got int: %d\n", *intPoint);
	while(1)
	{
		xtimer_sleep(CALIBRATION_SLEEP_TIME);
		msp430_init_dco();

		/*calibrate xtimer and stdio again*/
		xtimer_init();
		stdio_init();
	}
}

int main(void)
{
  puts("Welcome to RIOT! Starting Auto Calib");
	int i = 5;
	thread_create(calib_thread_stack, sizeof(calib_thread_stack), 1, THREAD_CREATE_STACKTEST, calib_thread, &i, "calib_thread");

	while(1)
	{
		xtimer_sleep(4);
		puts("Hello World");
	}

	
  return 0;
}



