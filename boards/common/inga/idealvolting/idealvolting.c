/*
 * Copyright (C) 2017 Rasmus Antons <r.antons@tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     idealvolting
 * @{
 *
 * @file
 * @brief       Idealvolting implementation.
 *
 * @author      Rasmus Antons <r.antons@tu-bs.de>
 *
 * @}
 */

#include "idealvolting.h"
#include "idealvolting_config.h"
#include "ad5242.h"
#include "ad5242_params.h"
#include "xtimer.h"
#include "msg.h"
#include "thread.h"
#include <stdio.h>
#include <stdint.h>

#define IV_THREAD_PRIORITY 0
#define IV_THREAD_FLAGS THREAD_CREATE_STACKTEST

static ad5242_t ad5242_dev;
static char iv_thread_stack[THREAD_STACKSIZE_MAIN];

static struct {
	uint8_t is_running;
} idealvolting_state;

void *iv_thread(void *arg)
{
	(void) arg;
	msg_t m;
	xtimer_ticks32_t last_wakeup = xtimer_now();

	while (1) {
		xtimer_periodic_wakeup(&last_wakeup, 1000000);
		if (msg_try_receive(&m) == 1)
			printf("Got msg from %" PRIkernel_pid "\n", m.sender_pid);
		printf("idealvolting %ld\n", last_wakeup);
	}
	return NULL;
}

void idealvolting_init(void)
{
	ad5242_init(&ad5242_dev, &ad5242_params);
	ad5242_set_reg(&ad5242_dev, IDEALVOLTING_RESET_VREG);

	idealvolting_state.is_running = 0;

	thread_create(iv_thread_stack, sizeof(iv_thread_stack),
			IV_THREAD_PRIORITY, IV_THREAD_FLAGS,
			iv_thread, NULL, "idealvolting_thread");
}

void idealvolting_print_status(void)
{
	puts("--------IdealVolting status--------");
	printf("is_running = %s\n", idealvolting_state.is_running ? "true" : "false");
	printf("Vreg       = %d\n", ad5242_get_reg(&ad5242_dev));
}
