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
#include "frame.h"
#include "alu_check.h"
#include "ad5242.h"
#include "ad5242_params.h"
#include "xtimer.h"
#include "thread.h"
#include "periph/i2c.h"
#include <stdio.h>
#include <stdint.h>

#define IV_THREAD_PRIORITY 0
#define IV_THREAD_FLAGS THREAD_CREATE_STACKTEST

static ad5242_t ad5242_dev;
static char iv_thread_stack[THREAD_STACKSIZE_MAIN];

static struct {
	uint8_t is_running;
} iv_state;

iv_res_t *send_si_req(iv_req_t *req)
{
	static iv_res_t res;
	uint8_t si_state;
	uint8_t i, n;
	i2c_acquire(SI_I2C_DEV);
	n = i2c_write_bytes(SI_I2C_DEV, SI_I2C_ADDR, req, sizeof(*req));
	printf("sent %d/%d bytes\n", n, sizeof(*req));
	i = 0;
	do {
		++i;
		i2c_release(SI_I2C_DEV);
		xtimer_usleep(1000);
		i2c_acquire(SI_I2C_DEV);
		n = i2c_read_reg(SI_I2C_DEV, SI_I2C_ADDR, SI_REG_LOCK, &si_state);
		printf("check busy: read %d bytes: %d\n", n, si_state);
	} while (si_state != SI_READY);
	printf("si done after %d tries - ", i);
	n = i2c_read_reg(SI_I2C_DEV, SI_I2C_ADDR, SI_REG_REPLY, &res);
	i2c_release(SI_I2C_DEV);
	printf("received %d/%d bytes\n", n, sizeof(res));
	return &res;
}

void *iv_thread(void *arg)
{
	(void) arg;

	iv_req_t req;
	iv_res_t *res;
	req.alt_byte = 0;
	xtimer_ticks32_t last_wakeup = xtimer_now();

	while (1) {
		xtimer_periodic_wakeup(&last_wakeup, 1000000);
		puts("!!!!!cycle-start!!!!!");
		if (!iv_state.is_running)
			continue;
		req.checksum = alu_check((uint8_t) 1212987413.12);
		req.temperature = 0x7F;
		req.osccal = 0xFF;
		req.rst_flags = 0x00;
		req.alt_byte ^= 1;
		req.rst_disable = 0x00;
		res = send_si_req(&req);
		(void) res;
	}

	return NULL;
}

void idealvolting_init(void)
{
	ad5242_init(&ad5242_dev, &ad5242_params);
	ad5242_set_reg(&ad5242_dev, IV_RESET_VREG);

	iv_state.is_running = 0;

	thread_create(iv_thread_stack, sizeof(iv_thread_stack),
			IV_THREAD_PRIORITY, IV_THREAD_FLAGS,
			iv_thread, NULL, "idealvolting");

	iv_state.is_running = 1;
}

void idealvolting_print_status(void)
{
	puts("--------IdealVolting status--------");
	printf("is_running = %s\n", iv_state.is_running ? "true" : "false");
	printf("Vreg       = %d\n", ad5242_get_reg(&ad5242_dev));
}
