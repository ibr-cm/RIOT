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

static char iv_thread_stack[THREAD_STACKSIZE_MAIN];

static struct {
	uint8_t is_running;
	uint8_t vreg;
} iv_state;

void debug_print_res(iv_res_t *res)
{
	printf("Response Frame:\n"
			"    osccal  = %u\n"
			"    voltage = %u\n"
			"    dt      = %u\n"
			"    debug:\n"
			"        state = %d\n"
			"        table = %d\n"
			"        flags = %s%s%s\n",
			res->osccal, res->voltage,
			res->dt_l | (res->dt_h << 8),
			res->debug & (3),
			(res->debug >> 6) & (3),
			res->debug & (1 << 3) ? "TABLE_ENTRY ": "",
			res->debug & (1 << 4) ? "HARDWARE_RESET ": "",
			res->debug & (1 << 5) ? "SOFTWARE_RESET ": "");
}

int wait_si_ready(void)
{
	uint8_t si_state;
	uint8_t i;
	i = 0;
	do {
		++i;
		i2c_acquire(SI_I2C_DEV);
		i2c_read_reg(SI_I2C_DEV, SI_I2C_ADDR,
				SI_REG_LOCK, &si_state);
		i2c_release(SI_I2C_DEV);
		xtimer_usleep(100);
	} while (si_state != SI_READY);
	return i;
}

int send_si_req(iv_req_t *req, iv_res_t *res)
{
	i2c_acquire(SI_I2C_DEV);
	i2c_write_regs(SI_I2C_DEV, SI_I2C_ADDR,
			SI_REG_REQUEST, req, sizeof(*req));
	i2c_release(SI_I2C_DEV);
	wait_si_ready();
	i2c_acquire(SI_I2C_DEV);
	i2c_read_regs(SI_I2C_DEV, SI_I2C_ADDR,
			SI_REG_REPLY, res, sizeof(*res));
	i2c_release(SI_I2C_DEV);
	return 0;
}

void *iv_thread(void *arg)
{
	(void) arg;

	static ad5242_t ad5242_dev;
	iv_req_t req;
	iv_res_t res;

	ad5242_init(&ad5242_dev, &ad5242_params);
	req.alt_byte = 0;
	wait_si_ready();

	xtimer_ticks32_t last_wakeup = xtimer_now();
	while (1) {
		xtimer_periodic_wakeup(&last_wakeup, 1000000);
		if (!iv_state.is_running)
			continue;
		req.checksum = alu_check((uint8_t) 1212987413.12);
		req.temperature = 0x15;
		req.osccal = 0xFF;
		req.rst_flags = 0x00;
		req.alt_byte ^= 1;
		req.rst_disable = 0x00;
		send_si_req(&req, &res);
		iv_state.vreg = res.voltage;
		debug_print_res(&res);
		ad5242_set_reg(&ad5242_dev, res.voltage);
	}

	return NULL;
}

void idealvolting_init(void)
{
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
	printf("Vreg       = %d\n", iv_state.vreg);
}
