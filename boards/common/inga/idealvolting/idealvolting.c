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
#include <assert.h>
#include <mutex.h>

#define IV_THREAD_PRIORITY 0
#define IV_THREAD_FLAGS THREAD_CREATE_STACKTEST

static char iv_thread_stack[THREAD_STACKSIZE_MAIN];
static mutex_t iv_mutex, iv_disable;
static struct {
	uint8_t is_running;
	uint8_t debug;
	uint8_t vreg;
	uint8_t osccal;
	uint8_t temp;
	uint8_t table;
} iv_state;

int8_t get_temp(void)
{
	uint8_t data;
	i2c_acquire(IV_I2C_DEV);
	i2c_read_reg(IV_I2C_DEV, TMP_ADDR, TMP_REG, &data);
	i2c_release(IV_I2C_DEV);
	return data;
}

int wait_si_ready(void)
{
	uint8_t si_state;
	uint8_t i;
	i = 0;
	do {
		++i;
		i2c_acquire(IV_I2C_DEV);
		i2c_read_reg(IV_I2C_DEV, SI_I2C_ADDR,
				SI_REG_LOCK, &si_state);
		i2c_release(IV_I2C_DEV);
		xtimer_usleep(100);
	} while (si_state != SI_READY);
	return i;
}

void send_si_req(iv_req_t *req, iv_res_t *res)
{
	i2c_acquire(IV_I2C_DEV);
	i2c_write_regs(IV_I2C_DEV, SI_I2C_ADDR,
			SI_REG_REQUEST, req, sizeof(*req));
	i2c_release(IV_I2C_DEV);
	wait_si_ready();
	i2c_acquire(IV_I2C_DEV);
	i2c_read_regs(IV_I2C_DEV, SI_I2C_ADDR,
			SI_REG_REPLY, res, sizeof(*res));
	i2c_release(IV_I2C_DEV);
	assert(res->osccal >= IV_OSCCAL_MIN && res->osccal <= IV_OSCCAL_MAX);
}

void prepare_si_req(iv_req_t *req) {
	req->checksum = MCU_CHECK();
	req->temperature = get_temp();
	iv_state.temp = req->temperature;
	req->osccal = OSCCAL;
	req->rst_flags = 0x00;
	req->alt_byte ^= 1;
	req->rst_disable = 0x00;
}

void *iv_thread(void *arg)
{
	(void) arg;

	mutex_init(&iv_mutex);
	static ad5242_t ad5242_dev;
	iv_req_t req;
	iv_res_t res;

	ad5242_init(&ad5242_dev, &ad5242_params);
	req.alt_byte = 0;
	wait_si_ready();

	xtimer_ticks32_t last_wakeup = xtimer_now();
	while (1) {
		xtimer_periodic_wakeup(&last_wakeup, 1000000);
		mutex_lock(&iv_mutex);
		mutex_lock(&iv_disable);
		mutex_unlock(&iv_disable);
		prepare_si_req(&req);
		send_si_req(&req, &res);
		iv_state.vreg = res.voltage;
		iv_state.osccal = res.osccal;
		iv_state.table = (res.debug >> 6) & (3);
		ad5242_set_reg(&ad5242_dev, res.voltage);
		OSCCAL = res.osccal;
		mutex_unlock(&iv_mutex);
	}

	return NULL;
}

void idealvolting_init(void)
{
	iv_state.is_running = 0;
	iv_state.debug = 0;

	i2c_acquire(IV_I2C_DEV);
	i2c_init_master(IV_I2C_DEV, SI_I2C_SPEED);
	i2c_release(IV_I2C_DEV);

	thread_create(iv_thread_stack, sizeof(iv_thread_stack),
			IV_THREAD_PRIORITY, IV_THREAD_FLAGS,
			iv_thread, NULL, "idealvolting");

	iv_state.is_running = 1;
}

void idealvolting_enable(void)
{
	mutex_lock(&iv_mutex);
	mutex_unlock(&iv_disable);
	iv_state.is_running = 1;
	mutex_unlock(&iv_mutex);
}

void idealvolting_disable(void)
{
	mutex_lock(&iv_mutex);
	mutex_unlock(&iv_disable);
	iv_state.is_running = 0;
	mutex_unlock(&iv_mutex);
}

void idealvolting_set_debug(uint8_t state)
{
	mutex_lock(&iv_mutex);
	iv_state.debug = state;
	mutex_unlock(&iv_mutex);
}

void idealvolting_print_status(void)
{
	mutex_lock(&iv_mutex);
	puts("--------IdealVolting status--------");
	printf("is_running  = %s\n", iv_state.is_running ? "true" : "false");
	printf("Vreg        = %d\n", iv_state.vreg);
	printf("OSCCAL      = %d\n", iv_state.osccal);
	printf("Temperature = %d\n", iv_state.temp);
	printf("Table used  = %s\n", iv_state.table ? "true" : "false");
	mutex_unlock(&iv_mutex);
}
