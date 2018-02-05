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
#include "idealvolting_frame.h"
#include "alu_check.h"
#include "temp.h"
#include "thread.h"
#include "periph/i2c.h"
#include "periph/i2c_slave.h"
#include "periph/rtt.h"
#include "msg.h"
#include <stdio.h>
#include <assert.h>
#include <mutex.h>

#define IV_THREAD_PRIORITY 0
#define IV_THREAD_FLAGS THREAD_CREATE_STACKTEST
#define TICKS_TO_WAIT (1 * RTT_FREQUENCY)

enum {
	IV_ACTIVE,
	IV_SLEEPING,
	IV_READY
};

enum {
	MSG_PERIODIC = 0,
	MSG_I2C_W = 1,
	MSG_SLEEP = 2,
	MSG_WAKE = 3
};

static kernel_pid_t iv_thread_pid;
static char iv_thread_stack[THREAD_STACKSIZE_MAIN];
static mutex_t iv_mutex;
static struct {
	uint8_t running;
	uint8_t vreg;
	uint8_t osccal;
	uint8_t temp;
	uint8_t table;
} iv_state;
static uint8_t swr_detection = 0;
static volatile uint32_t last;

void _rtt_cb(void *arg)
{
	(void) arg;

	last += TICKS_TO_WAIT;
	last &= RTT_MAX_VALUE;
	rtt_set_alarm(last, _rtt_cb, NULL);

	msg_t msg;
	msg.type = MSG_PERIODIC;
	msg_send(&msg, iv_thread_pid);
}

void _i2c_r_cb(uint8_t n, uint8_t *data)
{
	if (n == 1 && data[0] == 'w') {
		msg_t msg;
		msg.type = MSG_I2C_W;
		msg_send(&msg, iv_thread_pid);
	}
}

uint8_t _i2c_t_cb(uint8_t *data)
{
	(void) data;
	return 0;
}

uint8_t check_reset(void)
{
	if (MCUSR & 0b00000001) { //Power On
		MCUSR &= 0b11111110;
		return 0;
	} else if (MCUSR & 0b00000010) { //HW Reset
		MCUSR &= 0b11111101;
		return 1;
	} else if (swr_detection) { //SW Reset
		return 2;
	}
	return 0;
}

void wait_si_ready(void)
{
	uint8_t si_state;
	do {
		i2c_acquire(IV_I2C_DEV);
		i2c_read_reg(IV_I2C_DEV, SI_I2C_ADDR,
				SI_REG_LOCK, &si_state);
		i2c_release(IV_I2C_DEV);
		// Todo: Wait before trying again?
	} while (si_state != SI_READY);
}

void send_si_req(iv_req_t *req, iv_res_t *res)
{
	req->checksum = MCU_CHECK();
	req->temperature = get_temp();
	iv_state.temp = req->temperature;
	req->osccal = OSCCAL;
	req->alt_byte ^= 1;

	i2c_acquire(IV_I2C_DEV);
	i2c_write_regs(IV_I2C_DEV, SI_I2C_ADDR,
			SI_REG_REQUEST, req, sizeof(*req));
	i2c_release(IV_I2C_DEV);
	wait_si_ready();
	i2c_acquire(IV_I2C_DEV);
	i2c_read_regs(IV_I2C_DEV, SI_I2C_ADDR,
			SI_REG_REPLY, res, sizeof(*res));
	i2c_release(IV_I2C_DEV);

	req->rst_flags = 0x00;
	req->rst_disable = 0x00;
	iv_state.vreg = res->voltage;
	iv_state.osccal = res->osccal;
	iv_state.table = (res->debug >> 6) & (3);
}

void _request_i2c_master(void)
{
	msg_t msg;
	i2c_stop_slave();
	i2c_init_slave(MEGA_SL_ADDR_READY, _i2c_r_cb, _i2c_t_cb);
	do {
		msg_receive(&msg);
	} while (msg.type != MSG_I2C_W);
	i2c_stop_slave();
	i2c_init_master(IV_I2C_DEV, SI_I2C_SPEED);
}

void *iv_thread(void *arg)
{
	(void) arg;
	
	static vscale_t vscale_dev;
	msg_t msg;
	iv_req_t req;
	iv_res_t res;

	_request_i2c_master();

	setup_temp();

	mutex_lock(&iv_mutex);
	req.alt_byte = 0;
	req.rst_flags = check_reset();
	swr_detection = 1;
	req.rst_disable = 0x00;
	VSCALE_INIT(&vscale_dev);
	wait_si_ready();
	iv_state.running = IV_ACTIVE;
	mutex_unlock(&iv_mutex);

	while (1) {
		msg_receive(&msg);
		mutex_lock(&iv_mutex);
		switch (msg.type) {
		case MSG_SLEEP:
			req.rst_disable = 0xff;
			req.rst_flags = msg.content.value;
			send_si_req(&req, &res);
			i2c_init_slave(MEGA_SL_ADDR_SLEEP, _i2c_r_cb, _i2c_t_cb);
			iv_state.running = IV_SLEEPING;
			rtt_poweroff();
			break;
		case MSG_WAKE:
			iv_state.running = IV_READY;
			_request_i2c_master();
			rtt_init();
			last = TICKS_TO_WAIT;
			rtt_set_alarm(TICKS_TO_WAIT, _rtt_cb, NULL);
			iv_state.running = IV_ACTIVE;
			break;
		case MSG_I2C_W:
			rtt_init();
			i2c_stop_slave();
			i2c_init_master(IV_I2C_DEV, SI_I2C_SPEED);
			last = TICKS_TO_WAIT;
			rtt_set_alarm(TICKS_TO_WAIT, _rtt_cb, NULL);
			iv_state.running = IV_ACTIVE;
			break;
		default:
			if (iv_state.running == IV_ACTIVE) {
				send_si_req(&req, &res);
				VSCALE_SET_REG(&vscale_dev, res.voltage);
			}
		}
		mutex_unlock(&iv_mutex);
	}

	return NULL;
}

void idealvolting_init(void)
{
	iv_state.running = IV_READY;

	mutex_init(&iv_mutex);
	rtt_init();

	iv_thread_pid = thread_create(iv_thread_stack, sizeof(iv_thread_stack),
			IV_THREAD_PRIORITY, IV_THREAD_FLAGS,
			iv_thread, NULL, "idealvolting");

	last = TICKS_TO_WAIT;
	rtt_set_alarm(TICKS_TO_WAIT, _rtt_cb, NULL);
}

void idealvolting_wakeup(void)
{
	uint8_t valid;
	mutex_lock(&iv_mutex);
	valid = (iv_state.running == IV_SLEEPING);
	mutex_unlock(&iv_mutex);
	if (!valid)
		return;
	msg_t msg;
	msg.type = MSG_WAKE;
	msg_send(&msg, iv_thread_pid);
}

void idealvolting_sleep(uint8_t duration)
{
	uint8_t valid;
	mutex_lock(&iv_mutex);
	valid = (iv_state.running == IV_ACTIVE);
	mutex_unlock(&iv_mutex);
	if (!valid)
		return;
	msg_t msg;
	msg.type = MSG_SLEEP;
	msg.content.value = duration;
	msg_send(&msg, iv_thread_pid);
}

void idealvolting_print_status(void)
{
	mutex_lock(&iv_mutex);
	puts("--------IdealVolting status--------");
	printf("running     = %s\n", iv_state.running ? "no" : "yes");
	printf("Vreg        = %d\n", iv_state.vreg);
	printf("OSCCAL      = %d\n", iv_state.osccal);
	printf("Temperature = %d\n", iv_state.temp);
	printf("Table used  = %s\n", iv_state.table ? "true" : "false");
	mutex_unlock(&iv_mutex);
}
