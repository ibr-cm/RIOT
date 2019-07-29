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
#include "periph/pm.h"

#define ENABLE_DEBUG 	(0)
#include "debug.h"

#define IV_THREAD_PRIORITY 0
#define IV_THREAD_FLAGS THREAD_CREATE_STACKTEST
#define TICKS_TO_WAIT (1 * RTT_FREQUENCY)

enum {
	IV_ACTIVE,
	IV_SLEEPING, //does this means that iv is running or not?
	IV_READY
};

enum {
	MSG_PERIODIC = 0,
	MSG_I2C_W = 1,
	MSG_SLEEP = 2,
	MSG_WAKE = 3
};

static kernel_pid_t iv_thread_pid, sleeping_pid;
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
static uint8_t dozing = 0;
static volatile uint32_t last;

msg_t recievedMessage;

/**
 * @brief   cb for the rtt
 *
 * Is used by the rtt as callback. Sends a message to the iv thread and the sleeping thread every second
 *
 * @param[in] arg       not used
 */
void _rtt_cb(void *arg)
{
	(void) arg;

	last += TICKS_TO_WAIT;
	last &= RTT_MAX_VALUE;
	rtt_set_alarm(last, _rtt_cb, NULL);
	msg_t msg;
	msg.type = MSG_PERIODIC;
	msg_send(&msg, iv_thread_pid);
	if (dozing)
		msg_try_send(&msg, sleeping_pid);
}

/**
 * @brief   cb for recieving data in i2c slave mode
 *
 * is called, if you recieve data while in slave mode
 *
 * @param[in] n		??
 * @param[in] data	revieved data
 */

void _i2c_r_cb(uint8_t n, uint8_t *data)
{
	(void) n;
	if (n == 1) {
		msg_t msg;
		msg.type = MSG_I2C_W;
		msg.content.value = data[0];
		msg_send(&msg, iv_thread_pid);
	}
}

/**
 * @brief   cb for sending data in i2c slave mode
 *
 * is called, if you send data while in slave mode
 *
 * @param[in] data	you send
 */

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
	i2c_acquire(IV_I2C_DEV);
	do {
		i2c_read_reg(IV_I2C_DEV, SI_I2C_ADDR,
				SI_REG_LOCK, &si_state, 0x00);
		// Todo: Wait before trying again?
	} while (si_state != SI_READY);
	i2c_release(IV_I2C_DEV);
}

/**
 * @brief   Sends Data to the attiny and recieves answer
 *
 * Sends the required data to the attiny and recieves information from it. 
 *
 * @param[in] req       data for attiny
 * @param[out] res      data from attiny
 */

void send_si_req(iv_req_t *req, iv_res_t *res) //si = secure instance = attiny
{
	req->checksum = MCU_CHECK(); /* is alu_check() from alu_check.h . This define is set in idealvolting_config.h */
	req->temperature = get_temp();
	iv_state.temp = req->temperature;
	req->osccal = OSCCAL;
	req->alt_byte ^= 1;

	i2c_acquire(IV_I2C_DEV);
	i2c_write_regs(IV_I2C_DEV, SI_I2C_ADDR,
			SI_REG_REQUEST, req, sizeof(*req), 0x00);
	i2c_release(IV_I2C_DEV);
	wait_si_ready();
	i2c_acquire(IV_I2C_DEV);
	i2c_read_regs(IV_I2C_DEV, SI_I2C_ADDR,
			SI_REG_REPLY, res, sizeof(*res), 0x00);
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
	i2c_acquire(IV_I2C_DEV);
	i2c_init(IV_I2C_DEV);
	i2c_release(IV_I2C_DEV);
}

void _iv_deepsleep(uint8_t t_max)
{
	msg_t msg;
	msg.type = MSG_SLEEP;
	msg.content.value = t_max;
	msg_send(&msg, iv_thread_pid);
}

/**
 * @brief   Managing Thread for idealvolting
 *
 * This thread manages idealvolting
 *
 */

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
			i2c_acquire(IV_I2C_DEV);
			i2c_init(IV_I2C_DEV);
			i2c_release(IV_I2C_DEV);
			last = TICKS_TO_WAIT;
			rtt_set_alarm(TICKS_TO_WAIT, _rtt_cb, NULL);
			iv_state.running = IV_ACTIVE;
			msg_send(&msg, sleeping_pid); //send message to sleeping thread to tell him how long we are gonna sleep
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

/**
 * @brief   starts idealvolting
 *
 * This starts up idealvolting. It will create the Thread necessary to manage the communication between the MCU and the ATTiny on the reaper
 *
 */

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

/**
 * @brief   wakes up idealvolting
 *
 * This reactivates idealvolting if it is currently not active
 *
 */

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

/**
 * @brief   sets idealvolting to sleep
 *
 * This deactivates idealvolting for a given period of seconds. Idealvolting is started automatically at the beginning of the iv Thread. Deactivation of iv may result in increased power consumption.
 *
 * @param[in] duration      seconds idealvolting should sleep
 * 
 * Should return the recieved message if it is not from iv
 */
void* idealvolting_sleep(uint8_t duration)
{
	DEBUG("[DEBUG] Idealvolting is sleeping for %d seconds.\n", duration);

	uint8_t valid;
	mutex_lock(&iv_mutex);
	valid = (iv_state.table & 3);
	mutex_unlock(&iv_mutex);
	sleeping_pid = thread_getpid();
	if (valid) {
		DEBUG("[DEBUG] valid bit set.\n");
		_iv_deepsleep(duration);
		msg_receive(&recievedMessage);	/* This should be send by the iv_thread */
		if(recievedMessage.sender_pid == iv_thread_pid)
		{
			duration = recievedMessage.content.value;
			DEBUG("[DEBUG] Duration was set to %d seconds.\n", duration);
		} else {
			DEBUG("[DEBUG] IV was woken up due to a recieved message!\n");
			idealvolting_wakeup(); /* wakeup iv */
			return &recievedMessage;
		}
	}
	dozing = 1;
	/*
	#ifdef BOARD_REAPER
		pm_block(PM_SLEEPMODE_PWR_SAVE);
	#endif
	*/
	#if 0
	pm_block(PM_SLEEPMODE_ADC);
	#endif
	while (duration--) {
		//__builtin_avr_delay_cycles(8000000);
		msg_receive(&recievedMessage);
		if(recievedMessage.type != MSG_PERIODIC)
		{
			/*message is not from iv! wakeup iv!*/
			DEBUG("[DEBUG] IV was woken up due to a recieved message!\n");
			idealvolting_wakeup();
			return &recievedMessage;
		} 		
 	}
	#if 0
	pm_unblock(PM_SLEEPMODE_ADC);
	#endif
	/*
	#ifdef BOARD_REAPER
		pm_unblock(PM_SLEEPMODE_PWR_SAVE);
	#endif
	*/
	dozing = 0;
	DEBUG("[DEBUG] IV Sleep End\n");
	return NULL;
}

/**
 * @brief  	prints information about idealvolting
 *
 * Prints some information about idealvolting.
 *
 */
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

int8_t idealvolting_get_temp(void) {
    return iv_state.temp;
}
