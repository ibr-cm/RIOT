/* Copyright (c) 2014, Ulf Kulau
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

/**
 * \mainpage
 *  ___    _            ___     __    _ _   _
 * |_ _|__| | ___  __ _| \ \   / /__ | | |_(_)_ __   __ _
 *  | |/ _` |/ _ \/ _` | |\ \ / / _ \| | __| | '_ \ / _` |
 *  | | (_| |  __/ (_| | | \ V / (_) | | |_| | | | | (_| |
 * |___\__,_|\___|\__,_|_|  \_/ \___/|_|\__|_|_| |_|\__, |
 *                                                   |___/
 *
 * \section about About
 * This program implements the secure instance (SI) which can be used to run controlled undervolting
 * mechanisms on a wireless sensor node. The basic idea is that the SI acts like an I2C-Slave
 * device. The SI gets information about the state of the main MCU via periodic I2C-frames.
 * The content of this frame is not defined. Nevertheless, we need a checksum to prove the
 * integrity of the main MCU. More information are available in related publications.
 *
 */

 /* Includes */
#include "tiny.h"
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/io.h>
#include "vtable.h"
#include "drv/adc-drv.h"
#include "drv/sw_uart.h"
#include "drv/temp.h"

static iv_req_t *req_buffer = (iv_req_t *) rxbuffer;
static uint8_t *lock_buffer = (uint8_t *) txbuffer;
static iv_res_t *res_buffer = (iv_res_t *) (txbuffer + 1);

uint8_t this_altbyte = 0;

volatile uint16_t this_delta_t = 0;
static int8_t prv_temperature = 71;  //previous temperature
static int8_t this_temperature;      //current temperature
static uint8_t this_sleeptime;
static uint8_t error;
static uint8_t startup, current_voltage, iteration;
static uint16_t delta_t = 55050;
static uint8_t state, next_state;
static uint8_t master = 0;

/* Debug counter */
static uint8_t alu_errors = 0;
static uint8_t rst_errors = 0;
static uint8_t sent_hello = 0;
static uint8_t temp_setup = 0;

///For deadlock reset
volatile uint8_t count_overflows = 0;
static uint8_t fact_default;

/* FUNCTION DECLARARTIONS */
void approach_voltage(uint8_t value);
void reset_mega(void);

/*
 * Reset voltage, set initial values, initialize timers, start UART and i2c
 */
void si_init(void)
{
	cli();
	SI_LOCK();

#ifdef BOARD_REAPER
	DDRA &=~(1<<PA2);   // VBAT_OK input test
	DDRA |= (1<<PA1);   // Enable i2c level shifter
	DDRA |= (1<<PA3);   // VOUT_EN buck enable test
	PORTA |= (1<<PA1);  // Enable i2c level shifter
	PORTA |= (1<<PA3);  // Enable buck
#endif

	/* Reset Voltage to normal level when SI is reset */
	SI_INIT_RESET_LINE();
	SI_PULL_RESET_LINE();
	_delay_ms(100);
	reset_voltage_level();
	_delay_ms(100);
	SI_RELEASE_RESET_LINE();

	sw_uart_init();
	_delay_ms(1000);

	/* Timer Initialization, Settings:
	 * Prescaler 8, Overflow Interrupt every ~0.5s
	 * (maybe prsc32 would be better?)
	 */
	TCCR1A = 0b00000000;
	TCCR1B = 0b00000011;
	TIMSK1 |= (1 << TOIE1);
	TCNT1 = 0;

	/* Enable external Watchdog */
	req_buffer->rst_disable = 0x00;
	startup = SI_STARTUP_DELAY;
	this_sleeptime = 0;
	iteration = 0;
	current_voltage = SI_VOLT_REG_OFFSET;
	init_table();

	master = 1;

	sei();
	req_buffer->alt_byte = 0;
	SI_UNLOCK();
}

void si_master(void)
{
	puts(REPORT_MASTER ":e");
	i2c_init_master();
	if (!temp_setup) {
		setup_temp();
		temp_setup = 1;
	}
	usi_twi_result_t result;
	uint8_t data = 'w';
	while (true) {
		_delay_ms(1000);
		// see if Mega woke up
		result = i2c_write_bytes(MEGA_SL_ADDR_READY, &data, 1);
		if (result == USI_TWI_SUCCESS) {
			puts(REPORT_MASTER ":w1");
			break;
		}
		if (startup) {
			--startup;
			continue;
		}
		// wake Mega up if timer expired
		if (0 == this_sleeptime) {
			puts(REPORT_MASTER ":w2");
			result = i2c_write_bytes(MEGA_SL_ADDR_SLEEP, &data, 1);
			if (result == USI_TWI_SUCCESS)
				break;
		} else {
			printf(REPORT_MASTER ":s %u\n", this_sleeptime);
		}
		this_sleeptime--;
		// if temperature changed adapt voltage
		result = get_temp(&this_temperature);
		if (result != USI_TWI_SUCCESS) {
			puts(REPORT_MASTER ":e1");
			SI_PULL_RESET_LINE();
			_delay_ms(200);
			SI_RELEASE_RESET_LINE();
			startup = SI_STARTUP_DELAY;
			continue;
		}
		this_temperature += SI_TEMP_OFFSET;
		current_index = ((uint8_t) (this_temperature) >> 1);
		if ((get_entry().info != VTABLE_VALUE_IS_EMPTY)) {
			if (current_voltage != get_entry().voltage) {
				approach_voltage(get_entry().voltage);
				uint8_t data[2] = {VSCALE_REG, current_voltage};
				i2c_write_bytes(VSCALE_ADDR, data, sizeof(data));
			}
			iteration = 0;
		// if new voltage not available wake Mega up
		} else {
			puts(REPORT_MASTER ":e2");
			result = i2c_write_bytes(MEGA_SL_ADDR_SLEEP, &data, 1);
			if (result == USI_TWI_SUCCESS)
				break;
		}
	};
	puts(REPORT_MASTER ":l");
	cli();
	i2c_slave_init(SI_I2C_ADDR << 1);
	TCNT1 = 0;
	sei();
	req_buffer->rst_disable = 0x00;
}

void si_frame_received(void)
{
	/* Lock SI and copy current alternating byte */
	SI_LOCK();
	this_altbyte = req_buffer->alt_byte;
	/* Reset watchdog*/
	count_overflows = 0;
	/* Readout OSCALL Value from man MCU.
	 * Get Temperature and add offset due to negative values */
	fact_default = req_buffer->osccal; //??
	this_temperature = req_buffer->temperature;
	this_temperature += SI_TEMP_OFFSET;
	/* DEBUG ONLY:*/
	SI_REPLY_DEBUG_RESET();
	SI_REPLY_DEBUG_STATE(next_state);
	/* Request Frame was received => switch to next_frame*/
	error = 0;
	state = next_state;
}

void approach_voltage(uint8_t value)
{
	if (value - current_voltage > SI_STEP)
		current_voltage += SI_STEP;
	else if (value - current_voltage < -SI_STEP)
		current_voltage -= SI_STEP;
	else
		current_voltage = value;
}

void si_transient(void)
{
	/* After startup the system is in transient state.
	 * Controlled undervolting is delayed by some cycles */
	if (startup > 0) {
		startup--;
		delta_t = DT_TARGET_SRC;
		TCNT1 = 0;
		res_buffer->dt = delta_t;
	} else {
		/* Backup default OSCALL value before controlled undervolting starts*/
		fact_default = req_buffer->osccal;
		prv_temperature = this_temperature;
		next_state = SI_MAIN;
	}
	/*If EEPROM data are available, the voltage potential 128->~200 is too heavy */
	current_index = ((uint8_t) (this_temperature) >> 1);
	if(get_entry().info != VTABLE_VALUE_IS_EMPTY)
		approach_voltage(table[(current_index)].voltage);
	/* Reply of OSCALL and Voltage Level*/
	res_buffer->osccal = req_buffer->osccal;
	res_buffer->voltage = current_voltage;
	/* If a reset occured, send this information to MCU*/
	if (req_buffer->rst_flags == MCU_HW_RESET) {
		SI_REPLY_DEBUG_RESET_HW();
	} else if (req_buffer->rst_flags == MCU_SW_RESET) {
		SI_REPLY_DEBUG_RESET_SW();
	}
	/* Unlock SI and get back to IDLE state*/
	SI_UNLOCK();
}

/*
 * Increase the voltage and write table entry if it doesn't exist yet.
 * Try to predict table values.
 */
void si_main_error(void)
{
	alu_errors++;
	if (alu_errors == 1) {
		current_voltage -= SI_DEFAULT_VOLT_OFFSET;
		create_table_entry(current_voltage, req_buffer->osccal);
		/* DEBUG ONLY*/
		SI_REPLY_DEBUG_TABLE_ENTRY();
	} else if (alu_errors == 3) {
		reset_mega();
	}
}

/*
 * No Checksum Error occurred
 *
 * If the temperature decreased by at least 2 degrees, increase the voltage.
 *
 * If a table entry exists, use that voltage.
 *
 * If no table value has been used for SI_ADAPTION_INTERVAL iterations,
 * decrease voltage level by ~15mV (increase vreg by 1).
 */
void si_main_no_error(void)
{
	alu_errors = 0;
	uint8_t temp_diff = this_temperature - prv_temperature;
	if (temp_diff <= -2) {
		current_voltage -= 5;
		prv_temperature = this_temperature;
	} else if (temp_diff >= 2) {
		prv_temperature = this_temperature;
	}

	if (get_entry().info != VTABLE_VALUE_IS_EMPTY) {
		approach_voltage(get_entry().voltage);
		res_buffer->osccal = get_entry().osccal;
		SI_REPLY_DEBUG_TABLE_USED(get_entry().info);
		iteration = 0;
	} else if (++iteration > SI_ADAPTION_INTERVAL) {
		current_voltage += 1;
		/* If absolute min. voltage level is reached => table entry*/
		if (current_voltage >= 254) {
			create_table_entry(current_voltage, req_buffer->osccal);
			SI_REPLY_DEBUG_TABLE_ENTRY();
			current_voltage = 254;
		}
		iteration = 0;
	}
}

/*
 * Main State: Controlled Undervolting takes place here
 */
void si_main(void)
{
	/* Recalibration of OSCALL */
	this_delta_t = TCNT1;
	TCNT1 = 0;
	res_buffer->dt = this_delta_t;

	current_index = ((uint8_t) (this_temperature) >> 1);
	for (uint8_t i = 0; i < sizeof(req_buffer->checksum); ++i) {
		if (((uint8_t *) &req_buffer->checksum)[i] != (MCU_CHECK_RESULT_CORRECT)[i]) {
			error = 1;
			puts(REPORT_ERROR ":" ERROR_CHECKSUM);
			break;
		}
	}
	if (req_buffer->rst_flags == MCU_SW_RESET) {
		error = 1;
		puts(REPORT_ERROR ":" ERROR_RESET);
	}
	/*Possible Temp-Readout error*/
	if (((this_temperature + 10) <= prv_temperature) || (this_temperature >= (prv_temperature + 10))) {
		error = 1;
		current_index = ((uint8_t) (prv_temperature) >> 1);
		puts(REPORT_ERROR ":" ERROR_TEMP);
	}

	if (error)
		si_main_error();
	else
		si_main_no_error();

	res_buffer->voltage = current_voltage;

	if (this_delta_t > (delta_t + SI_DELTA_T_MARGIN))
		res_buffer->osccal = req_buffer->osccal + 1;
	else if (this_delta_t < (delta_t - SI_DELTA_T_MARGIN))
		res_buffer->osccal = req_buffer->osccal - 1;
	else
		res_buffer->osccal = req_buffer->osccal;
	SI_UNLOCK();
	printf(REPORT_PERIODIC ":v=%u,t=%d,n=%u\n",
				current_voltage,
				this_temperature - SI_TEMP_OFFSET,
				table_entries);
}

int main(void)
{
	state = SI_INIT;
	while (1) {
		if (master) {
			si_master();
			master = 0;
			this_altbyte = req_buffer->alt_byte;
		}
		switch (state) {
		case SI_INIT:
			si_init();
			state = SI_IDLE;
			next_state = SI_TRANSIENT;
			break;
		case SI_IDLE:
			if (req_buffer->rst_disable == 0xFF) {
				this_sleeptime = req_buffer->rst_flags;
				SI_REPLY_DEBUG_RESET();
				master = 1;
				break;
			}
			if (req_buffer->alt_byte != this_altbyte)
				si_frame_received();
			break;
		case SI_TRANSIENT:
			si_transient();
			if (!sent_hello) {
				sent_hello = 1;
				puts(REPORT_HELLO ":");
			}
			state = SI_IDLE;
			break;
		case SI_MAIN:
			si_main();
			state = SI_IDLE;
			break;
		}
	}
}

/*
 * Use I2C master to reset digital potentiometer
 */
void reset_voltage_level(void)
{
	uint8_t txb[2] = {VREG_OP, SI_VOLT_REG_RESET};
	current_voltage = SI_VOLT_REG_RESET;
	i2c_init_master();
	i2c_write_bytes((VREG_DEV_ADDR_W >> 1), txb, sizeof(txb));
}

void reset_mega(void)
{
	cli();
	SI_PULL_RESET_LINE();
	_delay_ms(300);
	reset_voltage_level();
	i2c_slave_init(SI_I2C_ADDR << 1);

	_delay_ms(300);
	SI_LOCK();
	SI_RELEASE_RESET_LINE();

	/* Reset state machine */
	startup = SI_STARTUP_DELAY;
	master = 1;
	state = SI_IDLE;
	next_state = SI_TRANSIENT;
	SI_REPLY_DEBUG_RESET();
	res_buffer->osccal = fact_default;
	res_buffer->voltage = SI_VOLT_REG_RESET;
	sei();
	this_altbyte = 0;
	SI_UNLOCK();
}

ISR(TIM1_OVF_vect)
{
	/* Counter for HW-Reset */
	count_overflows++;
	TCNT1 = 0;

	if (!master) {
		puts(REPORT_ERROR ":" ERROR_TIMEOUT);
		rst_errors++;
		/* Only store values to the table when we are not in transient mode
		   and the timeout does not happen immediately after another error */
		if (error == 0 || count_overflows >= 3) {
			current_voltage -= SI_DEFAULT_VOLT_OFFSET;
			current_index = ((uint8_t) (this_temperature) >> 1);
			create_table_entry(current_voltage, req_buffer->osccal);
			SI_REPLY_DEBUG_TABLE_ENTRY();
			if (table_entries >= SI_PREDICTION_THRESHOLD)
				prediction();
			count_overflows = 0;
		}
		/* If the watchdog is triggered => reset main MCU */
		reset_mega();
	}
}
