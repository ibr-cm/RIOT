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
#include <avr/eeprom.h>
#include <avr/io.h>
#include "math.h"
#include "drv/adc-drv.h"
#include "drv/sw_uart.h"

static iv_req_t *req_buffer = (iv_req_t *) rxbuffer;
static uint8_t *lock_buffer = (uint8_t *) txbuffer;
static iv_res_t *res_buffer = (iv_res_t *) (txbuffer + 1);

uint8_t this_altbyte = 0;

volatile uint16_t this_delta_t = 0;
static int8_t prv_temperature = 71;  //previous temperature
static int8_t this_temperature;      //current temperature
static uint8_t this_sleeptime;
static uint8_t checksum, error;
static uint8_t startup, current_voltage, iteration;
static uint16_t delta_t = 55050;
static uint8_t state, next_state;
static uint8_t current_index;
static uint8_t table_entries = 0;
static uint8_t eeprom_table_available = 0;
static uint8_t v_offset = 0;

/* Debug counter */
static uint8_t alu_errors = 0;
static uint8_t rst_errors = 0;
static uint8_t sent_hello = 0;

///For deadlock reset
volatile uint8_t count_overflows = 0;
static uint8_t fact_default;

struct table_entry table[SI_TABLE_SIZE];

/* FUNCTION DECLARARTIONS */

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

	/* Enable external Watchdog*/ //ToDo!!
	req_buffer->rst_disable = 0x00;
	startup = SI_STARTUP_DELAY;
	this_sleeptime = 0;
	iteration = 0;
	current_voltage = SI_VOLT_REG_OFFSET;
	init_table();

	sei();
	req_buffer->alt_byte = 0;
	SI_UNLOCK();
}

void si_master(void)
{
	puts(REPORT_MASTER ":e");
	i2c_init_master();
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
		//puts("reading temp");
		result = i2c_read_regs(TMP_ADDR, TMP_REG, (uint8_t *) &this_temperature, 1);
		//printf("temp = %d°C\n", this_temperature);
		if (result != USI_TWI_SUCCESS) { // the Mega is trying to sabotage the whole oparation
			puts(REPORT_MASTER ":e1");
			SI_PULL_RESET_LINE();
			_delay_ms(200);
			SI_RELEASE_RESET_LINE();
			startup = SI_STARTUP_DELAY;
			continue;
		}
		this_temperature += SI_TEMP_OFFSET;
		current_index = ((uint8_t) (this_temperature) >> 1);
		if ((table[(current_index)].info != SI_TABLE_VALUE_IS_EMPTY)) {
			if (current_voltage != table[(current_index)].voltage) {
				current_voltage = table[(current_index)].voltage;
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
	checksum = MCU_CHECK_RESULT_CORRECT; //validate(rxbuffer[mcu_temperature]);
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
	uint8_t entry_not_empty =
			table[(current_index)].info != SI_TABLE_VALUE_IS_EMPTY;
	if(eeprom_table_available || entry_not_empty) {
		approach_voltage(table[(current_index)].voltage);
	}
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
	current_voltage -= SI_DEFAULT_VOLT_OFFSET;
	alu_errors++;
	table[(current_index)].voltage = current_voltage;
	table[(current_index)].osccal = req_buffer->osccal;
	if (table[(current_index)].info == SI_TABLE_VALUE_IS_EMPTY) {
		table[(current_index)].info = SI_TABLE_VALUE_IS_MEASURED;
		table_entries++;
	}
	if (table_entries >= SI_PREDICTION_THRESHOLD)
		prediction();
	/* Reply OSCALL Value to MCU
	 * ToDo: Recalibration Process*/
	res_buffer->osccal = req_buffer->osccal;
	/* DEBUG ONLY*/
	SI_REPLY_DEBUG_TABLE_ENTRY();
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
	uint8_t temp_diff = this_temperature - prv_temperature;
	if (temp_diff <= -2) {
		current_voltage -= 5;
		prv_temperature = this_temperature;
	} else if (temp_diff >= 2) {
		prv_temperature = this_temperature;
	}

	if ((table[(current_index)].info != SI_TABLE_VALUE_IS_EMPTY)) {
		approach_voltage(table[(current_index)].voltage);
		res_buffer->osccal = table[(current_index)].osccal;
		SI_REPLY_DEBUG_TABLE_USED(table[(current_index)].info);
		iteration = 0;
	} else if (++iteration > SI_ADAPTION_INTERVAL) {
		current_voltage += 1;
		/* If absolute min. voltage level is reached => table entry*/
		if (current_voltage >= 254) {
			table[(current_index)].voltage = current_voltage;
			table[(current_index)].osccal = req_buffer->osccal;
			if (table[(current_index)].info
					== SI_TABLE_VALUE_IS_EMPTY) {
				table[(current_index)].info =
						SI_TABLE_VALUE_IS_MEASURED;
			}
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

	/* Get hashed table index:*/
	current_index = ((uint8_t) (this_temperature) >> 1);
	/* 1) Check if an Checksum error occurred*/
	if (req_buffer->checksum != checksum) {
		error = 1;
		puts(REPORT_ERROR ":" ERROR_CHECKSUM);
	}
	if (req_buffer->rst_flags == MCU_SW_RESET) {
		error = 1;
		puts(REPORT_ERROR ":" ERROR_RESET);
	}
	/*Possible Temp-Readout error*/
	if (((this_temperature + 10) <= prv_temperature)
			|| (this_temperature >= (prv_temperature + 10))) {
		error = 1;
		current_index = ((uint8_t) (prv_temperature) >> 1);
		puts(REPORT_ERROR ":" ERROR_TEMP);
	}

	if (error)
		si_main_error();
	else
		si_main_no_error();

	/* Reply Frame:
	 * New Voltage Level*/
	res_buffer->voltage = current_voltage;

	/*!
	 * Hier muss nochmal die Rekalibrierung überdacht werden
	*/
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

/* Main routine
 *
 * ToDo: shut-down pin implememtieren,
 * sodass die Programierfunktion nicht beeinträchtigt wird
 * Ist bare auch suboptimal, da nicht generisch genug */
int main(void)
{
	state = SI_INIT;
	while (1) {
		switch (state) {
		case SI_INIT:
			si_init();
			state = SI_MASTER;
			next_state = SI_TRANSIENT;
			break;
		case SI_MASTER:
			si_master();
			state = SI_IDLE;
			this_altbyte = req_buffer->alt_byte;
			break;
		case SI_IDLE:
			if (req_buffer->rst_disable == 0xFF) {
				this_sleeptime = req_buffer->rst_flags;
				SI_REPLY_DEBUG_RESET();
				state = SI_MASTER;
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

void init_table(void)
{
	if (eeprom_read_byte(0x0000) == 'y') {
		for (uint8_t i = 0; i < SI_TABLE_SIZE; i++) {
			void *entry_ptr = EEPROM_ADDR_TABLE
					+ (i * sizeof(struct table_entry));
			eeprom_read_block(&table[i], entry_ptr, 3);
		}
		eeprom_table_available = 1;
		v_offset = eeprom_read_byte(EEPROM_ADDR_VOFF);
		for (uint8_t i = 0; i < SI_TABLE_SIZE; i++)
			table[i].voltage -= v_offset;
	} else {
		for (uint8_t i = 0; i < SI_TABLE_SIZE; i++) {
			table[i].voltage = SI_TABLE_VALUE_IS_EMPTY;
			table[i].info = SI_TABLE_VALUE_IS_EMPTY;
		}
	}
}

/*
 * Use I2C master to reset digital potentiometer
 *
 * ToDO: den I2C Master aufräumen
 * todo Value to write
 */
void reset_voltage_level(void)
{
	uint8_t txb[2] = {VREG_OP, SI_VOLT_REG_RESET};
	current_voltage = SI_VOLT_REG_RESET;
	i2c_init_master();
	i2c_write_bytes((VREG_DEV_ADDR_W >> 1), txb, sizeof(txb));
}

void prediction_fill_table(
		double m_volt, double b_volt,
		double m_osc, double b_osc)
{
	uint8_t index;
	for (index = 0; index < SI_TABLE_SIZE; index++) {
		if (table[index].info != SI_TABLE_VALUE_IS_MEASURED) {
			table[index].voltage =
					round(m_volt * (double) index + b_volt);
			table[index].osccal =
					round(m_osc * (double) index + b_osc);
			table[index].info = SI_TABLE_VALUE_IS_PREDICTED;
		}
	}
}

uint8_t prediction_analyze_table(
		double *m_volt, double *b_volt,
		double *m_osc, double *b_osc)
{
	uint8_t n = 0;
	uint8_t index;
	double sum_xy = 0.0,
	       sum_x  = 0.0,
	       sum_x2 = 0.0,
	       sum_y  = 0.0;

	for (index = 0; index < SI_TABLE_SIZE; index++) {
		if (table[index].info == SI_TABLE_VALUE_IS_MEASURED) {
			n++;
			sum_x += (double) index;
			sum_x2 += (double) index * (double) index;
			sum_y += (double) table[index].voltage;
			sum_xy += (double) index
					* (double) table[index].voltage;
		}
	}
	*m_volt = (sum_xy - ((sum_x * sum_y) / n))
			/ (sum_x2 - ((sum_x * sum_x) / n));
	*b_volt = (sum_y / n) - (sum_x / n) * *m_volt;

	sum_y = 0.0;
	sum_xy = 0.0;
	for (index = 0; index < SI_TABLE_SIZE; index++) {
		if (table[index].info == SI_TABLE_VALUE_IS_MEASURED) {
			sum_y += (double) table[index].osccal;
			sum_xy += (double) index
					* (double) table[index].osccal;
		}
	}
	*m_osc = (sum_xy - ((sum_x * sum_y) / n))
			/ (sum_x2 - ((sum_x * sum_x) / n));
	*b_osc = (sum_y / n) - (sum_x / n) * *m_osc;
	return n;
}

void prediction(void)
{
	uint8_t n;
	double m_volt, b_volt, m_osc, b_osc;

	/* decrease table_entries - any failure forces a new prediction */
	table_entries--;

	/* runaways? */
	n = prediction_analyze_table(&m_volt, &b_volt, &m_osc, &b_osc);
	if (n >= SI_PREDICTION_THRESHOLD) {
		prediction_fill_table(m_volt, b_volt, m_osc, b_osc);
		SI_REPLY_DEBUG_TABLE_USED(SI_TABLE_PREDICTION);
	} else {
		table_entries = (uint8_t) n;
	}

	/* Write characteristic curve to eeprom */
	for (uint8_t i = 0; i < SI_TABLE_SIZE; i++) {
		void *entry_ptr = EEPROM_ADDR_TABLE
				+ (i * sizeof(struct table_entry));
		eeprom_write_block(&table[i], entry_ptr, 3);
	}

	/* Write 'y' to the first addr of the eeprom
	   to indicate that the cc already exists
	   and write the initial voltage offset */
	eeprom_write_byte(EEPROM_ADDR_AVAIL, 'y');
	eeprom_write_byte(EEPROM_ADDR_VOFF, 0);
}

ISR(TIM1_OVF_vect)
{
	/* Counter for HW-Reset */
	count_overflows++;
	TCNT1 = 0;

	/* External Watchdog: Reset if > DEADLOCK_THRESHOLD*/
	//if ((count_overflows > DEADLOCK_THRESHOLD) && (req_buffer->rst_disable == 0x01)) {
	//if ((count_overflows > DEADLOCK_THRESHOLD) && (state != SI_DEBUG)){
	if ((state != SI_MASTER)) {
		puts(REPORT_ERROR ":" ERROR_TIMEOUT);
		rst_errors++;
		/* If the watchdog is triggered => reset main MCU */
		cli();
		SI_PULL_RESET_LINE();
		
		/* Only store values to the table when we are not in transient mode
		   and the timeout does not happen immediately after another error */
		if (next_state != SI_TRANSIENT && error == 0) {
			current_index = ((uint8_t) (this_temperature) >> 1);//get hashed index of the table
			current_voltage -= SI_DEFAULT_VOLT_OFFSET;
			if (table[(current_index)].info != SI_TABLE_VALUE_IS_MEASURED)
				table_entries++;
			table[(current_index)].voltage = current_voltage; //updating table entry
			table[(current_index)].osccal = req_buffer->osccal;
			table[(current_index)].info = SI_TABLE_VALUE_IS_MEASURED;
			SI_REPLY_DEBUG_TABLE_ENTRY();
			/* Prediction Process:*/
			if (table_entries >= SI_PREDICTION_THRESHOLD)
				prediction();
		}
		_delay_ms(300);
		reset_voltage_level(); //secure instance as master
		//Implementation uses different address notation
		i2c_slave_init(SI_I2C_ADDR << 1); // secure instance as slave

		_delay_ms(300);
		/* The reset is stored as a failure event => table entry*/
		SI_LOCK();
		SI_RELEASE_RESET_LINE();

		/* Reset state machine */
		startup = SI_STARTUP_DELAY;
		state = SI_MASTER;
		next_state = SI_TRANSIENT;
		SI_REPLY_DEBUG_RESET();
		res_buffer->osccal = fact_default;
		res_buffer->voltage = SI_VOLT_REG_RESET;
		//ToDo: default voltage
		count_overflows = 0;
		/* Reset alternating byte */
		sei();	 
		this_altbyte = 0;
		SI_UNLOCK();
	}
}
