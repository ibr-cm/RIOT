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
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/io.h>
#include "drv/adc-drv.h"
#include "math.h"
#include "sw_uart.h"
#include "../include/idealvolting_config.h"
#include "../include/idealvolting_frame.h"

#include "i2c-master.h"
#include "i2c-slave.h"

/* Definitions */
struct table_entry {
	uint8_t voltage;
	uint8_t osccal;
	uint8_t info;
};

#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#define SI_INIT                             0
#define SI_TRANSIENT                        1
#define SI_MAIN                             2
#define SI_IDLE                             3
#define SI_RESET                            4
#define SI_DEBUG                            5

#define SI_STARTUP_DELAY                    7
#define SI_VOLT_REG_OFFSET                  130//128///200// für 4mhz200
#define SI_VOLT_REG_RESET                   140
#define SI_DEFAULT_VOLT_OFFSET              4
#define MATRIX_SIZE                         4
#define SI_ADAPTION_INTERVAL                5
#define SI_DELTA_T_MARGIN                   25
#define SI_TEMP_OFFSET                      25

#define SI_PREDICTION_THRESHOLD             7
#define SI_TABLE_SIZE                       51
#define SI_TABLE_VALUE_IS_EMPTY             0xFF
#define SI_TABLE_VALUE_IS_MEASURED          0x01
#define SI_TABLE_VALUE_IS_PREDICTED         0x02
#define SI_TABLE_PREDICTION                 0x03

#define EEPROM_ADDR_AVAIL                   ((void *) 0x00)
#define EEPROM_ADDR_VOFF                    ((void *) 0x01)
#define EEPROM_ADDR_TABLE                   ((void *) 0x02)

#define SI_PWR_MONITOR_ICC_ADC              ADC_CHANNEL_1
#define SI_PWR_MONITOR_VCC_ADC              ADC_CHANNEL_0

#define SI_LOCK()                           *lock_buffer = 0
#define SI_UNLOCK()                         *lock_buffer = 1
#define SI_BOOTING()                        *lock_buffer = 2

#define SI_REPLY_DEBUG_RESET()              res_buffer->debug = 0x00
#define SI_REPLY_DEBUG_STATE(current_state) res_buffer->debug |= current_state
#define SI_REPLY_DEBUG_TABLE_ENTRY()        res_buffer->debug |= (1 << 3)
#define SI_REPLY_DEBUG_RESET_HW()           res_buffer->debug |= (1 << 4)
#define SI_REPLY_DEBUG_RESET_SW()           res_buffer->debug |= (1 << 5)
#define SI_REPLY_DEBUG_TABLE_USED(info)     res_buffer->debug |= (info << 6)

#define SI_STAY_IN_DEBUG                    1

#define SI_INIT_RESET_LINE()                DDRA |= (1<<PA7)    //0b00000100 set PB2 to Output
#define SI_PULL_RESET_LINE()                PORTA |= (1<<PA7)   //0b00000100 set PB2 to 1
#define SI_RELEASE_RESET_LINE()             PORTA &= ~(1<<PA7)  // set PB2 to 0
#define MCU_HW_RESET                        0x01
#define MCU_SW_RESET                        0x02

///Deadlock Reset
#define DEADLOCK_THRESHOLD                  3 //overflows corresponds to a deadlock situation

///Digital Potentiometer Reset Software i2c
#if defined BOARD_INGA_BLUE
#define VREG_DEV_ADDR_W                     0x58
#define VREG_OP                             0x00
#elif defined BOARD_REAPER
#define VREG_DEV_ADDR_W                     0x54
#define VREG_OP                             0x11
#else
#error Define either BOARD_INGA_BLUE or BOARD_REAPER
#endif

#define SI_STEP                             10

#if USE_MEGA_CLOCK
#define DT_TARGET_SRC TCNT1;
#else
#define DT_TARGET_SRC 16000;
#endif

#define REPORT_HELLO                        'h'
#define REPORT_PERIODIC                     'p'
#define REPORT_ERROR                        'e'
#define REPORT_DEBUG                        'd'

static iv_req_t *req_buffer = (iv_req_t *) rxbuffer;
static uint8_t *lock_buffer = (uint8_t *) txbuffer;
static iv_res_t *res_buffer = (iv_res_t *) (txbuffer + 1);

uint8_t this_altbyte = 0;

volatile uint16_t this_delta_t = 0;
static int8_t prv_temperature = 71;  //previous temperature
static int8_t this_temperature;      //current temperature
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
void init_table(void);
uint8_t reset_voltage_level(void);
void prediction(void);
uint8_t get_table_status(void);
void erase_eeprom(void);
void send_report(char report_type);

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
	uint8_t rvl_success = reset_voltage_level() == 0;
	_delay_ms(100);
	SI_RELEASE_RESET_LINE();

	sw_uart_init();
	_delay_ms(1000);
	printf("rvl_success = %d\n", rvl_success);
	/* This I2C implementation uses a different address notation */
	i2c_slave_init(SI_I2C_ADDR << 1);

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
	iteration = 0;
	current_voltage = SI_VOLT_REG_OFFSET;
	init_table();

	sei();
	req_buffer->alt_byte = 0;
	SI_UNLOCK();
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
	/* Reply of OSCALL and Voltage Level*/
	res_buffer->osccal = req_buffer->osccal;
	res_buffer->voltage = current_voltage;
	/*If EEPROM data are available, the voltage potential 128->~200 is too heavy */
	current_index = ((uint8_t) (this_temperature) >> 1);
	uint8_t entry_not_empty =
			table[(current_index)].info != SI_TABLE_VALUE_IS_EMPTY;
	if(eeprom_table_available || entry_not_empty) {
		approach_voltage(table[(current_index)].voltage);
	}
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
		printf("%c:checksum\n", REPORT_ERROR);
	}
	if (req_buffer->rst_flags == MCU_SW_RESET) {
		error = 1;
		printf("%c:sw_reset\n", REPORT_ERROR);
	}
	/*Possible Temp-Readout error*/
	if (((this_temperature + 10) <= prv_temperature)
			|| (this_temperature >= (prv_temperature + 10))) {
		error = 1;
		current_index = ((uint8_t) (prv_temperature) >> 1);
		printf("%c:temp_readout\n", REPORT_ERROR);
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
}

void si_debug()
{
	/* Use 2 Bytes of the protocol frame 'rxbuffer' to enter (and communicate with) the debug state:
	 * debug_rx_0 = mcu_rst_disable (forbidden: 0xFF => entry)
	 *    7     6     5     4     3     2     1     0
	 * +-----+-----+-----+-----+-----+-----+-----+-----+
	 * |          input data   |     output select     |
	 * +-----+-----+-----+-----+-----+-----+-----+-----+
	 *
	 * debug_rx_0 = mcu_rstFlags
	 *    7     6     5     4     3     2     1     0
	 * +-----+-----+-----+-----+-----+-----+-----+-----+
	 * |                  input data                   |
	 * +-----+-----+-----+-----+-----+-----+-----+-----+
	 *
	 *
	 * Use 3 Bytes of the protocol frame 'txbuffer' to implement debug output:
	 * debug_tx_0 = si_dt_l
	 * debug_tx_1 = si_dt_h
	 * debug_tx_2 = si_debug
	 *    7     6     5     4     3     2     1     0
	 * +-----+-----+-----+-----+-----+-----+-----+-----+
	 * |                  output data                  |
	 * +-----+-----+-----+-----+-----+-----+-----+-----+
	 */

	/* SI is now in Debug mode: */
	SI_REPLY_DEBUG_STATE(SI_DEBUG);
	/* Stay in debug state until the main MCU will send the exit command */
	while (req_buffer->rst_disable > SI_STAY_IN_DEBUG) {
		/* Get table status of collected Temperatures: */
		if ((req_buffer->rst_disable & 0x0F) == 2) {
			/* get information if already predicted,
			 *  how many points are manifested and the table size*/
			res_buffer->dt = get_table_status()
					| (SI_TABLE_SIZE << 8);
			res_buffer->debug = v_offset;//get information about temp offset
			req_buffer->rst_disable = 0xFF;
		/* Get table entry */
		} else if ((req_buffer->rst_disable & 0x0F) == 3) {
			current_index = (int8_t)(req_buffer->rst_flags);
			current_index += SI_TEMP_OFFSET;
			current_index = (current_index >> 1);
			res_buffer->dt = table[(current_index)].voltage
					| (table[(current_index)].osccal << 8);
			res_buffer->debug = table[(current_index)].info;
			req_buffer->rst_disable = 0xFF;
		/* Get error counters */
		} else if ((req_buffer->rst_disable & 0x0F) == 4) {
			res_buffer->dt = alu_errors | (rst_errors << 8);
			req_buffer->rst_disable = 0xFF;
		/* erase eeprom */
		} else if ((req_buffer->rst_disable & 0x0F) == 5) {
			eeprom_write_byte(0x0000, 'n'); //fast erase, just ignore eeprom data
			req_buffer->rst_disable = 0xFF;
		/* eeprom status*/
		} else if ((req_buffer->rst_disable & 0x0F) == 6) {
			res_buffer->dt = eeprom_read_byte(0x0000);
			req_buffer->rst_disable = 0xFF;
		/* set voltage offset */
		} else if ((req_buffer->rst_disable & 0x0F) == 7) {
			v_offset = (uint8_t)(req_buffer->rst_flags);
			if (v_offset > 30) {
				v_offset = 30; //error, invalid offset
			}
			if (eeprom_table_available) {
				for (uint8_t i = 0; i < SI_TABLE_SIZE; i++) { //update table
					table[i].voltage -= v_offset;
				}
				eeprom_write_byte(EEPROM_ADDR_VOFF, v_offset);
			}
			req_buffer->rst_disable = 0xFF;
		}
	}
	/* Reset overflow counter for external Watchdog*/
	TCNT1 = 0;
	count_overflows = 0;
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
			state = SI_IDLE;
			next_state = SI_TRANSIENT;
			break;

		case SI_IDLE:
			if (req_buffer->alt_byte != this_altbyte)
				si_frame_received();
			if (req_buffer->rst_disable == 0xFF) {
				SI_REPLY_DEBUG_RESET();
				state = SI_DEBUG;
			}
			break;

		case SI_TRANSIENT:
			si_transient();
			if (sent_hello) {
				send_report(REPORT_PERIODIC);
			} else {
				printf("%c:hello\n", REPORT_HELLO);
				sent_hello = 1;
			}
			state = SI_IDLE;
			break;

		case SI_MAIN:
			si_main();
			send_report(REPORT_PERIODIC);
			state = SI_IDLE;
			break;

		case SI_RESET:
			/* wäre schöner, wenn es hier implementiert würde*/
			break;

		case SI_DEBUG:
			si_debug();
			state = SI_IDLE;
			next_state = SI_TRANSIENT;
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
uint8_t reset_voltage_level(void)
{
	char mi2c_tx_buffer[3] = {
			VREG_DEV_ADDR_W,   //Write Slave Address
			VREG_OP,           //Internal address
			SI_VOLT_REG_RESET  //SI_VOLT_REG_OFFSET;
	};
	current_voltage = SI_VOLT_REG_RESET;
	return i2c_master_transmit(mi2c_tx_buffer, sizeof(mi2c_tx_buffer)) - 1;
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

/*
 *  Table Status Byte:
 *
 *    7     6     5     4     3     2     1     0
 * +-----+-----+-----+-----+-----+-----+-----+-----+
 * |prdc?|            # of table entries           |
 * +-----+-----+-----+-----+-----+-----+-----+-----+
 */
uint8_t get_table_status(void)
{
	uint8_t index, m, p;
	m = 0;
	p = 0;
	for (index = 0; index < SI_TABLE_SIZE; index++) {
		if (table[index].info == SI_TABLE_VALUE_IS_MEASURED)
			m++;
		else if (table[index].info == SI_TABLE_VALUE_IS_PREDICTED)
			p++;
	}
	if (p > 0)
		m |= (1 << 7);
	return m;
}

void send_report(char report_type)
{
	printf("%c:v=%u,t=%d,n=%u\n",
			report_type,
			current_voltage,
			this_temperature - SI_TEMP_OFFSET,
			table_entries);
}

ISR(TIM1_OVF_vect)
{
	/* Counter for HW-Reset */
	count_overflows++;
	TCNT1 = 0;

	/* External Watchdog: Reset if > DEADLOCK_THRESHOLD*/
	//if ((count_overflows > DEADLOCK_THRESHOLD) && (req_buffer->rst_disable == 0x01)) {
	//if ((count_overflows > DEADLOCK_THRESHOLD) && (state != SI_DEBUG)){	
	if ((state != SI_DEBUG)) {
		printf("%c:timeout\n", REPORT_ERROR);
		rst_errors++;
		/* If the watchdog is triggered => reset main MCU */
		cli();
		SI_PULL_RESET_LINE();
		
		/* Only store values to the table when we are not in transient mode */			
		if (next_state != SI_TRANSIENT) {
			current_index = ((uint8_t) (this_temperature) >> 1);//get hashed index of the table
			current_voltage -= SI_DEFAULT_VOLT_OFFSET;
			table[(current_index)].voltage = current_voltage; //updating table entry
			table[(current_index)].osccal = req_buffer->osccal;
			table[(current_index)].info = SI_TABLE_VALUE_IS_MEASURED;
			table_entries++;
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
		state = SI_IDLE;
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

