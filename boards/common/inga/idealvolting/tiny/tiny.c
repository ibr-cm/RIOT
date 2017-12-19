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

#define SLAVE_ADDR_ATTINY                       0x5A

#ifndef F_CPU
#define F_CPU 1000000UL
#endif

#define SI_INIT                                 0
#define SI_TRANSIENT                            1
#define SI_MAIN                                 2
#define SI_IDLE                                 3
#define SI_RESET                                4
#define SI_DEBUG                                5

#define SI_STARTUP_DELAY                        7
#define SI_VOLT_REG_OFFSET                      130//128///200// für 4mhz200
#define SI_VOLT_REG_RESET                       140
#define SI_DEFAULT_VOLT_OFFSET                  4
#define MATRIX_SIZE                             4
#define SI_ADAPTION_INTERVAL                    5
#define SI_DELTA_T_MARGIN                       25
#define SI_TEMP_OFFSET                          25

#define SI_PREDICTION_THRESHOLD                 7
#define SI_TABLE_SIZE                           51
#define SI_TABLE_EEPROM_START_ADDR              2 // must be > 0
#define SI_TABLE_VALUE_IS_EMPTY                 0xFF
#define SI_TABLE_VALUE_IS_MEASURED              0x01
#define SI_TABLE_VALUE_IS_PREDICTED             0x02
#define SI_TABLE_PREDICTION                     0x03

#define SI_PWR_MONITOR_ICC_ADC                  ADC_CHANNEL_1
#define SI_PWR_MONITOR_VCC_ADC                  ADC_CHANNEL_0

#define SI_LOCK()                               0
#define SI_UNLOCK()                             1
#define SI_BOOTING()                            2

#define SI_REPLY_OSCCAL_FB(feedback)            res_buffer->osccal = feedback;
#define SI_REPLY_VOLTAGE(volt_value)            res_buffer->voltage = volt_value;
#define SI_REPLY_DELTA_T(cnt_value)             res_buffer->dt_l = cnt_value;\
                                                res_buffer->dt_h = (cnt_value >> 8)

#define SI_REPLY_DEBUG_RESET()                  res_buffer->debug = 0x00
#define SI_REPLY_DEBUG_STATE(current_state)     res_buffer->debug |= current_state
#define SI_REPLY_DEBUG_TABLE_ENTRY()            res_buffer->debug |= (1 << 3)
#define SI_REPLY_DEBUG_RESET_HW()               res_buffer->debug |= (1 << 4)
#define SI_REPLY_DEBUG_RESET_SW()               res_buffer->debug |= (1 << 5)
#define SI_REPLY_DEBUG_TABLE_USED(info)         res_buffer->debug |= (info << 6)
/*#define SI_REPLY_DEBUG_BATT_STATE(value)        txbuffer[6] = value;\
                                                txbuffer[7] = (value >> 8)*/

#define SI_STAY_IN_DEBUG                        1

#define SI_INIT_RESET_LINE()                    DDRA |= (1<<PA7)    //0b00000100 set PB2 to Output
#define SI_PULL_RESET_LINE()                    PORTA |= (1<<PA7)   //0b00000100 set PB2 to 1
#define SI_RELEASE_RESET_LINE()                 PORTA &= ~(1<<PA7)  // set PB2 to 0
#define MCU_HW_RESET                            0x01
#define MCU_SW_RESET                            0x02

#define EEPROM_ADD 0x00

///Deadlock Reset
#define DEADLOCK_THRESHOLD                      3 //overflows corresponds to a deadlock situation
///Digital Potentiometer Reset Software i2c
#define AD5242_DEV_ADDR_W                       0x58
#define AD5242_CHN_A                            0x00

#define SI_STEP                                 10

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

///For deadlock reset
volatile uint8_t count_overflows = 0;
static uint8_t fact_default;

struct table_entry table[SI_TABLE_SIZE];

/* FUNCTION DECLARARTIONS */
void init_SI();
void reset_voltage_level(void);
void prediction(void);
uint8_t get_table_status(void);
void erase_eeprom(void);


/* MAIN ROUTINE */

/* ToDo: shut-down pin implememtieren, sodass die Programierfunktion nicht beeinträchtigt wird 
 * Ist bare auch suboptimal, da nicht generisch genug */

int main(void) {
	state = SI_INIT;
	while (1) {
		switch (state) {

		case SI_INIT:
			//while(1){;}
			/* Initialize SI core components*/
			init_SI();
			/* Alternating Byte is set to zero (init)*/
			req_buffer->alt_byte = 0;
			/* Switch to idle state.
			 * When the first request frame arrives, next_state is transient state*/
			state = SI_IDLE;
			next_state = SI_TRANSIENT;
			break;

		case SI_IDLE:
			/* Check if alternating bytes has changed */
			if (req_buffer->alt_byte != this_altbyte) {
				/* Lock SI and copy current alternating byte */
				*lock_buffer = SI_LOCK();
				this_altbyte = req_buffer->alt_byte;
				/* Reset watchdog*/
				count_overflows = 0;
				/* Readout OSCALL Value from man MCU.
				 * Get Temperature and add offset due to negative values */
				fact_default = req_buffer->osccal; //??
				checksum = MCU_CHECK_RESULT_CORRECT; //validate(rxbuffer[mcu_temperature]);
				this_temperature = (int8_t) (req_buffer->temperature);
				this_temperature += SI_TEMP_OFFSET;
				/* DEBUG ONLY:*/
				SI_REPLY_DEBUG_RESET();
				SI_REPLY_DEBUG_STATE(next_state);
				/* Request Frame was received => switch to next_frame*/
				error = 0;
				state = next_state;
			}
			/* Only the main MCU initiates the enter of the debug state */
			if (req_buffer->rst_disable == 0xFF) {
				SI_REPLY_DEBUG_RESET();
				state = SI_DEBUG;
			}
			break;

		case SI_TRANSIENT:
			/* After startup the system is in transient state.
			 * Controlled undervolting is delayed by some cycles */
			if (startup > 0) {
				startup--;
#if USE_MEGA_CLOCK
				delta_t = TCNT1;
#else
				delta_t = 16000;
#endif
				TCNT1 = 0;
				SI_REPLY_DELTA_T(delta_t);
			} else {
				/* Backup default OSCALL value before controlled undervolting starts*/
				fact_default = req_buffer->osccal;
				prv_temperature = this_temperature;
				next_state = SI_MAIN;
			}
			/* Reply of OSCALL and Voltage Level*/
			res_buffer->osccal = req_buffer->osccal; //SI_REPLY_OSCCAL_FB(byte4); //nötig??
			res_buffer->voltage = current_voltage;
			/*If EEPROM data are available, the voltage potential 128->~200 is too heavy */
			if((eeprom_table_available) || (table[(current_index)].info != SI_TABLE_VALUE_IS_EMPTY)) {
				current_voltage += SI_STEP;
				current_index = ((uint8_t) (this_temperature) >> 1);
				if (current_voltage <= table[(current_index)].voltage) {
					res_buffer->voltage = current_voltage;
				} else {
					res_buffer->voltage = table[(current_index)].voltage;
					current_voltage = res_buffer->voltage;
				}
			}
			/* If a reset occured, send this information to MCU*/
			if (req_buffer->rst_flags == MCU_HW_RESET) {
				SI_REPLY_DEBUG_RESET_HW();
			} else if (req_buffer->rst_flags == MCU_SW_RESET) {
				SI_REPLY_DEBUG_RESET_SW();
			}
			/* Unlock SI and get back to IDLE state*/
			*lock_buffer = SI_UNLOCK();
			state = SI_IDLE;
			break;

		case SI_MAIN:
			/* Main State: Controlled Undervolting takes place here*/
			/* Recalibration of OSCALL */
			this_delta_t = TCNT1;
			TCNT1 = 0;
			SI_REPLY_DELTA_T(this_delta_t);
			
			/* Get hashed table index:*/
			current_index = ((uint8_t) (this_temperature) >> 1);
			/* 1) Check if an Checksum error occurred*/
			if (req_buffer->checksum != checksum) {
				error = 1;
			}			
			if (req_buffer->rst_flags == MCU_SW_RESET) {
				error = 1;
			}			
			/*Possible Temp-Readout error*/
			if (((this_temperature + 10) <= prv_temperature) || (this_temperature >= (prv_temperature + 10))) {
				error = 1;
				current_index = ((uint8_t) (prv_temperature) >> 1);
			}

			if (error) {
				/* Checksum Error:
				 * Decreases poti-reg value which increases voltage level */
				current_voltage -= SI_DEFAULT_VOLT_OFFSET; //
				alu_errors++;
				/* Supervised Learning:
				 * Add/Update this idealVoltage level to the table*/
				table[(current_index)].voltage = current_voltage;
				table[(current_index)].osccal = req_buffer->osccal;
				if (table[(current_index)].info == SI_TABLE_VALUE_IS_EMPTY) {
					table[(current_index)].info = SI_TABLE_VALUE_IS_MEASURED;
					table_entries++;
				}
				/* If enough idealVoltages are ascertained => predict*/
				if (table_entries >= SI_PREDICTION_THRESHOLD) {
					prediction();
					/* decrease table_entries - any failure forces a new prediction */
					table_entries--; //
				}
				/* Reply OSCALL Value to MCU
				 * ToDo: Recalibration Process*/
				res_buffer->osccal = req_buffer->osccal;
				/* DEBUG ONLY*/
				SI_REPLY_DEBUG_TABLE_ENTRY();

			} else {
				/* ToDo: Nicht optimal => sollte in einzelne Funktionen gepackt werden*/
				/* No Checksum Error occurred */
				/* Temperature change > +/- 2°C ?*/
				if (((this_temperature + 2) <= prv_temperature) || (this_temperature >= (prv_temperature + 2))) {
					/* Check if table entry exists for this temperature */
					if (table[(current_index)].info != SI_TABLE_VALUE_IS_EMPTY) { //Does an entry for this temperature exist?
						current_voltage = table[(current_index)].voltage;
						SI_REPLY_OSCCAL_FB(table[(current_index)].osccal);
						SI_REPLY_DEBUG_TABLE_USED( table[(current_index)].info);
						/* Jump over the iterative ascertaining process*/
						iteration = 0;
					} else if (this_temperature < prv_temperature) {
						/* If no entry exist and Temperature drops by 2°C => increase voltage level*/
						current_voltage -= 5;
					}
					/* Copy this_temperature to compare next value*/
					prv_temperature = this_temperature;
				} else if ((table[(current_index)].info	!= SI_TABLE_VALUE_IS_EMPTY)) {
					/* If the temperature is constant, check if table entries are available */
					current_voltage = table[(current_index)].voltage;
					SI_REPLY_OSCCAL_FB(table[(current_index)].osccal);
					SI_REPLY_DEBUG_TABLE_USED(table[(current_index)].info);
					/* Jump over the iterative ascertaining process*/
					iteration = 0;
				}
				/* Iterative ascertaining of idealVoltage Levels:
				 * Decrease Voltage Level*/
				if (++iteration > SI_ADAPTION_INTERVAL) {
					/* Increase poti-reg by 1 to increase voltage level by ~15mV*/
					current_voltage += 1;
					if (current_voltage >= 254) {
						/* If absolute min. voltage level is reached => table entry*/
						table[(current_index)].voltage = current_voltage; //updating table entry
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
			/* Reply Frame:
			 * New Voltage Level*/
			printf("vreg=%d\n", current_voltage);
			res_buffer->voltage = current_voltage;
			/*!
			 * Hier muss nochmal die Rekalibrierung überdachzt werden
			*/

			if (this_delta_t > (delta_t + SI_DELTA_T_MARGIN)) {
				res_buffer->osccal = req_buffer->osccal + 1;
			} else if (this_delta_t < (delta_t - SI_DELTA_T_MARGIN)) {
				res_buffer->osccal = req_buffer->osccal - 1;
			} else {
				res_buffer->osccal = req_buffer->osccal;
			}
			state = SI_IDLE;
			*lock_buffer = SI_UNLOCK();
			break;

		case SI_RESET:
			/* wäre schöner, wenn es hier implementiert würde*/
			break;

		case SI_DEBUG:
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
					res_buffer->dt_l = get_table_status(); //get information if already predicted and how many points are manifested
					res_buffer->dt_h = SI_TABLE_SIZE;      //get information about table size
					res_buffer->debug = v_offset;	        //get information about temp offset
					req_buffer->rst_disable = 0xFF;
				/* Get table entry */
				} else if ((req_buffer->rst_disable & 0x0F) == 3) {
					current_index = (int8_t)(req_buffer->rst_flags);
					current_index += SI_TEMP_OFFSET;
					current_index = (current_index >> 1);
					res_buffer->dt_l = table[(current_index)].voltage;
					res_buffer->dt_h = table[(current_index)].osccal;
					res_buffer->debug = table[(current_index)].info;
					req_buffer->rst_disable = 0xFF;
				/* Get error counters */
				} else if ((req_buffer->rst_disable & 0x0F) == 4) {
					res_buffer->dt_l = alu_errors;
					res_buffer->dt_h = rst_errors;
					req_buffer->rst_disable = 0xFF;
				/* erase eeprom */
				} else if ((req_buffer->rst_disable & 0x0F) == 5) {
					eeprom_write_byte(0x0000, 'n'); //fast erase, just ignore eeprom data
					req_buffer->rst_disable = 0xFF;
				/* eeprom status*/
				} else if ((req_buffer->rst_disable & 0x0F) == 6) {
					res_buffer->dt_l = eeprom_read_byte(0x0000);
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
						eeprom_write_byte(0x0001, v_offset);
					}
					req_buffer->rst_disable = 0xFF;
				}
			}
			state = SI_IDLE;
			next_state = SI_TRANSIENT;
			/* Reset overflow counter for external Watchdog*/
			TCNT1 = 0;
			count_overflows = 0;
			break;
		}
	}
}



void init_SI() {
	cli();
	sw_uart_init();
	*lock_buffer = SI_LOCK();
	/* Initialize the Reset Pin for main MCU HW-reset*/
	SI_INIT_RESET_LINE();
	/* Reset Voltage to normal level when SI is reset*/
	SI_PULL_RESET_LINE();
	_delay_ms(100);
	reset_voltage_level(); //secure instance as master
	_delay_ms(100);
	/* I2C Slave Initialization */
	i2c_slave_init(SLAVE_ADDR_ATTINY);
	SI_RELEASE_RESET_LINE();
	/* Timer Initialization, Settings:
	 * Prescaler 8, Overflow Interrupt every ~0.5s, (maybe prsc32 would be better?)
	 */
	TCCR1A = 0b00000000;
	TCCR1B = 0b00000011;
	TIMSK1 |= (1 << TOIE1);
	TCNT1 = 0;
	/* Enable external Watchdog*/ //ToDo!!
	req_buffer->rst_disable = 0x00;
	/* # of iterations with transient phase*/
	startup = SI_STARTUP_DELAY;
	/* */
	iteration = 0;
	/* Startup voltage level*/
	current_voltage = SI_VOLT_REG_OFFSET;
	/* Initialize Table for supervised Learning */	
	if (eeprom_read_byte(0x0000) == 'y') { //if the curve is already available
		for (uint8_t i = 0; i < SI_TABLE_SIZE; i++) {
			eeprom_read_block(&table[i], (i*3)+ SI_TABLE_EEPROM_START_ADDR, 3); //3 because sizeof(table_entry)
		}
		eeprom_table_available = 1;
		/* Get offset value */
		v_offset = eeprom_read_byte(0x0001);
		for (uint8_t i = 0; i < SI_TABLE_SIZE; i++) {
			table[i].voltage -= v_offset;
		}
	} else {					//if not,
		for (uint8_t i = 0; i < SI_TABLE_SIZE; i++) {
			table[i].voltage = SI_TABLE_VALUE_IS_EMPTY;
			table[i].info = SI_TABLE_VALUE_IS_EMPTY;
		}
	}
	sei();
	*lock_buffer = SI_UNLOCK();
}

void reset_voltage_level(void) {
	/* ToDO: den I2C Master aufräumen */
	/* */
	char mi2c_tx_buffer[3] = {AD5242_DEV_ADDR_W,  //Write Slave Address
				  AD5242_CHN_A,       //Internal address
				  SI_VOLT_REG_RESET}; //SI_VOLT_REG_OFFSET; //todo Value to write
	/* Use I2C master to reset digital potentiometer */
	current_voltage = SI_VOLT_REG_RESET;
	i2c_master_transmit(mi2c_tx_buffer, sizeof(mi2c_tx_buffer));
}

void prediction(void) {
	uint8_t index;
	double sum_xy, sum_x, sum_x2, sum_y, n, m_volt, b_volt, m_osc, b_osc;

	/* runaways? */
	n = 0;
	sum_x = 0.0;
	sum_y = 0.0;
	sum_xy = 0.0;
	sum_x2 = 0.0;
	for (index = 0; index < SI_TABLE_SIZE; index++) {
		if (table[index].info == SI_TABLE_VALUE_IS_MEASURED) {
			n++;
			sum_x += (double) index;
			sum_x2 += (double) index * (double) index;
			sum_y += (double) table[index].voltage;
			sum_xy += (double) index * (double) table[index].voltage;
		}
	}
	m_volt = (sum_xy - ((sum_x * sum_y) / n)) / (sum_x2 - ((sum_x * sum_x) / n));
	b_volt = (sum_y / n) - (sum_x / n) * m_volt;

	sum_y = 0.0;
	sum_xy = 0.0;
	for (index = 0; index < SI_TABLE_SIZE; index++) {
		if (table[index].info == SI_TABLE_VALUE_IS_MEASURED) {
			sum_y += (double) table[index].osccal;
			sum_xy += (double) index * (double) table[index].osccal;
		}
	}
	m_osc = (sum_xy - ((sum_x * sum_y) / n)) / (sum_x2 - ((sum_x * sum_x) / n));
	b_osc = (sum_y / n) - (sum_x / n) * m_osc;

	if (n >= SI_PREDICTION_THRESHOLD) {
		for (index = 0; index < SI_TABLE_SIZE; index++) {
			if (table[index].info != SI_TABLE_VALUE_IS_MEASURED) {
				table[index].voltage = round(m_volt * (double) index + b_volt);
				table[index].osccal = round(m_osc * (double) index + b_osc);
				table[index].info = SI_TABLE_VALUE_IS_PREDICTED;
			}
		}
		SI_REPLY_DEBUG_TABLE_USED(SI_TABLE_PREDICTION);
	} else {
		table_entries = (uint8_t) n;
	}
	/*Write characteristic curve to eeprom*/
	for (uint8_t i = 0; i < SI_TABLE_SIZE; i++) {
		eeprom_write_block(&table[i], (i*3)+ SI_TABLE_EEPROM_START_ADDR, 3);
	}
	eeprom_write_byte(0x0000, 'y'); //write 'y' to the first addr of the eeprom to indicate that the cc already exists
	eeprom_write_byte(0x0001, 0);   //write initial voltage offset
}

uint8_t get_table_status(void){
	/* Table Status Byte:
	 *
	 *    7     6     5     4     3     2     1     0
	 * +-----+-----+-----+-----+-----+-----+-----+-----+
	 * |prdc?|            # of table entries           |
	 * +-----+-----+-----+-----+-----+-----+-----+-----+
	 */
	uint8_t index, m, p;
	m = 0;
	p = 0;
	for (index = 0; index < SI_TABLE_SIZE; index++) {
		if (table[index].info == SI_TABLE_VALUE_IS_MEASURED) {
			m++;
		}
		if (table[index].info == SI_TABLE_VALUE_IS_PREDICTED) {
			p++;
		}
	}
	if (p > 0) {
		m |= (1 << 7);
	}
	return m;
}


ISR(TIM1_OVF_vect) {
	/* Counter for HW-Reset */
	count_overflows++;
	TCNT1 = 0;	

	/* External Watchdog: Reset if > DEADLOCK_THRESHOLD*/
	//if ((count_overflows > DEADLOCK_THRESHOLD) && (req_buffer->rst_disable == 0x01)) {
	//if ((count_overflows > DEADLOCK_THRESHOLD) && (state != SI_DEBUG)){	
	if ((state != SI_DEBUG)){	
	rst_errors++;
		/* If the watchdog is triggered => reset main MCU */
		cli();
		SI_PULL_RESET_LINE();
		
		/* Only store values to the table when we are not in transient mode */			
		if(next_state != SI_TRANSIENT){
			current_index = ((uint8_t) (this_temperature) >> 1);//get hashed index of the table
			current_voltage -= SI_DEFAULT_VOLT_OFFSET;
			table[(current_index)].voltage = current_voltage; //updating table entry
			table[(current_index)].osccal = req_buffer->osccal;
			table[(current_index)].info = SI_TABLE_VALUE_IS_MEASURED;
			table_entries++;
			SI_REPLY_DEBUG_TABLE_ENTRY();
			/* Prediction Process:*/
			if (table_entries >= SI_PREDICTION_THRESHOLD) { //make prediction
				prediction();
				table_entries--; //decrease value by 1. If now any failure occur, we predict again
			}
		}
		_delay_ms(300);
		reset_voltage_level(); //secure instance as master
		i2c_slave_init(SLAVE_ADDR_ATTINY);	// secure instance as slave

		_delay_ms(300);
		/* The reset is stored as a failure event => table entry*/
		*lock_buffer = SI_LOCK();
		SI_RELEASE_RESET_LINE();

		/* Reset state machine */
		startup = SI_STARTUP_DELAY;
		state = SI_IDLE;
		next_state = SI_TRANSIENT;
		SI_REPLY_DEBUG_RESET();
		res_buffer->osccal = fact_default;
		SI_REPLY_VOLTAGE(SI_VOLT_REG_RESET);
		//ToDo: default voltage
		count_overflows = 0;
		/* Reset alternating byte */
		sei();	 
		this_altbyte = 0;
		*lock_buffer = SI_UNLOCK();
	}
}

