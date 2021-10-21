/*
 * Copyright (C) 2014 Freie Universität Berlin, Hinnerk van Bruinehsen
 *               2021 Gerson Fernando Budke
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_avr8_common
 * @{
 *
 * @file
 * @brief       Startup code and interrupt vector definition
 *
 * @author      Hinnerk van Bruinehsen <h.v.bruinehsen@fu-berlin.de>
 * @author      Josua Arndt <jarndt@ias.rwth-aachen.de>
 * @author      Steffen Robertz <steffen.robertz@rwth-aachen.de>
 * @author      Gerson Fernando Budke <nandojve@gmail.com>
 * @}
 */

#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>

/* For Catchall-Loop */
#include "board.h"
#ifdef MODULE_PUF_SRAM
#include "puf_sram.h"
#endif
#ifdef MODULE_DBGPIN
#include "dbgpin.h"
#endif

/**
 * @brief functions for initializing the board, std-lib and kernel
 */
extern void board_init(void);
extern void kernel_init(void);
extern void __libc_init_array(void);


#ifdef MODULE_SRAM_UNDERVOLTING
//The Undervolting check needs to happen before any other part of the program can access the memory!
#include "sram_undervolting.h"
#include "uv_periph.h"
void init0_ovr(void) __attribute__((section(".init0")));
#endif

/**
 * @brief This pair of functions hook circumvent the call to main
 *
 * avr-libc normally uses the .init9 section for a call to main. This call
 * seems to be not replaceable without hacking inside the library. We
 * circumvent the call to main by using section .init7 to call the function
 * reset_handler which therefore is the real entry point and  section .init8
 * which should never be reached but just in case jumps to exit.
 * This way there should be no way to call main directly.
 */
void init7_ovr(void) __attribute__((section(".init7")));
void init8_ovr(void) __attribute__((section(".init8")));

__attribute__((used, naked)) void init7_ovr(void)
{
    __asm__ ("call reset_handler");
}

__attribute__((used, naked)) void init8_ovr(void)
{
    __asm__ ("jmp exit");
}

/**
 * @brief This function is the entry point after a system reset
 *
 * After a system reset, the following steps are necessary and carried out:
 * 1. initialize the board (sync clock, setup std-IO)
 * 2. initialize and start RIOTs kernel
 */
__attribute__((used)) void reset_handler(void)
{
#ifdef MODULE_PUF_SRAM
    puf_sram_init((uint8_t *)RAMEND-SEED_RAM_LEN, SEED_RAM_LEN);
#endif

#ifdef MODULE_DBGPIN
    dbgpin_init();
#endif

    /* initialize the board and startup the kernel */
    board_init();
    /* startup the kernel */
    kernel_init();
}

#ifdef MODULE_SRAM_UNDERVOLTING
//The SRAM Undervolting Routine needs to start before any data can be written into th memory!


/** Init3 happens before data is written to bss!
*
**/
__attribute__((used, naked)) void init0_ovr(void) {

	/**
	 * Disable JTAG to use the PINs for the UV HW
	**/
	MCUCR |= (1 << JTD);
	MCUCR |= (1 << JTD);

    /** Init Stack to Bottom of own Data Section. 
    *   The lower XX Bytes are not used!
    **/
    SPH = 0x40;
    SPL = 0xFF;

    /** Setting up these Pins should be handled in startup.
	*	They are essential to the system and should be hidden from the user!
	**/
	IIF_ENABLE_SETUP;
	IIF_SIG_SETUP;

	//__var_7 = 0x00;

	if (IIF_ENABLE_HIGH) {

		/** IF IIF_ENABLE == 1 and IIF_SIG == 0:
	  	*	OpAmp woke System, activate sleep mode (POWER_SAVE) immediately!
	 	**/
		if (IIF_SIG_LOW) {
			if(__var_7 == 0x02) {
				__var_7 = 0x06;
			} else {
				__var_7 = 0x01;
			}

			//sleep_bod_disable();
			do { \
			__asm__ __volatile__("in %[__temp_reg], %[mcucr]" "\n\t" \
								"ori %[__temp_reg], %[bods_bodse]" "\n\t" \
								"out %[mcucr], %[__temp_reg]" "\n\t" \
								"andi %[__temp_reg], %[not_bodse]" "\n\t" \
								"out %[mcucr], %[__temp_reg]" \
								: [__temp_reg] "=&d" (__temp_reg) \
								: [mcucr] "I" _SFR_IO_ADDR(BOD_CONTROL_REG), \
								[bods_bodse] "i" (_BV(BODS) | _BV(BODSE)), \
								[not_bodse] "i" (~_BV(BODSE))); \
			} while (0);

			MCUCR |= (1 << JTD);
			MCUCR |= (1 << JTD);
			//Clear Sleep reg
			SMCR &= 0x00;
			//Set sleepmode to POWER_SAVE
			set_sleep_mode(SLEEP_MODE_PWR_DOWN);
			sleep_enable(); //set Sleep Enable Bit

			sleep_cpu(); //go to sleep.
			__var_7 = 0x04;

		} else {
			__var_7 = 0x02;
			/** IRQ woke uC! **/
			POWER_MIKROC_SETUP;
			POWER_MIKROC_OFF; //Dont activate Power-Up Latch yet

			IIF_RESET_SETUP;
			IIF_RESET_OFF; //Dont Reset IIF yet

			POWER_MIKROC_ON;

			//Check if data is valid and restore context.
			undervolting_restore();

			//Data is invalid, start from main, reset IIF before.
			IIF_RESET_ON; //IRQ detected, reset IIF.
			//_delay_ms(10); //If Pulse is too short, the IIF wont reset!
			IIF_RESET_OFF;
		}
	} else {
		__var_7 = 0x03;
		__data_invalid = 0xA1;
		/** 
		*	IF IIF_ENABLE == 0: This is first Power-Up. Start with Main NOW!
		**/

		POWER_MIKROC_SETUP;
		POWER_MIKROC_OFF; //Dont activate Power-Up Latch yet

		IIF_RESET_SETUP;
		IIF_RESET_OFF; //Dont Reset IIF yet

		POWER_MIKROC_ON; /** Signal that uC is activ **/

	}

	/**
	*   Init Stack here! Stack is not inside of pattern!
	**/
    extern uint8_t __stack_start;
    /** Init Stack Pointer to start of pattern **/
    SPH = (((uint16_t)(&__stack_start)) & 0xFF00) >> 8;
    SPL = ((uint16_t)(&__stack_start)) & 0x00FF;
	
	gpio_update_restore_table();

	/** ==>End of startup sequence, goto main. **/

}
#endif

