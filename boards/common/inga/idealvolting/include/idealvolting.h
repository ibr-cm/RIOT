/*
 * Copyright (C) 2017 Rasmus Antons <r.antons@tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    idealvolting IdealVolting implementation for RIOT
 * @ingroup     boards_inga_common
 * @brief       Idealvolting implementation.
 * @{
 *
 * @file
 * @brief       Idealvolting implementation.
 *
 * @author      Rasmus Antons <r.antons@tu-bs.de>
 *
 * @}
 */

#ifndef IDEALVOLTING_H
#define IDEALVOLTING_H

#include <stdint.h>

#define DEBUG_PRINT_REQ(req) printf("Request Frame:\n" \
		"    temperature = %u\n" \
		"    osccal      = %u\n" \
		"    rst_flags   = %x\n" \
		"    alt_byte    = %u\n" \
		"    rst_disable = %x\n", \
		(req)->temperature, \
		(req)->osccal, \
		(req)->rst_flags, \
		(req)->alt_byte, \
		(req)->rst_disable);

#define DEBUG_PRINT_RES(res) printf("Response Frame:\n" \
		"    osccal  = %u\n" \
		"    voltage = %u\n" \
		"    dt      = %u\n" \
		"    debug:\n" \
		"        state = %d\n" \
		"        table = %d\n" \
		"        flags = %s%s%s\n", \
		(res)->osccal, (res)->voltage, \
		(res)->dt_l | ((res)->dt_h << 8), \
		(res)->debug & (3), \
		((res)->debug >> 6) & (3), \
		(res)->debug & (1 << 3) ? "TABLE_ENTRY ": "", \
		(res)->debug & (1 << 4) ? "HARDWARE_RESET ": "", \
		(res)->debug & (1 << 5) ? "SOFTWARE_RESET ": "");

/**
 * Initialize IdealVolting
 *
 * This will start the idealvolting thread with priority 0
 * IdealVolting will be enabled immediately
 */
void idealvolting_init(void);

/**
 * Enable Idealvolting
 *
 * If it was disabled, IdealVolting will be enabled and return to
 * the minimal voltage.
 */
void idealvolting_enable(void);

/**
 * Disable Idealvolting
 *
 * This should raise the voltage to the initial value and disable
 * resets on the Tiny.
 * Currently this will only pause the idealvolting thread, and the
 * Tiny will reset the node.
 */
void idealvolting_disable(void);


/**
 * Set the debug mode state
 *
 * If the debug mode is enabled, every incoming frame will be printed out.
 *
 * @param state 0 to disable debug mode, non 0 to enable it
 */
void idealvolting_set_debug(uint8_t state);

/**
 * Print information about the current state of IdealVolting
 */
void idealvolting_print_status(void);

#endif /* IDEALVOLTING_H */
