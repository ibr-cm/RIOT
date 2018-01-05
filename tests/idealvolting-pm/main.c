/*
 * Copyright (C) 2017 Rasmus Antons <r.antons@tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Test application for the IdealVolting implementation in
 *              boards_inga_common.
 *
 * @author      Rasmus Antons <r.antons@tu-bs.de>
 *
 * @}
 */

#include <stdio.h>
#include <stdint.h>
#include "idealvolting.h"
#include "periph/uart.h"
#include "uart_stdio.h"
#include "periph/i2c.h"

#define DELAY(x) for (unsigned long i = 0; i < 65535 >> x; ++i) \
                	l = (double) i / (double) i + 0.00004

extern uint8_t pm_blocker[4];

int main(void)
{
	volatile double l = 0.0;
	(void) l;
	uart_poweroff(UART_STDIO_DEV);
	while (1) {
		uint8_t b = pm_blocker[3];
		//uart_poweron(UART_STDIO_DEV);
		printf("%d\n", b);
		idealvolting_print_status();
		DELAY(4);
		//uart_poweroff(UART_STDIO_DEV);
		DELAY(2);
	}
	return 0;
}
