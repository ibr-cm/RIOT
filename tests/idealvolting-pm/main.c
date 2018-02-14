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

#include <stdint.h>
#include <util/delay_basic.h>
#include "idealvolting.h"
#include "periph/uart.h"
#include "uart_stdio.h"
#include "periph/pm.h"

int main(void)
{
	uart_poweroff(UART_STDIO_DEV);
	while (1) {
		idealvolting_sleep(10);
	}
	return 0;
}
