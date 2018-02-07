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
#include "idealvolting.h"

extern uint8_t pm_blocker[4];

int main(void)
{
	while (1) {
		__builtin_avr_delay_cycles(8000000);
		idealvolting_sleep(0);
	}
	return 0;
}
