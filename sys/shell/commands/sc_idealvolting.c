/*
 * Copyright (C) 2017 Rasmus Antons <r.antons@tu-bs.de>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     sys_shell_commands
 * @{
 *
 * @file
 * @brief       Shell commands for the IdealVolting implementation in
 *              boards_inga_common.
 *
 * @author      Rasmus Antons <r.antons@tu-bs.de>
 *
 * @}
 */

#include "idealvolting.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

int _idealvolting_handler(int argc, char **argv)
{
	(void) argc;
	(void) argv;

	if (argc == 1) {
		printf("Usage: %s \n"
				"        status\n"
				"        enable\n"
				"        disable\n"
				"        debug on|off\n",
				argv[0]);
		return 1;
	} else if (strcmp("status", argv[1]) == 0) {
		idealvolting_print_status();
	} else if (strcmp("wakeup", argv[1]) == 0) {
		idealvolting_wakeup();
		puts("Idealvolting is waking up.");
	} else if (strcmp("sleep", argv[1]) == 0) {
		uint8_t duration;
		if (argc > 2)
			duration = atoi(argv[2]);
		else
			duration = 3;
		printf("IdealVolting is going to sleep for %us.\n", duration);
		idealvolting_sleep(duration);
	} else {
		printf("Unknown command: %s\n", argv[1]);
	}

	return 0;
}
